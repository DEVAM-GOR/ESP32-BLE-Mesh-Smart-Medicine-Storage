/* =========================================================
 * NODE 1 — LDR SENSOR NODE
 * ✅ BLE Mesh only (no WiFi, no MQTT)
 * ✅ LDR reads light → controls Relay + LED
 * ✅ Publishes LDR state to Gateway via Generic Level
 * ✅ Responds to ON/OFF from Gateway (Anedya command)
 * PINS: LDR→GPIO27  RELAY→GPIO5  LED→GPIO2  BUTTON→GPIO13
 * ========================================================= */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_system.h"

#define TAG "LDR_NODE"
#define LDR_GPIO     27
#define RELAY_GPIO    5
#define LED_GPIO      2
#define BUTTON_GPIO  13

static volatile bool overrideActive = false;
static QueueHandle_t gpio_evt_queue;
static int g_last_published_ldr = -1;

/* ================= BLE MESH SETUP ================= */
static uint8_t dev_uuid[16] = {
    0xdd,0xdd,0x00,0x01,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01
};
static esp_ble_mesh_cfg_srv_t config_server = {
    .relay=ESP_BLE_MESH_RELAY_ENABLED,
    .beacon=ESP_BLE_MESH_BEACON_ENABLED,
    .friend_state=ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    .gatt_proxy=ESP_BLE_MESH_GATT_PROXY_ENABLED,
    .default_ttl=7,
    .net_transmit=ESP_BLE_MESH_TRANSMIT(2,20),
    .relay_retransmit=ESP_BLE_MESH_TRANSMIT(2,20),
};
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub, 2+3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server = {
    .rsp_ctrl.get_auto_rsp=ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp=ESP_BLE_MESH_SERVER_RSP_BY_APP,
};
ESP_BLE_MESH_MODEL_PUB_DEFINE(level_pub, 2+3, ROLE_NODE);
static esp_ble_mesh_gen_level_srv_t level_server = {
    .rsp_ctrl.get_auto_rsp=ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp=ESP_BLE_MESH_SERVER_AUTO_RSP,
};
static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub,&onoff_server),
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&level_pub,&level_server),
};
static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0,root_models,ESP_BLE_MESH_MODEL_NONE),
};
static esp_ble_mesh_comp_t composition = {
    .cid=0x02E5,.elements=elements,.element_count=ARRAY_SIZE(elements)
};
static esp_ble_mesh_prov_t provision = {
    .uuid=dev_uuid,.output_size=0,.output_actions=0
};

/* ================= MESH PUBLISH + CONTROL ================= */
static void ensure_publish_addr(void)
{
    esp_ble_mesh_model_t *model = &root_models[2];
    if (model->pub->publish_addr == ESP_BLE_MESH_ADDR_UNASSIGNED) {
        model->pub->publish_addr = 0xC000;
        ESP_LOGI(TAG,"📌 Publish addr → 0xC000 (fallback)");
    }
}

static void mesh_publish_ldr(int ldr_state)
{
    if (ldr_state == g_last_published_ldr) return;
    esp_ble_mesh_model_t *model = &root_models[2];
    /* Force publish address if unassigned */
    if (model->pub->publish_addr == ESP_BLE_MESH_ADDR_UNASSIGNED) {
        model->pub->publish_addr = 0xC000;
    }
    int16_t val = (int16_t)ldr_state;
    esp_err_t err = esp_ble_mesh_model_publish(model,
        ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS,
        sizeof(int16_t),(uint8_t *)&val,ROLE_NODE);
    if (err == ESP_OK) { g_last_published_ldr = ldr_state; ESP_LOGI(TAG,"📡 LDR=%d → gateway",ldr_state); }
    else { ESP_LOGW(TAG,"Publish failed: 0x%x",err); }
}

static void apply_control(int ldr)
{
    if (overrideActive) { gpio_set_level(RELAY_GPIO,0); gpio_set_level(LED_GPIO,0); return; }
    if (ldr == 0) { /* BRIGHT → alert */
        gpio_set_level(RELAY_GPIO,1); gpio_set_level(LED_GPIO,1);
        ESP_LOGI(TAG,"BRIGHT → Relay ON"); mesh_publish_ldr(1);
    } else { /* DARK → safe */
        gpio_set_level(RELAY_GPIO,0); gpio_set_level(LED_GPIO,0);
        ESP_LOGI(TAG,"DARK → Relay OFF"); mesh_publish_ldr(0);
    }
}

/* ================= ISR + TASKS ================= */
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    int pin = (int)arg;
    xQueueSendFromISR(gpio_evt_queue,&pin,NULL);
}
static void event_task(void *arg)
{
    int pin, lastLdr=-1; TickType_t lastDebounce=0;
    while (1) {
        if (xQueueReceive(gpio_evt_queue,&pin,portMAX_DELAY)) {
            if ((xTaskGetTickCount()-lastDebounce)<pdMS_TO_TICKS(50)) continue;
            lastDebounce=xTaskGetTickCount();
            if (pin==BUTTON_GPIO) { overrideActive=true; apply_control(gpio_get_level(LDR_GPIO)); }
            if (pin==LDR_GPIO) {
                int ldr=gpio_get_level(LDR_GPIO);
                if (ldr!=lastLdr) { overrideActive=false; apply_control(ldr); lastLdr=ldr; }
            }
        }
    }
}

static void periodic_publish_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(5000)); // wait 5s after boot
    while (1) {
        if (esp_ble_mesh_node_is_provisioned()) {
            ensure_publish_addr();
            g_last_published_ldr = -1; // force re-publish
            apply_control(gpio_get_level(LDR_GPIO));
        }
        vTaskDelay(pdMS_TO_TICKS(10000)); // every 10 seconds
    }
}

/* ================= BLE MESH CALLBACKS ================= */
static void provisioning_cb(esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
        case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
            ESP_LOGI(TAG,"✅ Provisioned! Addr:0x%04x",param->node_prov_complete.addr);
            vTaskDelay(pdMS_TO_TICKS(2000));
            ensure_publish_addr();
            apply_control(gpio_get_level(LDR_GPIO));
            break;
        case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
            g_last_published_ldr=-1;
            esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV|ESP_BLE_MESH_PROV_GATT);
            break;
        default: break;
    }
}
static void generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event,
                               esp_ble_mesh_generic_server_cb_param_t *param)
{
    if (event==ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT &&
        param->model->model_id==ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV) {
        uint8_t onoff=param->value.set.onoff.onoff;
        ESP_LOGI(TAG,"MESH SET OnOff=%d",onoff);
        gpio_set_level(RELAY_GPIO,onoff); gpio_set_level(LED_GPIO,onoff);
        esp_ble_mesh_server_model_send_msg(param->model,&param->ctx,
            ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,sizeof(onoff),&onoff);
    }
}

/* ================= MAIN ================= */
static void gpio_init_custom(void)
{
    gpio_config_t in_conf = {
        .mode=GPIO_MODE_INPUT,
        .pin_bit_mask=(1ULL<<LDR_GPIO)|(1ULL<<BUTTON_GPIO),
        .pull_up_en=GPIO_PULLUP_ENABLE,.intr_type=GPIO_INTR_ANYEDGE,
    };
    gpio_config(&in_conf);
    gpio_set_direction(RELAY_GPIO,GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GPIO,GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_GPIO,0); gpio_set_level(LED_GPIO,0);
    gpio_evt_queue=xQueueCreate(10,sizeof(int));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(LDR_GPIO,gpio_isr_handler,(void*)LDR_GPIO);
    gpio_isr_handler_add(BUTTON_GPIO,gpio_isr_handler,(void*)BUTTON_GPIO);
}

void app_main(void)
{
    ESP_LOGI(TAG,"=== NODE1 LDR START ===");
    esp_err_t err=nvs_flash_init();
    if (err==ESP_ERR_NVS_NO_FREE_PAGES||err==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }

    /* ---- BUTTON-HOLD RESET: hold GPIO13 on boot for 3s to unprovision ---- */
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);
    if (gpio_get_level(BUTTON_GPIO) == 0) {
        ESP_LOGW(TAG,"Button held — waiting 3s for factory reset...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        if (gpio_get_level(BUTTON_GPIO) == 0) {
            ESP_LOGW(TAG,"🔴 FACTORY RESET — erasing mesh NVS!");
            nvs_flash_erase();
            nvs_flash_init();
            ESP_LOGW(TAG,"Done — rebooting as unprovisioned");
            esp_restart();
        }
    }
    /* ----------------------------------------------------------------------- */
    esp_bt_controller_config_t bt_cfg=BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    gpio_init_custom();

    esp_ble_mesh_register_prov_callback(provisioning_cb);
    esp_ble_mesh_register_generic_server_callback(generic_server_cb);
    err=esp_ble_mesh_init(&provision,&composition);
    if (err!=ESP_OK) { ESP_LOGE(TAG,"Mesh init failed"); return; }
    if (esp_ble_mesh_node_is_provisioned()) {
        ESP_LOGI(TAG,"✅ Already provisioned — NVS restored");
        ensure_publish_addr();
    } else {
        esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV|ESP_BLE_MESH_PROV_GATT);
        ESP_LOGI(TAG,"Waiting to be provisioned...");
    }
    apply_control(gpio_get_level(LDR_GPIO));
    xTaskCreate(event_task,"event_task",4096,NULL,5,NULL);
    xTaskCreate(periodic_publish_task,"periodic_pub",4096,NULL,4,NULL);
    ESP_LOGI(TAG,"=== NODE1 READY ===");
}
