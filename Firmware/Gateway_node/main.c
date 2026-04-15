/* =========================================================
 * GATEWAY ESP32 — BLE Mesh NODE + WiFi + Anedya MQTT
 * ========================================================= */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "esp_sntp.h"
#include "esp_gap_ble_api.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"

extern const uint8_t anedya_root_ca_pem_start[] asm("_binary_anedya_root_ca_pem_start");
extern const uint8_t anedya_root_ca_pem_end[] asm("_binary_anedya_root_ca_pem_end");
#define TAG "GATEWAY"
#define WIFI_SSID    "Bluetooth 1"
#define WIFI_PASS    "87654321"
#define ANEDYA_GW_NODE_ID   "DEVAM1010"
#define ANEDYA_GW_CONN_KEY  "3e0b3751c5b3ca067b28c3188602d4be"
#define ANEDYA_LDR_NODE_ID  "019d8048-9405-72d3-94ff-d5e41dd55faf"
#define ANEDYA_SRV_NODE_ID  "019d8049-5d1b-70a5-af40-2f6f0406c075"
#define ANEDYA_BROKER       "mqtts://mqtt.ap-in-1.anedya.io:8883"
#define TOPIC_GW_CMD     "$anedya/device/DEVAM1010/commands"
#define TOPIC_GW_CMD_UPD "$anedya/device/DEVAM1010/commands/updateStatus/json"
#define TOPIC_LDR_SUBMIT "$anedya/device/DEVAM1010/submitdata/json"
#define TOPIC_SRV_SUBMIT "$anedya/device/DEVAM1010/submitdata/json"
#define WIFI_CONNECTED_BIT   BIT0
#define MQTT_CONNECTED_BIT   BIT1
#define GROUP_NODE1   0xC000
#define GROUP_NODE2   0xC001

static EventGroupHandle_t events;
static esp_mqtt_client_handle_t mqtt;
static float g_temperature = 0, g_humidity = 0;
static int   g_ldr_state = 0, g_relay1_state = 0, g_relay2_state = 0;

static uint8_t dev_uuid[16] = {
    0xdd, 0xdd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03
};

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay             = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon            = ESP_BLE_MESH_BEACON_ENABLED,
    .friend_state      = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    .gatt_proxy        = ESP_BLE_MESH_GATT_PROXY_ENABLED,  // ← change this
    .default_ttl       = 7,
    .net_transmit      = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit  = ESP_BLE_MESH_TRANSMIT(2, 20),
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
};
static esp_ble_mesh_client_t onoff_client;
static esp_ble_mesh_client_t level_client;

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub, &onoff_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(NULL, &onoff_client),
    ESP_BLE_MESH_MODEL_GEN_LEVEL_CLI(NULL, &level_client),
};
static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};
static esp_ble_mesh_comp_t comp = {
    .cid = 0x02E5, .elements = elements, .element_count = ARRAY_SIZE(elements),
};
static esp_ble_mesh_prov_t prov = { .uuid = dev_uuid };

static uint8_t s_tid = 0;
void mesh_send_onoff(uint16_t group_addr, uint8_t state)
{
    esp_ble_mesh_client_common_param_t p = {0};
    esp_ble_mesh_generic_client_set_state_t s = {0};
    p.opcode = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK;
    p.model = &root_models[2];
    p.ctx.addr = group_addr; p.ctx.net_idx = 0; p.ctx.app_idx = 0; p.ctx.send_ttl = 7;
    s.onoff_set.onoff = state; s.onoff_set.tid = s_tid++;
    esp_err_t err = esp_ble_mesh_generic_client_set_state(&p, &s);
    ESP_LOGI(TAG, "Mesh→0x%04x=%d (%s)", group_addr, state, err == ESP_OK ? "OK" : "FAIL");
}

static void pub(const char *topic, const char *var, float val, int is_int)
{
    if (!(xEventGroupGetBits(events) & MQTT_CONNECTED_BIT)) return;
    char p[256];
    if (is_int) snprintf(p, sizeof(p), "{\"data\":[{\"variable\":\"%s\",\"value\":%d,\"timestamp\":0}]}", var, (int)val);
    else        snprintf(p, sizeof(p), "{\"data\":[{\"variable\":\"%s\",\"value\":%.1f,\"timestamp\":0}]}", var, val);
    esp_mqtt_client_publish(mqtt, topic, p, 0, 1, 0);
    ESP_LOGI(TAG, "☁️  %s → %.1f", var, val);
}
static void anedya_publish_ldr(int v)          { pub(TOPIC_LDR_SUBMIT, "ldr",         v, 1); }
static void anedya_publish_relay1(int v)        { pub(TOPIC_LDR_SUBMIT, "relay1",      v, 1); }
static void anedya_publish_temperature(float v) { pub(TOPIC_SRV_SUBMIT, "temperature", v, 0); }
static void anedya_publish_humidity(float v)    { pub(TOPIC_SRV_SUBMIT, "humidity",    v, 0); }
static void anedya_publish_relay2(int v)        { pub(TOPIC_SRV_SUBMIT, "relay2",      v, 1); }

static void anedya_ack_command(const char *id)
{
    if (!id) return;
    char p[256];
    snprintf(p, sizeof(p), "{\"commandId\":\"%s\",\"status\":\"success\"}", id);
    esp_mqtt_client_publish(mqtt, TOPIC_GW_CMD_UPD, p, 0, 1, 0);
}

static void mqtt_event_handler(void *a, esp_event_base_t b, int32_t id, void *d)
{
    esp_mqtt_event_handle_t ev = d;
    if (id == MQTT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "MQTT connected");
        xEventGroupSetBits(events, MQTT_CONNECTED_BIT);
        esp_mqtt_client_subscribe(mqtt, TOPIC_GW_CMD, 1);
    } else if (id == MQTT_EVENT_DISCONNECTED) {
        ESP_LOGW(TAG, "MQTT disconnected");
        xEventGroupClearBits(events, MQTT_CONNECTED_BIT);
    } else if (id == MQTT_EVENT_DATA) {
        char data[512] = {0};
        snprintf(data, sizeof(data), "%.*s", ev->data_len, ev->data);
        cJSON *root = cJSON_Parse(data);
        if (!root) return;

        cJSON *ci = cJSON_GetObjectItem(root, "commandId");
        cJSON *cd = cJSON_GetObjectItem(root, "data");

        const char *cid = ci ? ci->valuestring : NULL;

        if (cd && cd->valuestring) {
            cJSON *data_json = cJSON_Parse(cd->valuestring);
            if (data_json) {
                cJSON *cmd = cJSON_GetObjectItem(data_json, "command");
                if (cmd && cmd->valuestring) {
                    const char *name = cmd->valuestring;
                    ESP_LOGI(TAG, "Command received: %s", name);
                    if      (!strcmp(name, "node1_on"))  { mesh_send_onoff(GROUP_NODE1, 1); g_relay1_state = 1; anedya_publish_relay1(1); }
                    else if (!strcmp(name, "node1_off")) { mesh_send_onoff(GROUP_NODE1, 0); g_relay1_state = 0; anedya_publish_relay1(0); }
                    else if (!strcmp(name, "node2_on"))  { mesh_send_onoff(GROUP_NODE2, 1); g_relay2_state = 1; anedya_publish_relay2(1); }
                    else if (!strcmp(name, "node2_off")) { mesh_send_onoff(GROUP_NODE2, 0); g_relay2_state = 0; anedya_publish_relay2(0); }
                    else { ESP_LOGW(TAG, "Unknown command: %s", name); }
                }
                cJSON_Delete(data_json);
            }
        }
        anedya_ack_command(cid);
        cJSON_Delete(root);
    }
}


static void mqtt_init(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = ANEDYA_BROKER,
        .broker.verification.certificate = (const char *)anedya_root_ca_pem_start,
        .broker.verification.skip_cert_common_name_check = true,
        .credentials.client_id = ANEDYA_GW_NODE_ID,
        .credentials.username  = ANEDYA_GW_NODE_ID,
        .credentials.authentication.password = ANEDYA_GW_CONN_KEY,
        .network.timeout_ms = 20000, .network.reconnect_timeout_ms = 5000,
        .session.keepalive = 60,
    };
    mqtt = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt);
    xEventGroupWaitBits(events, MQTT_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

static void wifi_event_handler(void *a, esp_event_base_t b, int32_t id, void *d)
{
    if (id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected — reconnecting...");
        esp_wifi_connect();
        xEventGroupClearBits(events, WIFI_CONNECTED_BIT);
    } else if (id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "WiFi connected");
        xEventGroupSetBits(events, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,    wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wc = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_start();
    xEventGroupWaitBits(events, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

static void obtain_time(void)
{
    ESP_LOGI(TAG, "Syncing time...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "time.anedya.io");
    esp_sntp_setservername(1, "pool.ntp.org");
    esp_sntp_init();
    time_t now = 0; struct tm ti = {0}; int r = 0;
    while (ti.tm_year < (2020 - 1900) && r < 15) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now); localtime_r(&now, &ti); r++;
    }
    ESP_LOGI(TAG, "Time synced");
}

static void cloud_task(void *arg)
{
    ESP_LOGI(TAG, "Starting WiFi + MQTT...");
    wifi_init();
    ESP_LOGI(TAG, "WiFi OK");
    obtain_time();
    mqtt_init();
    ESP_LOGI(TAG, "MQTT OK — gateway fully ready!");
    vTaskDelete(NULL);
}

static void mesh_generic_client_cb(esp_ble_mesh_generic_client_cb_event_t event,
                                    esp_ble_mesh_generic_client_cb_param_t *param)
{
    if (event != ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT) return;
    uint16_t mid = param->params->model->model_id;
    uint16_t dst = param->params->ctx.recv_dst;
    ESP_LOGI(TAG, "Publish dst=0x%04x model=0x%04x", dst, mid);

    if (mid == ESP_BLE_MESH_MODEL_ID_GEN_LEVEL_CLI) {

    int16_t level = param->status_cb.level_status.present_level;

    // ===== NODE1 (LDR) =====
    if (dst == GROUP_NODE1) {
        static int last_ldr = -1;
        int current = (int)level;

        if (current != last_ldr) {
            g_ldr_state = current;
            last_ldr = current;

            ESP_LOGI(TAG, "LDR = %d", g_ldr_state);
            anedya_publish_ldr(g_ldr_state);
        }
    }
      // ===== NODE2 (TEMP + HUM) =====
    else if (dst == GROUP_NODE2) {

    float value = level / 10.0f;

    if (level >= 1000) {
        // HUMIDITY
        g_humidity = (level - 1000) / 10.0f;
        ESP_LOGI(TAG, "Hum=%.1f", g_humidity);
        anedya_publish_humidity(g_humidity);
    } else {
        // TEMPERATURE
        g_temperature = value;
        ESP_LOGI(TAG, "Temp=%.1f", g_temperature);
        anedya_publish_temperature(g_temperature);
    }
    
    }
}

    if (mid == ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI) {
        uint8_t onoff = param->status_cb.onoff_status.present_onoff;
        if (dst == GROUP_NODE1) { g_relay1_state = onoff; anedya_publish_relay1(onoff); }
        if (dst == GROUP_NODE2) { g_relay2_state = onoff; anedya_publish_relay2(onoff); }
    }
}

static void mesh_generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event,
                                    esp_ble_mesh_generic_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT &&
        param->model->model_id == ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV) {
        uint8_t onoff = param->value.set.onoff.onoff;
        ESP_LOGI(TAG, "nRF->Gateway OnOff=%d", onoff);
        esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(onoff), &onoff);
    }
}

static void mesh_prov_cb(esp_ble_mesh_prov_cb_event_t event,
                          esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "Provisioned! Addr=0x%04x", param->node_prov_complete.addr);
        ESP_LOGI(TAG, ">>> In nRF Mesh app:");
        ESP_LOGI(TAG, "    1. Bind AppKey to Level Client + OnOff Client");
        ESP_LOGI(TAG, "    2. Subscribe Level Client to 0xC000 and 0xC001");
        ESP_LOGI(TAG, "    3. Subscribe OnOff Client to 0xC000 and 0xC001");
        xTaskCreate(cloud_task, "cloud", 8192, NULL, 5, NULL);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGW(TAG, "Reset — re-advertising...");
        esp_ble_gap_set_device_name("ESP-BLE-MESH");
        esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
        break;
    default: break;
    }
}

static void ble_mesh_init(void)
{
    esp_ble_mesh_register_prov_callback(mesh_prov_cb);
    esp_ble_mesh_register_generic_client_callback(mesh_generic_client_cb);
    esp_ble_mesh_register_generic_server_callback(mesh_generic_server_cb);

    esp_err_t err = esp_ble_mesh_init(&prov, &comp);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Mesh init failed: %d", err); return; }

    if (esp_ble_mesh_node_is_provisioned()) {
        ESP_LOGI(TAG, "Already provisioned — starting cloud");
        xTaskCreate(cloud_task, "cloud", 8192, NULL, 5, NULL);
    } else {
        esp_err_t en = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
        ESP_LOGI(TAG, "prov_enable result: %d", en);
        ESP_LOGI(TAG, "Advertising — provision this device in nRF Mesh app");
        ESP_LOGI(TAG, "UUID: CC:CC:00:00:00:00:00:00:00:00:00:00:00:00:00:01");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== GATEWAY START ===");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    events = xEventGroupCreate();

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    /* Set name AFTER bluedroid enable, BEFORE mesh init */
    esp_ble_gap_set_device_name("ESP-BLE-MESH");

    ble_mesh_init();

    ESP_LOGI(TAG, "=== GATEWAY READY ===");
}