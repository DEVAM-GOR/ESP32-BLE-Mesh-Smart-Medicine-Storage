/* =========================================================
 * NODE 2 — SERVO + DHT22 SENSOR NODE
 * ✅ BLE Mesh only (no WiFi, no MQTT)
 * ✅ DHT22 reads temp+humidity → publishes to Gateway via Level
 * ✅ Auto servo if temp >= threshold
 * ✅ Responds to ON/OFF from Gateway
 * PINS: DHT22→GPIO4  RELAY→GPIO5  SERVO→GPIO18  LED→GPIO2
 * ========================================================= */
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_config_model_api.h"

#define TAG "NODE2_SERVO"
#define DHTPIN      GPIO_NUM_4
#define RELAY_PIN   GPIO_NUM_5
#define SERVO_PIN   GPIO_NUM_18
#define LED_PIN     GPIO_NUM_2
#define TEMP_THRESHOLD 30.0f
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_FREQ_HZ    50
#define LEDC_RESOLUTION LEDC_TIMER_14_BIT
#define SERVO_DUTY_0    410
#define SERVO_DUTY_90   1188

static volatile int  meshServoCmd=-1;
static volatile bool tempHighFlag=false,tempDropped=false;
static volatile bool servoRunning=false,relayOn=false;
static float g_temp=0.0f,g_hum=0.0f;
static int g_last_servo_state=-1;

/* ================= BLE MESH SETUP ================= */
static uint8_t dev_uuid[16]={0xEE,0xEE,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static esp_ble_mesh_cfg_srv_t config_server={
    .relay=ESP_BLE_MESH_RELAY_ENABLED,.beacon=ESP_BLE_MESH_BEACON_ENABLED,
    .friend_state=ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    .gatt_proxy=ESP_BLE_MESH_GATT_PROXY_ENABLED,.default_ttl=7,
    .net_transmit=ESP_BLE_MESH_TRANSMIT(2,20),.relay_retransmit=ESP_BLE_MESH_TRANSMIT(2,20),
};
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub,  2+3,ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server={
    .rsp_ctrl.get_auto_rsp=ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp=ESP_BLE_MESH_SERVER_RSP_BY_APP,
};
ESP_BLE_MESH_MODEL_PUB_DEFINE(temp_pub,2+3,ROLE_NODE);
static esp_ble_mesh_gen_level_srv_t temp_level_server={
    .rsp_ctrl.get_auto_rsp=ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp=ESP_BLE_MESH_SERVER_AUTO_RSP,
};
ESP_BLE_MESH_MODEL_PUB_DEFINE(hum_pub,2+3,ROLE_NODE);
static esp_ble_mesh_gen_level_srv_t hum_level_server={
    .rsp_ctrl.get_auto_rsp=ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp=ESP_BLE_MESH_SERVER_AUTO_RSP,
};
static esp_ble_mesh_model_t root_models[]={
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub,&onoff_server),
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&temp_pub,&temp_level_server),
};
static esp_ble_mesh_model_t secondary_models[]={
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&hum_pub,&hum_level_server),
};
static esp_ble_mesh_elem_t elements[]={
    ESP_BLE_MESH_ELEMENT(0,root_models,ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0,secondary_models,ESP_BLE_MESH_MODEL_NONE),
};
static esp_ble_mesh_comp_t composition={.cid=0x02E5,.elements=elements,.element_count=ARRAY_SIZE(elements)};
static esp_ble_mesh_prov_t provision={.uuid=dev_uuid,.output_size=0,.output_actions=0};

/* ================= MESH PUBLISH ================= */
static void ensure_publish_addr(void)
{
    esp_ble_mesh_model_t *tm=&root_models[2], *hm=&secondary_models[0];
    if (tm->pub->publish_addr==ESP_BLE_MESH_ADDR_UNASSIGNED) { tm->pub->publish_addr=0xC001; ESP_LOGI(TAG,"📌 Temp → 0xC001"); }
    if (hm->pub->publish_addr==ESP_BLE_MESH_ADDR_UNASSIGNED) { hm->pub->publish_addr=0xC001; ESP_LOGI(TAG,"📌 Hum  → 0xC001"); }
}
static void mesh_publish_temperature(float temp)
{
    if (!esp_ble_mesh_node_is_provisioned()) return;
    esp_ble_mesh_model_t *m=&root_models[2];
    if (m->pub->publish_addr==ESP_BLE_MESH_ADDR_UNASSIGNED) return;
    int16_t val=(int16_t)(temp*10.0f);
    if (esp_ble_mesh_model_publish(m,ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS,sizeof(int16_t),(uint8_t*)&val,ROLE_NODE)==ESP_OK)
        ESP_LOGI(TAG,"📡 Temp=%.1f→gateway",temp);
}
static void mesh_publish_humidity(float hum)
{
    if (!esp_ble_mesh_node_is_provisioned()) return;
    esp_ble_mesh_model_t *m=&secondary_models[0];
    if (m->pub->publish_addr==ESP_BLE_MESH_ADDR_UNASSIGNED) return;
    int16_t val=(int16_t)(1000 + hum*10.0f);
    if (esp_ble_mesh_model_publish(m,ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS,sizeof(int16_t),(uint8_t*)&val,ROLE_NODE)==ESP_OK)
        ESP_LOGI(TAG,"📡 Hum=%.1f→gateway",hum);
}
static void mesh_publish_servo(bool running)
{
    int state=running?1:0; if (state==g_last_servo_state) return;
    if (!esp_ble_mesh_node_is_provisioned()) return;
    esp_ble_mesh_model_t *m=&root_models[1];
    if (m->pub->publish_addr==ESP_BLE_MESH_ADDR_UNASSIGNED) return;
    uint8_t onoff=(uint8_t)state;
    if (esp_ble_mesh_model_publish(m,ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,sizeof(uint8_t),&onoff,ROLE_NODE)==ESP_OK)
        { g_last_servo_state=state; ESP_LOGI(TAG,"📡 Servo=%s",running?"ON":"OFF"); }
}

/* ================= DHT22 ================= */
static int dht_wait_level(int level,int timeout_us){int e=0;while(gpio_get_level(DHTPIN)!=level){if(e++>=timeout_us)return -1;esp_rom_delay_us(1);}return e;}
static esp_err_t dht22_read(float *t,float *h){
    uint8_t data[5]={0};
    gpio_set_direction(DHTPIN,GPIO_MODE_OUTPUT_OD);gpio_set_level(DHTPIN,0);vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(DHTPIN,1);esp_rom_delay_us(30);gpio_set_direction(DHTPIN,GPIO_MODE_INPUT);
    gpio_set_pull_mode(DHTPIN,GPIO_PULLUP_ONLY);esp_rom_delay_us(10);
    if(dht_wait_level(0,200)<0)return ESP_ERR_TIMEOUT;
    if(dht_wait_level(1,200)<0)return ESP_ERR_TIMEOUT;
    if(dht_wait_level(0,200)<0)return ESP_ERR_TIMEOUT;
    for(int i=0;i<40;i++){
        if(dht_wait_level(1,100)<0)return ESP_ERR_TIMEOUT;
        esp_rom_delay_us(45);data[i/8]<<=1;
        if(gpio_get_level(DHTPIN))data[i/8]|=1;
        if(dht_wait_level(0,100)<0)return ESP_ERR_TIMEOUT;
    }
    if(((data[0]+data[1]+data[2]+data[3])&0xFF)!=data[4])return ESP_ERR_INVALID_CRC;
    *h=((data[0]<<8)|data[1])/10.0f;
    *t=(((data[2]&0x7F)<<8)|data[3])/10.0f;
    if(data[2]&0x80)*t=-(*t);
    return ESP_OK;
}
/* ================= SERVO + RELAY ================= */
static void servo_init(void){
    ledc_timer_config_t t={.speed_mode=LEDC_MODE,.duty_resolution=LEDC_RESOLUTION,.timer_num=LEDC_TIMER,.freq_hz=LEDC_FREQ_HZ};ledc_timer_config(&t);
    ledc_channel_config_t ch={.gpio_num=SERVO_PIN,.speed_mode=LEDC_MODE,.channel=LEDC_CHANNEL,.timer_sel=LEDC_TIMER,.duty=SERVO_DUTY_0};ledc_channel_config(&ch);
}
static void servo_set(int duty){ledc_set_duty(LEDC_MODE,LEDC_CHANNEL,duty);ledc_update_duty(LEDC_MODE,LEDC_CHANNEL);}
static void set_relay(bool on){if(relayOn==on)return;relayOn=on;gpio_set_level(RELAY_PIN,on?1:0);gpio_set_level(LED_PIN,on?1:0);}

/* ================= CONTROL + TASKS ================= */
static void update_outputs(void){
    set_relay(tempHighFlag);
    bool shouldSweep;
    if(meshServoCmd==0)shouldSweep=false;
    else if(tempHighFlag)shouldSweep=true;
    else if(tempDropped){shouldSweep=false;meshServoCmd=-1;tempDropped=false;}
    else shouldSweep=(meshServoCmd==1);
    if(shouldSweep!=servoRunning){servoRunning=shouldSweep;mesh_publish_servo(servoRunning);}
}
static void servo_sweep_task(void *arg){
    bool dir=false;
    while(1){if(servoRunning){dir=!dir;servo_set(dir?SERVO_DUTY_90:SERVO_DUTY_0);vTaskDelay(pdMS_TO_TICKS(700));}
    else{servo_set(SERVO_DUTY_0);vTaskDelay(pdMS_TO_TICKS(200));}}
}
static void sensor_task(void *arg){
    while(1){
        float t,h;esp_err_t ret=ESP_FAIL;
        for(int i=0;i<3;i++){ret=dht22_read(&t,&h);if(ret==ESP_OK)break;vTaskDelay(pdMS_TO_TICKS(100));}
        if(ret==ESP_OK){
            g_temp=t;g_hum=h;ESP_LOGI(TAG,"Temp=%.1f°C  Hum=%.1f%%",t,h);
            mesh_publish_temperature(t);vTaskDelay(pdMS_TO_TICKS(300));mesh_publish_humidity(h);
            bool prevHigh=tempHighFlag;tempHighFlag=(t>=TEMP_THRESHOLD);
            if(!prevHigh&&tempHighFlag)update_outputs();
            else if(prevHigh&&!tempHighFlag){tempDropped=true;update_outputs();}
        }else ESP_LOGW(TAG,"DHT read failed (0x%x)",ret);
        vTaskDelay(pdMS_TO_TICKS(4700));
    }
}
/* ================= BLE MESH CALLBACKS ================= */
static void provisioning_cb(esp_ble_mesh_prov_cb_event_t event,esp_ble_mesh_prov_cb_param_t *param){
    switch(event){
        case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
            ESP_LOGI(TAG,"✅ Provisioned! Addr:0x%04x",param->node_prov_complete.addr);
            vTaskDelay(pdMS_TO_TICKS(2000));ensure_publish_addr();break;
        case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
            g_last_servo_state=-1;esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV|ESP_BLE_MESH_PROV_GATT);break;
        default:break;
    }
}
static void generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event,esp_ble_mesh_generic_server_cb_param_t *param){
    if(event==ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT&&
       param->model->model_id==ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV){
        uint8_t onoff=param->value.set.onoff.onoff;
        meshServoCmd=(int)onoff;update_outputs();
        esp_ble_mesh_server_model_send_msg(param->model,&param->ctx,
            ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,sizeof(onoff),&onoff);
    }
}

/* ================= MAIN ================= */
void app_main(void)
{
    ESP_LOGI(TAG,"=== NODE2 SERVO+DHT22 START ===");
    esp_err_t err=nvs_flash_init();
    if(err==ESP_ERR_NVS_NO_FREE_PAGES||err==ESP_ERR_NVS_NEW_VERSION_FOUND){nvs_flash_erase();nvs_flash_init();}

    /* ---- BOOT BUTTON (GPIO0) hold 3s on boot = factory reset ---- */
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_0, GPIO_PULLUP_ONLY);
    if (gpio_get_level(GPIO_NUM_0) == 0) {
        ESP_LOGW(TAG,"Boot button held — waiting 3s for factory reset...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        if (gpio_get_level(GPIO_NUM_0) == 0) {
            ESP_LOGW(TAG,"🔴 FACTORY RESET — erasing mesh NVS!");
            nvs_flash_erase(); nvs_flash_init();
            ESP_LOGW(TAG,"Done — rebooting as unprovisioned");
            esp_restart();
        }
    }
    /* -------------------------------------------------------------- */

    esp_bt_controller_config_t bt_cfg=BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();esp_bluedroid_enable();

    gpio_set_direction(RELAY_PIN,GPIO_MODE_OUTPUT);gpio_set_level(RELAY_PIN,0);
    gpio_set_direction(LED_PIN,GPIO_MODE_OUTPUT);gpio_set_level(LED_PIN,0);
    gpio_set_direction(DHTPIN,GPIO_MODE_INPUT);gpio_set_pull_mode(DHTPIN,GPIO_PULLUP_ONLY);
    servo_init();

    esp_ble_mesh_register_prov_callback(provisioning_cb);
    esp_ble_mesh_register_generic_server_callback(generic_server_cb);
    err=esp_ble_mesh_init(&provision,&composition);
    if(err!=ESP_OK){ESP_LOGE(TAG,"Mesh init failed");return;}
    if(esp_ble_mesh_node_is_provisioned()){
        ESP_LOGI(TAG,"✅ Already provisioned — NVS restored");
        ensure_publish_addr();
    }else{
        esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV|ESP_BLE_MESH_PROV_GATT);
        ESP_LOGI(TAG,"Waiting to be provisioned...");
    }
    xTaskCreate(servo_sweep_task,"servo_sweep",2048,NULL,4,NULL);
    xTaskCreate(sensor_task,"sensor",4096,NULL,5,NULL);
    ESP_LOGI(TAG,"=== NODE2 READY — threshold=%.1f°C ===",TEMP_THRESHOLD);
}
