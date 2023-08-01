#ifdef ENABLE_BLE

#include "common.h"
#include "configuration.h"
#include "BluetoothLowEnergy.h"

#include "esp_system.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Defines
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#define BLE_NUM             1
#define BLE_APP_ID          0
#define PROFILE_NUM_HANDLE  3
#define BLE_TAG             "BLE_SENNA"

#define RGB_DEVICE_NAME           "ESP32 SENNA"
#define adv_config_flag           (1 << 0)


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Locals Prototypes
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static void BLE_Init(void);
static void BLE_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void BLE_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void BLE_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void BLE_adv_set_complete_handle(void);
static void BLE_adv_config(void);
static void BLE_crete_service(esp_gatt_if_t gatts_if);
static void BLE_handle_read_event(uint8_t * p_data, uint16_t len);
static void BLE_handle_write_event(uint8_t * p_data, uint16_t len);
static void BLE_adv_start(void);
static void BLE_start_service(esp_ble_gatts_cb_param_t *param);
static void BLE_add_characteristic(void);


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Variables
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

struct gatts_profile_inst {
  esp_gatts_cb_t gatts_cb;
  uint16_t gatts_if;
  uint16_t app_id;
  uint16_t conn_id;
  uint16_t service_handle;
  esp_gatt_srvc_id_t service_id;
  uint16_t char_handle;
  esp_bt_uuid_t char_uuid;
  esp_gatt_perm_t perm;
  esp_gatt_char_prop_t property;
};

// One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT 
static struct gatts_profile_inst gl_profile_tab[BLE_NUM] = {
  [BLE_APP_ID] = {
    .gatts_cb = BLE_gatts_profile_event_handler,
    .gatts_if = ESP_GATT_IF_NONE,       // Not get the gatt_if, so initial is ESP_GATT_IF_NONE 
  }
};

uint8_t rgb_service_uuid[ESP_UUID_LEN_128] = {0x00, 0x56, 0xa5, 0x97, 0xd2, 0xb7, 0x2e, 0x81, 0x57, 0x49, 0x00, 0x47, 0x6c, 0x55, 0x02, 0x3b};
uint8_t rgb_characteristic_uuid[ESP_UUID_LEN_128] = {0x00, 0x56, 0xa5, 0x97, 0xd2, 0xb7, 0x2e, 0x81, 0x57, 0x49, 0x00, 0x47, 0x6c, 0x55, 0x02, 0x3c};

static uint8_t adv_config_done = 0;

static esp_ble_adv_params_t adv_params = {
  .adv_int_min        = 0x20,
  .adv_int_max        = 0x40,
  .adv_type           = ADV_TYPE_IND,
  .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
  .channel_map        = ADV_CHNL_ALL,
  .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

QueueHandle_t queueBLE;


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Global Functions
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*- - BLE_Task() - - - - - - - - - - - - - - - - - - - - - - - - -*//*!

 \brief Task of Control Loop Position-QRESensor

 \note Attention, this task changes the setpoint based on the sector 
        and/or position

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void BLE_Task(void *pvParameter ){
  // initialization of task
  static GlobalData_t *BLE_global_data;
  static GlobalData_t BLE_local_data;
  memset(&BLE_local_data, 0, sizeof(GlobalData_t));
  BLE_global_data = (GlobalData_t *) pvParameter;
  
  char rxBuffer[50];
  queueBLE = xQueueCreate(10, sizeof(rxBuffer));

  BLE_Init();

  for(;;){

    // wait queue receive
    if (xQueueReceive(queueBLE, &rxBuffer, portMAX_DELAY) == true){

    }
  }  
}

/*- - BLE_Send() - - - - - - - - - - - - - - - - - - - - - - - - -*//*!

 \brief Send a msg to BLE

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void BLE_Send(char* Msg){
  xQueueSend(queueBLE, (void*)Msg, (TickType_t) 0);
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Local Functions
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*- - BLE_Init() - - - - - - - - - - - - - - - - - - - - - -*//*!

 \brief Initialization of BLE

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
static void BLE_Init(void){

  // Initialize bluetooth 
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bt_controller_init(&bt_cfg);
  esp_bt_controller_enable(ESP_BT_MODE_BLE);
  esp_bluedroid_init();
  esp_bluedroid_enable();

  // callbacks 
  esp_ble_gatts_register_callback(BLE_gatts_event_handler);
  esp_ble_gap_register_callback(BLE_gap_event_handler);
  esp_ble_gatts_app_register(BLE_APP_ID);

  esp_ble_gatt_set_local_mtu(500);
}

static void BLE_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
  // If event is register event, store the gatts_if for each profile 
  if (event == ESP_GATTS_REG_EVT) {
    if (param->reg.status == ESP_GATT_OK) {
      gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
    } else {
      #ifdef ENABLE_LOG
        ESP_LOGI(BLE_TAG, "Reg app failed, app_id %" PRIu16 ", status %d\n",
                param->reg.app_id,
                param->reg.status);
      #endif //ENABLE_LOG
      return;
    }
  }

  // If the gatts_if equal to profile A, call profile A cb handler,
  //   so here call each profile's callback 
  do {
    int idx;
    for (idx = 0; idx < BLE_NUM; idx++) {
      if (gatts_if == ESP_GATT_IF_NONE || // ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function 
              gatts_if == gl_profile_tab[idx].gatts_if) {
        if (gl_profile_tab[idx].gatts_cb) {
          gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
        }
      }
    }
  } while (0);
}

static void BLE_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
      BLE_adv_set_complete_handle();
      break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
      break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
      //advertising start complete event to indicate advertising start successfully or failed
      if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
        #ifdef ENABLE_LOG
        ESP_LOGE(BLE_TAG, "Advertising start failed\n");
        #endif //ENABLE_LOG
      }
      break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
      if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
        #ifdef ENABLE_LOG
        ESP_LOGE(BLE_TAG, "Advertising stop failed\n");
        #endif //ENABLE_LOG
      } else {
        #ifdef ENABLE_LOG
        ESP_LOGI(BLE_TAG, "Stop adv successfully\n");
        #endif //ENABLE_LOG
      }
      break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
      #ifdef ENABLE_LOG
      ESP_LOGI(BLE_TAG, "update connection params status = %d, min_int = %" PRIu16 ", max_int = %" PRIu16 ",conn_int = %" PRIu16 ",latency = %" PRIu16 ", timeout = %" PRIu16 "",
                param->update_conn_params.status,
                param->update_conn_params.min_int,
                param->update_conn_params.max_int,
                param->update_conn_params.conn_int,
                param->update_conn_params.latency,
                param->update_conn_params.timeout);
      #endif //ENABLE_LOG
      break;
    default:
      break;
  }
}

static void BLE_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
  switch (event) {
    case ESP_GATTS_REG_EVT:
      #ifdef ENABLE_LOG
      ESP_LOGI(BLE_TAG, "REGISTER_APP_EVT, status %d, app_id %" PRIu16 "\n", param->reg.status, param->reg.app_id);
      #endif //ENABLE_LOG
      BLE_adv_config();       
      BLE_crete_service(gatts_if);
      break;
    case ESP_GATTS_READ_EVT: {
      #ifdef ENABLE_LOG
      ESP_LOGI(BLE_TAG, "GATT_READ_EVT, conn_id %" PRIu16 ", trans_id %" PRIu32 ", handle %" PRIu16 "\n", param->read.conn_id, param->read.trans_id, param->read.handle);
      #endif //ENABLE_LOG

      #ifdef ENABLE_LOG
      ESP_LOGI(BLE_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
      esp_log_buffer_hex(BLE_TAG, param->write.value, param->write.len);
      #endif //ENABLE_LOG
      //BLE_handle_read_event(param->write.value, param->write.len);

      break;
    }
    case ESP_GATTS_WRITE_EVT: {
      #ifdef ENABLE_LOG
      ESP_LOGI(BLE_TAG, "GATT_WRITE_EVT, conn_id %" PRIu16 ", trans_id %" PRIu32 ", handle %" PRIu16 "", param->write.conn_id, param->write.trans_id, param->write.handle);
      #endif //ENABLE_LOG
      if (!param->write.is_prep){
          #ifdef ENABLE_LOG
          ESP_LOGI(BLE_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
          esp_log_buffer_hex(BLE_TAG, param->write.value, param->write.len);
          #endif //ENABLE_LOG
          BLE_handle_write_event(param->write.value, param->write.len);
      }
      esp_gatt_status_t status = ESP_GATT_OK;
      if (param->write.need_rsp){
          if (!param->write.is_prep){
              esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
          }
      }
      break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
      #ifdef ENABLE_LOG
      ESP_LOGI(BLE_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
      #endif //ENABLE_LOG
      break;
    case ESP_GATTS_MTU_EVT:
      #ifdef ENABLE_LOG
      ESP_LOGI(BLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
      #endif //ENABLE_LOG
      break;
    case ESP_GATTS_UNREG_EVT:
      break;
    case ESP_GATTS_CREATE_EVT:
      #ifdef ENABLE_LOG
      ESP_LOGI(BLE_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %" PRIu16 "\n", param->create.status, param->create.service_handle);
      #endif //ENABLE_LOG
      BLE_start_service(param);
      BLE_add_characteristic();
      break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
      break;
    case ESP_GATTS_ADD_CHAR_EVT: {
      #ifdef ENABLE_LOG
      ESP_LOGI(BLE_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %" PRIu16 ", service_handle %" PRIu16 "\n",
              param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
      #endif //ENABLE_LOG
      break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
      #ifdef ENABLE_LOG
      ESP_LOGI(BLE_TAG, "ADD_DESCR_EVT, status %d, attr_handle %" PRIu16 ", service_handle %" PRIu16 "\n",
                param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
      #endif //ENABLE_LOG
      break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
      #ifdef ENABLE_LOG
      ESP_LOGI(BLE_TAG, "SERVICE_START_EVT, status %d, service_handle %" PRIu16 "\n",
                param->start.status, param->start.service_handle);
      #endif //ENABLE_LOG
      break;
    case ESP_GATTS_STOP_EVT:
      break;
    case ESP_GATTS_CONNECT_EVT: {
      esp_ble_conn_update_params_t conn_params = {0};
      memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
      // For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. 
      conn_params.latency = 0;
      conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
      conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
      conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
      #ifdef ENABLE_LOG
      ESP_LOGI(BLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %" PRIu16 ", remote %02x:%02x:%02x:%02x:%02x:%02x:",
                param->connect.conn_id,
                param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
      #endif //ENABLE_LOG
      gl_profile_tab[BLE_APP_ID].conn_id = param->connect.conn_id;
      //start sent the update connection parameters to the peer device.
      esp_ble_gap_update_conn_params(&conn_params);
      break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
      #ifdef ENABLE_LOG
      ESP_LOGI(BLE_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
      #endif //ENABLE_LOG
      BLE_adv_start();
      break;
    case ESP_GATTS_CONF_EVT:
      #ifdef ENABLE_LOG
      ESP_LOGI(BLE_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %" PRIu16 "", param->conf.status, param->conf.handle);
      #endif //ENABLE_LOG
      if (param->conf.status != ESP_GATT_OK){
        esp_log_buffer_hex(BLE_TAG, param->conf.value, param->conf.len);
      }
      break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
      break;
  }
}

static void BLE_adv_set_complete_handle(void)
{
  adv_config_done &= (~adv_config_flag);
  if (adv_config_done == 0){
    esp_ble_gap_start_advertising(&adv_params);
  }
}

static void BLE_adv_config(void)
{
  esp_ble_adv_data_t adv_data = {
      .set_scan_rsp = false,
      .include_name = true,
      .include_txpower = false,
      .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
      .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
      .appearance = 0x00,
      .manufacturer_len = 0, 
      .p_manufacturer_data =  NULL, 
      .service_data_len = 0,
      .p_service_data = NULL,
      .service_uuid_len = 0,
      .p_service_uuid = NULL,
      .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
  };

  esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(RGB_DEVICE_NAME);
  if (set_dev_name_ret){
    #ifdef ENABLE_LOG
    ESP_LOGE(BLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
    #endif //ENABLE_LOG
  }
  
  //config adv data
  esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
  if (ret){
    #ifdef ENABLE_LOG
    ESP_LOGE(BLE_TAG, "config adv data failed, error code = %x", ret);
    #endif //ENABLE_LOG
  }
  adv_config_done |= adv_config_flag;
}

static void BLE_crete_service(esp_gatt_if_t gatts_if)
{
  gl_profile_tab[BLE_APP_ID].service_id.is_primary = true;
  gl_profile_tab[BLE_APP_ID].service_id.id.inst_id = 0x00;
  gl_profile_tab[BLE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_128;
  memcpy(gl_profile_tab[BLE_APP_ID].service_id.id.uuid.uuid.uuid128, rgb_service_uuid, ESP_UUID_LEN_128);

  esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[BLE_APP_ID].service_id, PROFILE_NUM_HANDLE);
}

static void BLE_handle_read_event(uint8_t * p_data, uint16_t len)
{
  uint8_t opcode = p_data[0];
  #ifdef ENABLE_LOG
  ESP_LOGI(BLE_TAG, "opcode = 0x%x", opcode);
  #endif //ENABLE_LOG
  switch (opcode)
  {
    case 1:

      // Copy global base to secure modify 
      // if(xSemaphoreTake(GlobalDataMutex, portMAX_DELAY)){
      //   memcpy(&BLE_local_data, BLE_global_data, sizeof(GlobalData_t));
      //   xSemaphoreGive(GlobalDataMutex);
      // }

      // Modify local base 


      // Set global base to secure modify 
      // if(xSemaphoreTake(GlobalDataMutex, portMAX_DELAY)){
      //   memcpy(BLE_global_data, &BLE_local_data, sizeof(GlobalData_t));
      //   xSemaphoreGive(GlobalDataMutex);
      // }
      // rgb_ctrl(RGB_RED, RGB_ON);
      // rgb_ctrl(RGB_GREEN, RGB_OFF);
      // rgb_ctrl(RGB_BLUE, RGB_OFF);
      break;
    default:
      break;
  }
}

static void BLE_handle_write_event(uint8_t * p_data, uint16_t len)
{
  uint8_t opcode = p_data[0];
  #ifdef ENABLE_LOG
  ESP_LOGI(BLE_TAG, "opcode = 0x%x", opcode);
  #endif //ENABLE_LOG
  switch (opcode)
  {
    case 1:

      // Copy global base to secure modify 
      // if(xSemaphoreTake(GlobalDataMutex, portMAX_DELAY)){
      //   memcpy(&BLE_local_data, BLE_global_data, sizeof(GlobalData_t));
      //   xSemaphoreGive(GlobalDataMutex);
      // }

      // Modify local base 


      // Set global base to secure modify 
      // if(xSemaphoreTake(GlobalDataMutex, portMAX_DELAY)){
      //   memcpy(BLE_global_data, &BLE_local_data, sizeof(GlobalData_t));
      //   xSemaphoreGive(GlobalDataMutex);
      // }
      // rgb_ctrl(RGB_RED, RGB_ON);
      // rgb_ctrl(RGB_GREEN, RGB_OFF);
      // rgb_ctrl(RGB_BLUE, RGB_OFF);
      break;
    default:
      break;
  }
}

static void BLE_adv_start(void)
{
  esp_ble_gap_start_advertising(&adv_params);
}

static void BLE_start_service(esp_ble_gatts_cb_param_t *param)
{ 
  gl_profile_tab[BLE_APP_ID].service_handle = param->create.service_handle;
  esp_ble_gatts_start_service(gl_profile_tab[BLE_APP_ID].service_handle);
}

static void BLE_add_characteristic(void)
{
  uint8_t char_value[] = {0x00};

  esp_attr_value_t char_val =
  {
    .attr_max_len = sizeof(char_value),
    .attr_len     = sizeof(char_value),
    .attr_value   = char_value,
  };

  gl_profile_tab[BLE_APP_ID].char_uuid.len = ESP_UUID_LEN_128;
  memcpy(gl_profile_tab[BLE_APP_ID].char_uuid.uuid.uuid128, rgb_characteristic_uuid, ESP_UUID_LEN_128);

  esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[BLE_APP_ID].service_handle, &gl_profile_tab[BLE_APP_ID].char_uuid,
                                                  ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                  ESP_GATT_CHAR_PROP_BIT_WRITE,
                                                  &char_val, NULL);
  if (add_char_ret){
    #ifdef ENABLE_LOG
    ESP_LOGE(BLE_TAG, "add char failed, error code =%x",add_char_ret);
    #endif //ENABLE_LOG
  }
}

#endif //ENABLE_BLE
