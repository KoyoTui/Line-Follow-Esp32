#include "../include/common.h"
#include "../include/configuration.h"

#include "nvs_flash.h"

#include "PositionSensorControl.h"
#include "SpeedEncoderControl.h"
#include "SectorControl.h"
#include "BluetoothLowEnergy.h"
#include "Interface_GPIO.h"
#include "LogInFlash.h"

#include "esp_system.h"


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Defines
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Variables
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

enum CTL_State{
  CTL_INIT = 0, 
  CTL_WAIT,
  CTL_CALIBRATE,
  CTL_DEBUG0,
  CTL_RUN,
  CTL_STOP,
};

// estrutura global para compartilhamento de informa�oes entre tasks 
static GlobalData_t struct_global_data;

SemaphoreHandle_t GlobalDataMutex;
SemaphoreHandle_t ADC1Mutex;
SemaphoreHandle_t SEC_semaphore = NULL;
SemaphoreHandle_t PSC_semaphore = NULL;
SemaphoreHandle_t SCT_semaphore = NULL;

adc_oneshot_unit_handle_t ADCHandle = NULL;
gptimer_handle_t GPTIMERHandle = NULL;


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Locals Prototypes
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void global_init( void );
void global_data_default( GlobalData_t* Data );
void CTL_Task( void *pvParameter );


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Global Functions
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void app_main( void ) {
  memset(&struct_global_data, 0, sizeof(GlobalData_t));

  // Create Mutex 
  GlobalDataMutex = xSemaphoreCreateMutex();
  xSemaphoreGive(GlobalDataMutex);
  ADC1Mutex = xSemaphoreCreateMutex();
  xSemaphoreGive(ADC1Mutex);

	SEC_semaphore = xSemaphoreCreateBinary();
	PSC_semaphore = xSemaphoreCreateBinary();
	SCT_semaphore = xSemaphoreCreateBinary();

  global_init();

  global_data_default(&struct_global_data);

  // 0 is higher priority 

  // Tasks pinned to core 0  
  xTaskCreatePinnedToCore(PSC_Task, "psc_t", 16384, (void *) &struct_global_data, 5, NULL, 0);
  xTaskCreatePinnedToCore(CTL_Task, "ctl_t", 16384, (void *) &struct_global_data, 10, NULL, 0);
  xTaskCreatePinnedToCore(SCT_Task, "sct_t", 16384, (void *) &struct_global_data, 6, NULL, 0);
  xTaskCreatePinnedToCore(SEC_Task, "sec_t", 16384, (void *) &struct_global_data, 4, NULL, 0);

  // Tasks pinned to core 1  
  // keep interrupt in core 1
  #ifdef ENABLE_BLE
    xTaskCreatePinnedToCore(BLE_Task, "ble_t", 16384, (void *) &struct_global_data, 1, NULL, 1);
  #endif //ENABLE_BLE
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Local Functions
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void CTL_Task( void *pvParameter ){
  // initialization of task
  static GlobalData_t *CTL_global_data;
  CTL_global_data = (GlobalData_t *) pvParameter;
  static GlobalData_t CTL_local_data;

  uint64_t curr_timer_ms = 0;
  uint64_t set_timer_ms = 0;
  uint64_t led_timer_ms = 0;
  uint64_t psc_timer_ms = 0;
  uint64_t sct_timer_ms = 0;
  uint64_t sec_timer_ms = 0;

  uint8_t CTL_CurrState = 0;
  uint8_t CTL_NextState = 0;

  for(;;){

    // Copy global base to secure modify 
    if(xSemaphoreTake(GlobalDataMutex, portMAX_DELAY)){
      memcpy(&CTL_local_data, CTL_global_data, sizeof(GlobalData_t));

      gptimer_get_raw_count(GPTIMERHandle, &curr_timer_ms);

      curr_timer_ms /= 1000;

      CTL_local_data.counter.time = curr_timer_ms;

      switch (CTL_CurrState){
        case CTL_INIT:
          // wait 2 seconds
          set_timer_ms = curr_timer_ms + 5000;
          CTL_CurrState = CTL_WAIT;
          CTL_NextState = CTL_CALIBRATE;

          break;
        case CTL_WAIT:
          if(curr_timer_ms >= set_timer_ms){
            CTL_CurrState = CTL_NextState;
            set_timer_ms = 0;
          }

          // indicate this state
          if(curr_timer_ms >= led_timer_ms){
            led_timer_ms = curr_timer_ms + 1000;
            LED_OperToggle();
          }

          break;
        case CTL_CALIBRATE:
          // pass 5 seconds in this state
          if (!set_timer_ms){
            set_timer_ms = curr_timer_ms + 10000;
          }
          if (curr_timer_ms > set_timer_ms){
            set_timer_ms = curr_timer_ms + 5000;
            CTL_CurrState = CTL_WAIT;
            CTL_NextState = CTL_RUN;
          }

          // indicate this state
          if(curr_timer_ms >= led_timer_ms){
            led_timer_ms = curr_timer_ms + 500;
            LED_CommToggle();

            PSC_Calibration(&CTL_local_data);
            SCT_Calibration(&CTL_local_data);
          }

          break;
        case CTL_DEBUG0:
          // pass 5 seconds in this state
          if (!set_timer_ms){
            set_timer_ms = curr_timer_ms + 2000;
          }
          if (curr_timer_ms > set_timer_ms){
            set_timer_ms = curr_timer_ms + 3000;
            CTL_CurrState = CTL_STOP;
            CTL_NextState = CTL_STOP;
          }

          // indicate this state
          if(curr_timer_ms >= led_timer_ms){
            led_timer_ms = curr_timer_ms + 200;
            LED_SectorToggle();
          }

          xSemaphoreGive(PSC_semaphore);

          xSemaphoreGive(SCT_semaphore);

          xSemaphoreGive(SEC_semaphore);

          break;
        case CTL_RUN:

          LED_OperSet();

          // When passing the two markers on the right, it goes to CTL_STOP 
          if (CTL_local_data.counter.starter >= 99){
            CTL_CurrState = CTL_STOP;
            CTL_NextState = CTL_STOP;
            set_timer_ms = curr_timer_ms + 100;
          }

          // Every time the current time is a multiple of the time 
          // defined for that task to enter, give the semaphore of that task

          // if you don't have a defined time, never give the semaphore
          #ifdef SEC_TIME_TO_ENTER_MS
            if( curr_timer_ms >= sec_timer_ms ){
              sec_timer_ms = curr_timer_ms + SEC_TIME_TO_ENTER_MS;
              xSemaphoreGive(SEC_semaphore);
            }
          #endif

          #ifdef PSC_TIME_TO_ENTER_MS
            if( curr_timer_ms >= psc_timer_ms ){
              psc_timer_ms = curr_timer_ms + PSC_TIME_TO_ENTER_MS;
              xSemaphoreGive(PSC_semaphore);
            }
          #endif

          #ifdef SCT_TIME_TO_ENTER_MS
            if( curr_timer_ms >= sct_timer_ms ){
              sct_timer_ms = curr_timer_ms + SCT_TIME_TO_ENTER_MS;
              xSemaphoreGive(SCT_semaphore);
            }
          #endif

          break;
        case CTL_STOP:

          CTL_local_data.position_error = 0;
          CTL_local_data.pwm.setup = 0;

          if(curr_timer_ms >= led_timer_ms){
            led_timer_ms = curr_timer_ms + 1000;
            LED_OperToggle();
            LED_CommToggle();
            LED_SectorToggle();
          }

          xSemaphoreGive(SEC_semaphore);

          break;
        
        default:
          break;
      }

      memcpy(CTL_global_data, &CTL_local_data, sizeof(GlobalData_t));
      xSemaphoreGive(GlobalDataMutex);
    }
    
    vTaskDelay(1000);
  }  
}


void global_init( void ){
  nvs_flash_init();                          // 1 - Initialize NVS flash using

  gptimer_config_t timer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000, // 1MHz, 1 tick= 1us
  };
  gptimer_new_timer(&timer_config, &GPTIMERHandle);
  gptimer_enable(GPTIMERHandle);
  gptimer_start(GPTIMERHandle);

  GPIO_Init();

  /* adc1 initialization */  
  if(xSemaphoreTake(ADC1Mutex, portMAX_DELAY)){
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &ADCHandle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_11,
    };
    adc_oneshot_config_channel(ADCHandle, LATERAL_SENSOR_4_CH, &config);
    adc_oneshot_config_channel(ADCHandle, LATERAL_SENSOR_3_CH, &config);
    adc_oneshot_config_channel(ADCHandle, LATERAL_SENSOR_2_CH, &config);
    adc_oneshot_config_channel(ADCHandle, LATERAL_SENSOR_1_CH, &config);
    adc_oneshot_config_channel(ADCHandle, FRONTAL_SENSOR_CH, &config);
    xSemaphoreGive(ADC1Mutex);
  }

  gpio_reset_pin(BUZZER_PIN);
  /* Set the GPIO as a push/pull output */
  gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(BUZZER_PIN, 0);
}

void global_data_default( GlobalData_t* Data ){  
  Data->SEC_PID.kp = SEC_INIT_KP;
  Data->SEC_PID.ki = SEC_INIT_KI;
  Data->SEC_PID.kd = SEC_INIT_KD;
  
  Data->PSC_PID.kp = PSC_INIT_KP;
  Data->PSC_PID.ki = PSC_INIT_KI;
  Data->PSC_PID.kd = PSC_INIT_KD;

  Data->pwm.setup = MOTOR_PWM_SETUP;
  
  Data->global_state = STATE_ENABLE_MOTORS_MSK;
}

int remap_int(int uc_val, int uc_min, int uc_max, int ue_min, int ue_max){
  return ((uc_val - uc_min) * (ue_max - ue_min) / (uc_max - uc_min)) + ue_min;
}
