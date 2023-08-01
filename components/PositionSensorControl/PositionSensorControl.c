#include "common.h"
#include "configuration.h"
#include "PositionSensorControl.h"
#include <driver/gpio.h>


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Defines
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

typedef struct{
  int raw[FRONTAL_SENSOR_NUM_MAX];
} sensor;


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Variables
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static portMUX_TYPE GetRobotPosition_spinlock = portMUX_INITIALIZER_UNLOCKED;
#define PSC_TAG             "PSC_SENNA"


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Locals Prototypes
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void PSC_SensorInit(void);
float PSC_PIDPositionCalculateError( GlobalData_t *Data );
float PSC_GetRobotPosition( GlobalData_t *Data );
void PSC_FrontalRead( sensor *sensor_values );
void PSC_FrontalReadCal( sensor *sensor_values, GlobalData_t *Data );


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Global Functions
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*- - PSC_Task() - - - - - - - - - - - - - - - - - - - - - - - - -*//*!

 \brief Task of Control Loop Position-QRESensor

 \note Attention, it is necessary to release the PSC_semaphore
        resource to release the task

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void PSC_Task(void *pvParameter ){
  // initialization of task
  GlobalData_t *PSC_global_data;
  PSC_global_data = (GlobalData_t *) pvParameter;
  GlobalData_t PSC_local_data;
  memset(&PSC_local_data, 0, sizeof(GlobalData_t));

  PSC_SensorInit();

  for(;;){
    // the semaphore is given every X miliseconds 
    if(xSemaphoreTake(PSC_semaphore, portMAX_DELAY)){
      // code of task 

      // Copy global base to secure modify 
      if(xSemaphoreTake(GlobalDataMutex, portMAX_DELAY)){
        memcpy(&PSC_local_data, PSC_global_data, sizeof(GlobalData_t));
        xSemaphoreGive(GlobalDataMutex);
      }

      // Modify local base 
      PSC_local_data.position_error = PSC_PIDPositionCalculateError(&PSC_local_data);


      // Only write global data if it has been changed 
      if (memcmp(&PSC_local_data.position_error, &PSC_global_data->position_error, sizeof(float))){
        // Set global base to secure modify 
        if(xSemaphoreTake(GlobalDataMutex, portMAX_DELAY)){
          memcpy(&PSC_global_data->position_error, &PSC_local_data.position_error, sizeof(float));
          xSemaphoreGive(GlobalDataMutex);
        }
      }
    }
  }  
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Local Functions
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*- - PSC_SensorInit() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Initialization of QRE Sensors

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void PSC_SensorInit(void){
  // Configure the front sensors mux control pins   
  gpio_config_t io_conf = {}; //zero-initialize the config structure.
  
  io_conf.intr_type = GPIO_INTR_DISABLE; //disable interrupt
  io_conf.mode = GPIO_MODE_OUTPUT; //set as output mode
  io_conf.pin_bit_mask = FRONTAL_CTRL_MASK; //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pull_down_en = 0; //disable pull-down mode
  io_conf.pull_up_en = 0; //disable pull-up mode
  gpio_config(&io_conf); //configure GPIO with the given settings
}

/*- - PSC_PIDPositionCalculateError() - - - - - - - - - - - - - - *//*!

 \brief Calculate Position Error by PID

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
float PSC_PIDPositionCalculateError( GlobalData_t *Data ){
  static float acummulated_error = 0;
  static float last_error = 0;
  float current_position = PSC_GetRobotPosition(Data);

  #ifdef ENABLE_PSC_POS_LOG
  ESP_LOGI(PSC_TAG, "position: %f", current_position);
  #endif //ENABLE_PSC_POS_LOG

  float current_error = current_position - TARGET_SENSOR_READING;
  float error_delta =  current_error - last_error;
  acummulated_error += current_error;

  if(acummulated_error > TARGET_SENSOR_READING/ 10)
    acummulated_error = TARGET_SENSOR_READING/ 10;
  else 
    if(acummulated_error < -TARGET_SENSOR_READING/ 10)
      acummulated_error = -TARGET_SENSOR_READING/ 10;

  last_error = current_error;

  float result =  Data->PSC_PID.kp * ((float)current_error) + 
                  Data->PSC_PID.ki * ((float)acummulated_error) +
                  Data->PSC_PID.kd * ((float)error_delta);


  #ifdef ENABLE_PSC_PID_LOG
  ESP_LOGI(PSC_TAG, "position: %f", result );
  #endif //ENABLE_PSC_PID_LOG

  return result;
}

/*- - PSC_GetRobotPosition() - - - - - - - - - - - - - - - - - - -*//*!
 \brief Calculate the current position of robot

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
float PSC_GetRobotPosition( GlobalData_t *Data ){
  sensor sensor_values;

  bool onLine = false;
  float avg = 0;
  int sum = 0;
  static float _lastPosition = 0;

  PSC_FrontalReadCal(&sensor_values, Data);

  #ifdef ENABLE_PSC_ADC_LOG
  ESP_LOGI(PSC_TAG, "frontal sensors: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
          sensor_values.raw[0], sensor_values.raw[1], sensor_values.raw[2], sensor_values.raw[3],
          sensor_values.raw[4], sensor_values.raw[5], sensor_values.raw[6], sensor_values.raw[7], 
          sensor_values.raw[8], sensor_values.raw[9], sensor_values.raw[10], sensor_values.raw[11], 
          sensor_values.raw[12], sensor_values.raw[13], sensor_values.raw[14], sensor_values.raw[15] );
  #endif //ENABLE_PSC_ADC_LOG
  
  for (uint8_t i = 0; i < FRONTAL_SENSOR_NUM_MAX; i++){

    if (sensor_values.raw[i] >= (int)FRONTAL_THRESHOLD){
      onLine = true;
      avg += sensor_values.raw[i] * ( (i + 1) * 1000);
    }
    
    sum += sensor_values.raw[i];
  }
  
  if (!onLine) {
    if (_lastPosition < TARGET_SENSOR_READING){
      // If it last read to the left of center, return 0.
      return 0;
    }
    else{
      // If it last read to the right of center, return the max.
      return (FRONTAL_SENSOR_NUM_MAX + 1) * 1000;
    }
  }

  _lastPosition = avg / sum;  

  return _lastPosition;
}

/*- - PSC_FrontalRead() - - - - - - - - - - - - - - - - - - -*//*!
 \brief Read frontal sensors values

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void PSC_FrontalRead( sensor *sensor_values ){
  if(xSemaphoreTake(ADC1Mutex, portMAX_DELAY)){
    taskENTER_CRITICAL(&GetRobotPosition_spinlock);
    int raw_value = 0;
    
    for (uint8_t channel = ADC_CHANNEL_0; channel < FRONTAL_SENSOR_NUM_MAX; channel++){
      
      gpio_set_level(FRONTAL_CTRL_0_PIN, ( (channel & 0x1) >> 0 ));
      
      gpio_set_level(FRONTAL_CTRL_1_PIN, ( (channel & 0x2) >> 1 ));
      
      gpio_set_level(FRONTAL_CTRL_2_PIN, ( (channel & 0x4) >> 2 ));
      
      gpio_set_level(FRONTAL_CTRL_3_PIN, ( (channel & 0x8) >> 3 ));
      
      #if ADC_SAMPLES != 1
        int sum = 0;

        for (uint8_t sample = 0; sample < ADC_SAMPLES; sample ++){
          adc_oneshot_read(ADCHandle, FRONTAL_SENSOR_CH, &raw_value);
          sum += raw_value;
        }

        sensor_values->raw[channel] = sum / ADC_SAMPLES;
      #else
        adc_oneshot_read(ADCHandle, FRONTAL_SENSOR_CH, &raw_value);
        sensor_values->raw[channel] = raw_value;
      #endif
    }
    
    taskEXIT_CRITICAL(&GetRobotPosition_spinlock);

    xSemaphoreGive(ADC1Mutex);
  }
}

/*- - PSC_FrontalReadCal() - - - - - - - - - - - - - - - - - - -*//*!
 \brief Read frontal sensors values with calibration

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void PSC_FrontalReadCal( sensor *sensor_values, GlobalData_t *Data ){
  PSC_FrontalRead(sensor_values);
  #ifdef ENABLE_PSC_ADC_LOG
  ESP_LOGI(PSC_TAG, "cal frontal sensors: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
          sensor_values->raw[0], sensor_values->raw[1], sensor_values->raw[2], sensor_values->raw[3],
          sensor_values->raw[4], sensor_values->raw[5], sensor_values->raw[6], sensor_values->raw[7], 
          sensor_values->raw[8], sensor_values->raw[9], sensor_values->raw[10], sensor_values->raw[11], 
          sensor_values->raw[12], sensor_values->raw[13], sensor_values->raw[14], sensor_values->raw[15] );
  #endif //ENABLE_PSC_CAL_ADC_LOG

  for (uint8_t i = 0; i < FRONTAL_SENSOR_NUM_MAX; i++){
    if (sensor_values->raw[i] > Data->PSC_Cal.max){
      sensor_values->raw[i] = Data->PSC_Cal.max;
    } else if (sensor_values->raw[i] < Data->PSC_Cal.min) {
        sensor_values->raw[i] = Data->PSC_Cal.min;
    }

    #ifdef USE_RAILUX_INTERFACE
      #ifdef BLACK_LINE
        sensor_values->raw[i]  = remap_int( sensor_values->raw[i], Data->PSC_Cal.min, Data->PSC_Cal.max, 0, 1000);
        if (sensor_values->raw[i] > 1000) sensor_values->raw[i] = 1000;
        if (sensor_values->raw[i] < 0) sensor_values->raw[i] = 0;
      #else
        sensor_values->raw[i]  = 1000 - remap_int( sensor_values->raw[i], Data->PSC_Cal.min, Data->PSC_Cal.max, 0, 1000);
        if (sensor_values->raw[i] > 1000) sensor_values->raw[i] = 1000;
        if (sensor_values->raw[i] < 0) sensor_values->raw[i] = 0;
      #endif
    #else // USE_RAILUX_INTERFACE
      #ifdef BLACK_LINE
        if (sensor_values->raw[i] < ( (Data->PSC_Cal.min + Data->PSC_Cal.max) / 2) )
          sensor_values->raw[i] = 0;
        else 
          sensor_values->raw[i] = 1000;
      #else
        if (sensor_values->raw[i] > ( (Data->PSC_Cal.min + Data->PSC_Cal.max) / 2) )
          sensor_values->raw[i] = 0;
        else
          sensor_values->raw[i] = 1000;
      #endif

    #endif //USE_RAILUX_INTERFACE
  }

}

/*- - PSC_Calibration() - - - - - - - - - - - - - - - - - - -*//*!
 \brief Calibrate the frontal sensors

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void PSC_Calibration( GlobalData_t *Data ){
  sensor sensor_values;
  
  PSC_FrontalRead(&sensor_values);

  #ifdef ENABLE_PSC_ADC_LOG
  ESP_LOGI(PSC_TAG, "cal frontal sensors: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
          sensor_values.raw[0], sensor_values.raw[1], sensor_values.raw[2], sensor_values.raw[3],
          sensor_values.raw[4], sensor_values.raw[5], sensor_values.raw[6], sensor_values.raw[7], 
          sensor_values.raw[8], sensor_values.raw[9], sensor_values.raw[10], sensor_values.raw[11], 
          sensor_values.raw[12], sensor_values.raw[13], sensor_values.raw[14], sensor_values.raw[15] );
  #endif //ENABLE_PSC_CAL_ADC_LOG
  
  // if not calibrate, get some value
  if (Data->PSC_Cal.max == 0){
    Data->PSC_Cal.max = sensor_values.raw[12];
    Data->PSC_Cal.min = sensor_values.raw[12];
  }
  
  for (uint8_t i = 0; i < FRONTAL_SENSOR_NUM_MAX; i++){
    if (sensor_values.raw[i] > Data->PSC_Cal.max){
      Data->PSC_Cal.max = sensor_values.raw[i];

      #ifdef ENABLE_PSC_CAL_ADC_LOG
      ESP_LOGI(PSC_TAG, "new max cal sensor: %d", Data->PSC_Cal.max );
      #endif //ENABLE_PSC_CAL_ADC_LOG
    } else if (sensor_values.raw[i] < Data->PSC_Cal.min){
      Data->PSC_Cal.min = sensor_values.raw[i];

      #ifdef ENABLE_PSC_CAL_ADC_LOG
      ESP_LOGI(PSC_TAG, "new min cal sensor: %d", Data->PSC_Cal.min );
      #endif //ENABLE_PSC_CAL_ADC_LOG
      }
  }
}
