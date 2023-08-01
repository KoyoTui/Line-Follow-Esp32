#include "common.h"
#include "configuration.h"
#include "SectorControl.h"
#include "Interface_GPIO.h"
#include "Buzzer.h"


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Defines
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

typedef struct {
  int right_1;
  int right_2;
  int left_1;
  int left_2;
} LateralSensor_t;


enum LATERAL_SENSOR_STATUS{
  NONE_ACTIVE       = 0,
  RIGHT_MARK_ACTIVE = 1,
  LEFT_MARK_ACTIVE  = 2,
  BOTH_ACTIVE       = 3
};



/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Variables
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static portMUX_TYPE SCT_ADCRead_spinlock = portMUX_INITIALIZER_UNLOCKED;
#define SCT_TAG             "SCT_SENNA"


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Locals Prototypes
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void SCT_SensorInit(void);
void SCT_LateralRead(LateralSensor_t *curr_read);
void SCT_LateralReadCal(LateralSensor_t *curr_read, GlobalData_t *Data);
uint8_t SCT_LateralStatus( GlobalData_t *Data );
void SCT_ChangesBySector(GlobalData_t *Data);


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Global Functions
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*- - SCT_Task() - - - - - - - - - - - - - - - - - - - - - - - - -*//*!

 \brief Task of Sector Control

 \note Attention, this task changes the setpoint based on the sector 
        and/or position

 \note Attention, it is necessary to release the SCT_semaphore
        resource to release the task

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SCT_Task(void *pvParameter ){
  // initialization of task
  static GlobalData_t *SCT_global_data;
  static GlobalData_t SCT_local_data;
  memset(&SCT_local_data, 0, sizeof(GlobalData_t));
  SCT_global_data = (GlobalData_t *) pvParameter;

  uint8_t last_marker_status[3] = {0, 0, 0};

  SCT_SensorInit();

  for(;;){
    // the semaphore is given every X miliseconds 
    if(xSemaphoreTake(SCT_semaphore, portMAX_DELAY)){
      // code of task 

      // Copy global base to secure modify 
      if(xSemaphoreTake(GlobalDataMutex, portMAX_DELAY)){
        memcpy(&SCT_local_data, SCT_global_data, sizeof(GlobalData_t));
        xSemaphoreGive(GlobalDataMutex);
      }

      // Modify local base 

      uint8_t marker_status = SCT_LateralStatus(&SCT_local_data);

      if (last_marker_status[0] != marker_status){
        // pass last marker
        if (marker_status == NONE_ACTIVE){
          // we have 3 situations, marker, left, right marker and crossing

          // to be a left marker, must have gone from NONE_ACTIVE, to LEFT_MARK_ACTIVE and back to NONE_ACTIVE

          // to be a right marker, must have gone from NONE_ACTIVE, to RIGHT_MARK_ACTIVE and back to NONE_ACTIVE

          // to be a crossing, must have gone from NONE_ACTIVE || LEFT_MARK_ACTIVE || RIGHT_MARK_ACTIVE
          // to BOTH_ACTIVE || LEFT_MARK_ACTIVE || RIGHT_MARK_ACTIVE
          // and back to NONE_ACTIVE

          // left marker
          if (    ( last_marker_status[0] == LEFT_MARK_ACTIVE )
                && 
                  (   ( last_marker_status[1] == LEFT_MARK_ACTIVE ) 
                    || 
                      ( last_marker_status[1] == NONE_ACTIVE ) 
                  )
                && 
                  (   ( last_marker_status[2] == LEFT_MARK_ACTIVE ) 
                    || 
                      ( last_marker_status[2] == NONE_ACTIVE ) 
                  )
              ){            
            SCT_local_data.counter.sector += 1;
            
            LED_SectorToggle();

            #ifdef ENABLE_SCT_MKR_LOG
            ESP_LOGI(SCT_TAG, "Left Counter %d", SCT_local_data.counter.sector);
            #endif //ENABLE_SCT_MKR_LOG
          }

          // right marker
          if (    ( last_marker_status[0] == RIGHT_MARK_ACTIVE )
                && 
                  (   ( last_marker_status[1] == RIGHT_MARK_ACTIVE ) 
                    || 
                      ( last_marker_status[1] == NONE_ACTIVE ) 
                  )
                && 
                  (   ( last_marker_status[2] == RIGHT_MARK_ACTIVE ) 
                    || 
                      ( last_marker_status[2] == NONE_ACTIVE ) 
                  )
              ){
            SCT_local_data.counter.starter += 1;
            
            LED_CommToggle();

            #ifdef ENABLE_SCT_MKR_LOG
            ESP_LOGI(SCT_TAG, "Right Counter %d", SCT_local_data.counter.starter);
            #endif //ENABLE_SCT_MKR_LOG
          }
          
        }

      }

      // update last markers
      last_marker_status[2] = last_marker_status[1];
      last_marker_status[1] = last_marker_status[0];
      last_marker_status[0] = marker_status;

      SCT_ChangesBySector(&SCT_local_data);


      // Only write global data if it has been changed 
      if (memcmp(&SCT_local_data.counter, &SCT_global_data->counter, sizeof(GlobalCounterU64_t))){
        // Set global base to secure modify 
        if(xSemaphoreTake(GlobalDataMutex, portMAX_DELAY)){
          memcpy(&SCT_global_data->counter, &SCT_local_data.counter, sizeof(GlobalCounterU64_t));
          xSemaphoreGive(GlobalDataMutex);
        }
      }
    }
  }  
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Local Functions
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*- - SCT_SensorInit() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Initialization of QRE Sensors

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SCT_SensorInit(void){
}

/*- - SCT_LateralRead() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Acquires analog readings from side sensors

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SCT_LateralRead(LateralSensor_t *curr_read){
  if(xSemaphoreTake(ADC1Mutex, portMAX_DELAY)){
    
    taskENTER_CRITICAL(&SCT_ADCRead_spinlock);

    #if ADC_SAMPLES != 1
      int sum = 0;
      int raw_value = 0;

      for (uint8_t sample = 0; sample < ADC_SAMPLES; sample ++){
        adc_oneshot_read(ADCHandle, LATERAL_SENSOR_1_CH, &raw_value);
        sum += raw_value;
      }
      curr_read->right_1 = sum / ADC_SAMPLES;

      for (uint8_t sample = 0; sample < ADC_SAMPLES; sample ++){
        adc_oneshot_read(ADCHandle, LATERAL_SENSOR_2_CH, &raw_value);
        sum += raw_value;
      }
      curr_read->right_2 = sum / ADC_SAMPLES;

      for (uint8_t sample = 0; sample < ADC_SAMPLES; sample ++){
        adc_oneshot_read(ADCHandle, LATERAL_SENSOR_3_CH, &raw_value);
        sum += raw_value;
      }
      curr_read->left_1 = sum / ADC_SAMPLES;

      for (uint8_t sample = 0; sample < ADC_SAMPLES; sample ++){
        adc_oneshot_read(ADCHandle, LATERAL_SENSOR_4_CH, &raw_value);
        sum += raw_value;
      }
      curr_read->left_2 = sum / ADC_SAMPLES;

    #else
      adc_oneshot_read(ADCHandle, LATERAL_SENSOR_1_CH, &curr_read->right_1);
      adc_oneshot_read(ADCHandle, LATERAL_SENSOR_2_CH, &curr_read->right_2);
      adc_oneshot_read(ADCHandle, LATERAL_SENSOR_3_CH, &curr_read->left_1);
      adc_oneshot_read(ADCHandle, LATERAL_SENSOR_4_CH, &curr_read->left_2);
    #endif
    
    taskEXIT_CRITICAL(&SCT_ADCRead_spinlock);

    xSemaphoreGive(ADC1Mutex);
  }
}

/*- - SCT_LateralReadCal() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Acquires analog readings from side sensors

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SCT_LateralReadCal(LateralSensor_t *curr_read, GlobalData_t *Data){
    
  SCT_LateralRead(curr_read);

  if (curr_read->left_1 > Data->SCT_Cal.max){
    curr_read->left_1 = Data->SCT_Cal.max;
  } else if ((curr_read->left_1 < Data->SCT_Cal.min)) {
    curr_read->left_1 = Data->SCT_Cal.min;
  }

  if (curr_read->left_2 > Data->SCT_Cal.max){
    curr_read->left_2 = Data->SCT_Cal.max;
  } else if ((curr_read->left_2 < Data->SCT_Cal.min)) {
    curr_read->left_2 = Data->SCT_Cal.min;
  }

  if (curr_read->right_1 > Data->SCT_Cal.max){
    curr_read->right_1 = Data->SCT_Cal.max;
  } else if ((curr_read->right_1 < Data->SCT_Cal.min)) {
    curr_read->right_1 = Data->SCT_Cal.min;
  }

  if (curr_read->right_2 > Data->SCT_Cal.max){
    curr_read->right_2 = Data->SCT_Cal.max;
  } else if ((curr_read->right_2 < Data->SCT_Cal.min)) {
    curr_read->right_2 = Data->SCT_Cal.min;
  }

  #ifdef USE_RAILUX_INTERFACE
    #ifdef BLACK_LINE
      curr_read->left_1  = remap_int( curr_read->left_1, Data->SCT_Cal.min, Data->SCT_Cal.max, 0, 1000);
      curr_read->left_2  = remap_int( curr_read->left_2, Data->SCT_Cal.min, Data->SCT_Cal.max, 0, 1000);
      curr_read->right_1 = remap_int(curr_read->right_1, Data->SCT_Cal.min, Data->SCT_Cal.max, 0, 1000);
      curr_read->right_2 = remap_int(curr_read->right_2, Data->SCT_Cal.min, Data->SCT_Cal.max, 0, 1000);
    #else
      curr_read->left_1  = 1000 - remap_int( curr_read->left_1, Data->SCT_Cal.min, Data->SCT_Cal.max, 0, 1000);
      curr_read->left_2  = 1000 - remap_int( curr_read->left_2, Data->SCT_Cal.min, Data->SCT_Cal.max, 0, 1000);
      curr_read->right_1 = 1000 - remap_int(curr_read->right_1, Data->SCT_Cal.min, Data->SCT_Cal.max, 0, 1000);
      curr_read->right_2 = 1000 - remap_int(curr_read->right_2, Data->SCT_Cal.min, Data->SCT_Cal.max, 0, 1000);
    #endif
  #else // USE_RAILUX_INTERFACE
    #ifdef BLACK_LINE
      if (curr_read->left_1 < ( (Data->SCT_Cal.min + Data->SCT_Cal.max) / 2) )
        curr_read->left_1 = 0;
      else 
        curr_read->left_1 = 1000;

      if (curr_read->left_2 < ( (Data->SCT_Cal.min + Data->SCT_Cal.max) / 2) )
        curr_read->left_2 = 0;
      else 
        curr_read->left_2 = 1000;

      if (curr_read->right_1 < ( (Data->SCT_Cal.min + Data->SCT_Cal.max) / 2) )
        curr_read->right_1 = 0;
      else 
        curr_read->right_1 = 1000;

      if (curr_read->right_2 < ( (Data->SCT_Cal.min + Data->SCT_Cal.max) / 2) )
        curr_read->right_2 = 0;
      else 
        curr_read->right_2 = 1000;

    #else
      if (curr_read->left_1 > ( (Data->SCT_Cal.min + Data->SCT_Cal.max) / 2) )
        curr_read->left_1 = 0;
      else
        curr_read->left_1 = 1000;
        
      if (curr_read->left_2 > ( (Data->SCT_Cal.min + Data->SCT_Cal.max) / 2) )
        curr_read->left_2 = 0;
      else
        curr_read->left_2 = 1000;
        
      if (curr_read->right_1 > ( (Data->SCT_Cal.min + Data->SCT_Cal.max) / 2) )
        curr_read->right_1 = 0;
      else
        curr_read->right_1 = 1000;
        
      if (curr_read->right_2 > ( (Data->SCT_Cal.min + Data->SCT_Cal.max) / 2) )
        curr_read->right_2 = 0;
      else
        curr_read->right_2 = 1000;
    #endif

  #endif //USE_RAILUX_INTERFACE
}

/*- - SCT_LateralStatus() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Checks the state of global markers

 \return NONE_ACTIVE, RIGHT_MARK_ACTIVE, LEFT_MARK_ACTIVE or BOTH_ACTIVE
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
uint8_t SCT_LateralStatus( GlobalData_t *Data ){

  static LateralSensor_t buffer_values[LATERAL_CIRC_BUFF_LEN];
  #if LATERAL_CIRC_BUFF_LEN != 1
  static uint8_t circular_index = 0;
  #endif //LATERAL_CIRC_BUFF_LEN != 1
  LateralSensor_t avg_values;

  bool right_marker;
  bool left_marker;
      

  #if LATERAL_CIRC_BUFF_LEN != 1
    if (circular_index == LATERAL_CIRC_BUFF_LEN)
      circular_index = 0;
    else
      circular_index++;
  // get new values
  SCT_LateralReadCal(&buffer_values[circular_index], Data);
  #else
  // get new values
  SCT_LateralReadCal(&buffer_values[0], Data);
  #endif //LATERAL_CIRC_BUFF_LEN != 1

  #if LATERAL_CIRC_BUFF_LEN != 1
    for (uint8_t i = 0; i < LATERAL_CIRC_BUFF_LEN; i++) {
      avg_values.left_1  += buffer_values[i].left_1; 
      avg_values.left_2  += buffer_values[i].left_2; 
      avg_values.right_1 += buffer_values[i].right_1; 
      avg_values.right_2 += buffer_values[i].right_2; 
    }

    avg_values.left_1  = avg_values.left_1  / LATERAL_CIRC_BUFF_LEN;
    avg_values.left_2  = avg_values.left_2  / LATERAL_CIRC_BUFF_LEN;
    avg_values.right_1 = avg_values.right_1 / LATERAL_CIRC_BUFF_LEN;
    avg_values.right_2 = avg_values.right_2 / LATERAL_CIRC_BUFF_LEN;
  #else
    avg_values.left_1  = buffer_values[0].left_1;
    avg_values.left_2  = buffer_values[0].left_2;
    avg_values.right_1 = buffer_values[0].right_1;
    avg_values.right_2 = buffer_values[0].right_2;
  #endif //LATERAL_CIRC_BUFF_LEN != 1  

  left_marker = ( (avg_values.left_1 
  #ifdef WHITE_LINE
                                      >= 
  #else
                                      <= 
  #endif //WHITE_LINE
                                        LATERAL_THRESHOLD) 
  #ifdef LATERAL_LEFT_SERIAL_READ
                    && 
  #else
                    || 
  #endif //LATERAL_LEFT_SERIAL_READ
                  (avg_values.left_2 
  #ifdef WHITE_LINE
                                      >= 
  #else
                                      <= 
  #endif //WHITE_LINE
                                        LATERAL_THRESHOLD));

  right_marker = ( (avg_values.right_1 
  #ifdef WHITE_LINE
                                      >= 
  #else
                                      <= 
  #endif //WHITE_LINE
                                        LATERAL_THRESHOLD)
  #ifdef LATERAL_RIGHT_SERIAL_READ
                    && 
  #else
                    || 
  #endif //LATERAL_RIGHT_SERIAL_READ
                    (avg_values.right_2 
  #ifdef WHITE_LINE
                                        >= 
  #else
                                        <= 
  #endif //WHITE_LINE
                                          LATERAL_THRESHOLD));

  if (right_marker && left_marker) return BOTH_ACTIVE;
  if (!right_marker && left_marker) return LEFT_MARK_ACTIVE;
  if (right_marker && !left_marker) return RIGHT_MARK_ACTIVE;

  return NONE_ACTIVE;
}

/*- - SCT_ChangesBySector() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Changes to global parameters based on current sector

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SCT_ChangesBySector(GlobalData_t *Data){
  static uint16_t last_sector = 0;
  static uint64_t sector_timer = 0;
  static float avg_distance_sector = 0.0;

  // When entering a new sector, mark the time you entered it
  if (last_sector != Data->counter.sector){
    sector_timer = Data->counter.time;
    avg_distance_sector = Data->distance.avg;
  }

  switch (Data->counter.sector)
  {
  case 0:
    /* code */
    break;

  case 1:
    /* code */
    break;

  case 99:
    // // in first 200 ms of sector, is slower
    // if ( (sector_timer - Data->counter.sector) < 200 ){
    //   Data->rpm_setup = 1;
    // } else {
    //   // otherwise

    //   // after this, first 0,5 meters is faster
    //   if ( (Data->distance.avg - avg_distance_sector) < 0.5 ){
    //     Data->rpm_setup = 2;
    //   } else {
    //     // after this, break
    //     Data->rpm_setup = 0.5;
    //   }
    // }

    break;
  
  default:
    break;
  }
}

/*- - SCT_Calibration() - - - - - - - - - - - - - - - - - - -*//*!
 \brief Calibrate the frontal sensors

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SCT_Calibration( GlobalData_t *Data ){
  LateralSensor_t raw_values;

  SCT_LateralRead(&raw_values);

  #ifdef ENABLE_SCT_ADC_LOG
  ESP_LOGI(SCT_TAG, "lateral sensors: %d %d %d %d",
          raw_values.right_1, raw_values.right_2, raw_values.left_1, raw_values.left_2);
  #endif //ENABLE_SCT_ADC_LOG
  
  // if not calibrate, get some value
  if (Data->PSC_Cal.max == 0){
    Data->PSC_Cal.max = raw_values.left_1;
    Data->PSC_Cal.min = raw_values.left_1;
  }

  if (raw_values.left_1 > Data->SCT_Cal.max){
    Data->SCT_Cal.max = raw_values.left_1;
  } else if (raw_values.left_1 < Data->SCT_Cal.min){
    Data->SCT_Cal.min = raw_values.left_1;
    }
  
  if (raw_values.left_2 > Data->SCT_Cal.max){
    Data->SCT_Cal.max = raw_values.left_2;
  } else if (raw_values.left_2 < Data->SCT_Cal.min){
    Data->SCT_Cal.min = raw_values.left_2;
    }
  
  if (raw_values.right_1 > Data->SCT_Cal.max){
    Data->SCT_Cal.max = raw_values.right_1;
  } else if (raw_values.right_1 < Data->SCT_Cal.min){
    Data->SCT_Cal.min = raw_values.right_1;
    }
  
  if (raw_values.right_2 > Data->SCT_Cal.max){
    Data->SCT_Cal.max = raw_values.right_2;
  } else if (raw_values.right_2 < Data->SCT_Cal.min){
    Data->SCT_Cal.min = raw_values.right_2;
    }  
}
