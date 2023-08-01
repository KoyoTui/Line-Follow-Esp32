#include "SpeedEncoderControl.h"
#include "common.h"
#include "configuration.h"
#include <driver/gpio.h>


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Defines
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */


#define MAGIC_NUMBER                    0.1111114

#define SEC_TAG                   "SEC_SENNA"


enum SEC_MotorState{
  SEC_MotorNotInit = 0,
  SEC_MotorStop,
  SEC_MotorFoward,
  SEC_MotorBackward,
};


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Variables
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

typedef struct {
  uint16_t    a_pin;
  uint16_t    b_pin;
  int32_t     count;
  uint8_t     enable;
} SECEncData_t;

volatile SECEncData_t SEC_ENCA_data;
volatile SECEncData_t SEC_ENCB_data;

portMUX_TYPE DRAM_ATTR ENCADataMutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE DRAM_ATTR ENCBDataMutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE DRAM_ATTR SEC_PWMMutex = portMUX_INITIALIZER_UNLOCKED;

uint8_t SEC_MotorRState = SEC_MotorNotInit;
uint8_t SEC_MotorLState = SEC_MotorNotInit;


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Locals Prototypes
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void SEC_RefreshDutyCycle( GlobalData_t *local_data );
void SEC_EncoderInit( void );
void SEC_MotorInit( void );
void SEC_ISR_ENCA_A( void *pvParameter );
void SEC_ISR_ENCB_A( void *pvParameter );
void SEC_ISR_ENCA_B( void *pvParameter );
void SEC_ISR_ENCB_B( void *pvParameter );
void SEC_PIDRightRPMCalculateError( GlobalData_t *Data, float Curr_RPM );
void SEC_PIDLeftRPMCalculateError( GlobalData_t *Data, float Curr_RPM );


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Global Functions
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*- - SEC_Task() - - - - - - - - - - - - - - - - - - - - - - - - -*//*!

 \brief Task of Control Loop Speed-Encoder

 \note Attention, it is necessary to release the SEC_semaphore
        resource to release the task

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SEC_Task(void *pvParameter ){
  // initialization of task
  GlobalData_t *SEC_global_data;
  GlobalData_t SEC_local_data;
  memset(&SEC_local_data, 0, sizeof(GlobalData_t));
  SEC_global_data = (GlobalData_t *) pvParameter;

  float curr_left_rpm = 0;
  float curr_right_rpm = 0;
  float curr_left_count = 0;
  float curr_right_count = 0;

  SEC_EncoderInit();

  SEC_MotorInit();

  for (;;){

    // the semaphore is given every X miliseconds 
    if(xSemaphoreTake(SEC_semaphore, portMAX_DELAY)){
      // code of task 

      // Copy global base to secure modify 
      if(xSemaphoreTake(GlobalDataMutex, portMAX_DELAY)){
        memcpy(&SEC_local_data, SEC_global_data, sizeof(GlobalData_t));
        xSemaphoreGive(GlobalDataMutex);
      }

      // Modify local base 
      
      // get encoder A count 
      portENTER_CRITICAL(&ENCADataMutex);
      curr_left_count = SEC_ENCA_data.count;
      SEC_ENCA_data.count = 0;
      portEXIT_CRITICAL(&ENCADataMutex);
      
      // get encoder B count 
      portENTER_CRITICAL(&ENCBDataMutex);
      curr_right_count = SEC_ENCB_data.count;
      SEC_ENCB_data.count = 0;
      portEXIT_CRITICAL(&ENCBDataMutex);

      // Update distance count 
      // (pulses) / (pulses in a revolution) -> x revolution
      // x * circumference -> travelled distance
      //                                 2 * pi       *      0,024         *       -10        /       45
      SEC_local_data.distance.right += ( ( 2 * PI * (WHEEL_RADIUS / 1000.0) * curr_right_count / PPR_WHEEL) ) * MAGIC_NUMBER;
      SEC_local_data.distance.left  += ( ( 2 * PI * (WHEEL_RADIUS / 1000.0) *  curr_left_count / PPR_WHEEL) ) * MAGIC_NUMBER;
      SEC_local_data.distance.avg = (SEC_local_data.distance.right + SEC_local_data.distance.left) / 2;

      #ifdef ENABLE_SEC_DTC_LOG
        ESP_LOGI(SEC_TAG, "Distance total L: %f R: %f", SEC_local_data.distance.left, SEC_local_data.distance.right);
      #endif //ENABLE_SEC_DTC_LOG

      #ifdef ENABLE_SEC
        // Convert pulses to rpm
        // (pulse/ms) * (1000 ms) / (1s) * (60s) / (1 min) -> (pulses / min)
        curr_left_rpm  = ( curr_left_count / SEC_TIME_TO_ENTER_MS) * 60.0 * 1000.0;
        curr_right_rpm = (curr_right_count / SEC_TIME_TO_ENTER_MS) * 60.0 * 1000.0;

        #ifdef ENABLE_SEC_RPM_LOG
        ESP_LOGI(SEC_TAG, "Curr RPM L: %f, R: %f", curr_left_rpm, curr_right_rpm);
        #endif //ENABLE_SEC_RPM_LOG

        // Checks if the engines must be enabled
        if ( !(SEC_local_data.global_state & STATE_ENABLE_MOTORS_MSK)){
          SEC_local_data.rpm.left.curr = 0;
          SEC_local_data.rpm.right.curr = 0;
        } else {
          SEC_PIDLeftRPMCalculateError(&SEC_local_data,  curr_left_rpm);
          SEC_PIDRightRPMCalculateError(&SEC_local_data, curr_right_rpm);
        }

        SEC_local_data.pwm.right = ((SEC_local_data.rpm_setup + SEC_local_data.rpm_error.right) * RPM_TO_DUTY);
        SEC_local_data.pwm.left = ((SEC_local_data.rpm_setup + SEC_local_data.rpm_error.left) * RPM_TO_DUTY);
      #else
        if (SEC_local_data.position_error > MOTOR_TIMER_MAX_DUTY){
          SEC_local_data.pwm.right = SEC_local_data.pwm.setup + MOTOR_TIMER_MAX_DUTY;
          SEC_local_data.pwm.left  = SEC_local_data.pwm.setup - MOTOR_TIMER_MAX_DUTY;
        } else if (SEC_local_data.position_error < MOTOR_TIMER_MIN_DUTY){
          SEC_local_data.pwm.right = SEC_local_data.pwm.setup + MOTOR_TIMER_MIN_DUTY;
          SEC_local_data.pwm.left  = SEC_local_data.pwm.setup - MOTOR_TIMER_MIN_DUTY;
        } else {
          SEC_local_data.pwm.right = SEC_local_data.pwm.setup + SEC_local_data.position_error;
          SEC_local_data.pwm.left  = SEC_local_data.pwm.setup - SEC_local_data.position_error;
        }

      #ifdef ENABLE_SEC_PWM_LOG
        ESP_LOGI(SEC_TAG, "PWM L: %d - %f", SEC_local_data.pwm.setup, SEC_local_data.position_error);
      #endif //ENABLE_SEC_DTC_LOG

        if (SEC_local_data.pwm.right > MOTOR_TIMER_MAX_DUTY) SEC_local_data.pwm.right = MOTOR_TIMER_MAX_DUTY;
        else if (SEC_local_data.pwm.right < MOTOR_TIMER_MIN_DUTY) SEC_local_data.pwm.right = MOTOR_TIMER_MIN_DUTY;

        if (SEC_local_data.pwm.left > MOTOR_TIMER_MAX_DUTY) SEC_local_data.pwm.left = MOTOR_TIMER_MAX_DUTY;
        else if (SEC_local_data.pwm.left < MOTOR_TIMER_MIN_DUTY) SEC_local_data.pwm.left = MOTOR_TIMER_MIN_DUTY;
      #endif //ENABLE_SEC

      #ifdef ENABLE_SEC_PWM_LOG
        ESP_LOGI(SEC_TAG, "PWM L: %d R: %d", SEC_local_data.pwm.left, SEC_local_data.pwm.right);
      #endif //ENABLE_SEC_DTC_LOG

      #ifdef ENABLE_MOTOR
        // Checks if the engines must be enabled
        if ( SEC_local_data.global_state & STATE_ENABLE_MOTORS_MSK ){
          // Refresh motors inputs
          SEC_RefreshDutyCycle(&SEC_local_data);
        }
      #endif // ENABLE_MOTOR

      // Only write global data if it has been changed
      if (memcmp(&SEC_local_data.distance, &SEC_global_data->distance, (sizeof(GlobalPWM_t) + sizeof(GlobalRigLefAvgF_t)))){
        // Set global base to secure modify
        if(xSemaphoreTake(GlobalDataMutex, portMAX_DELAY)){
          memcpy(&SEC_global_data->distance, &SEC_local_data.distance, (sizeof(GlobalPWM_t) + sizeof(GlobalRigLefAvgF_t)));
          xSemaphoreGive(GlobalDataMutex);
        }
      }
    }
  }  
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Local Functions
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*- - SEC_RefreshDutyCycle() - - - - - - - - - - - - - - - - - - -*//*!
 \brief Refresh Duty Cycle of DRV pins

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SEC_RefreshDutyCycle( GlobalData_t *local_data ){
  if (local_data->pwm.right > 0){

    ledc_set_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INA_1_CH, local_data->pwm.right);
    ledc_set_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INA_2_CH, 0);
  } else if (local_data->pwm.right < 0){

    ledc_set_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INA_1_CH, 0);
    ledc_set_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INA_2_CH, -local_data->pwm.right);
  } else {

    ledc_set_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INA_1_CH, MOTOR_TIMER_MAX_DUTY);
    ledc_set_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INA_2_CH, MOTOR_TIMER_MAX_DUTY);
  }

  if (local_data->pwm.left > 0){

    ledc_set_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INB_1_CH, local_data->pwm.left);
    ledc_set_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INB_2_CH, 0);
  } else if (local_data->pwm.left < 0){

    ledc_set_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INB_1_CH, 0);
    ledc_set_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INB_2_CH, -local_data->pwm.left);
  } else {

    ledc_set_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INB_1_CH, MOTOR_TIMER_MAX_DUTY);
    ledc_set_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INB_2_CH, MOTOR_TIMER_MAX_DUTY);
  }

  // Update duty to apply the new value
  vPortEnterCriticalSafe(&SEC_PWMMutex);
  gpio_set_level(DRV_ENABLE_A_PIN, true);
  gpio_set_level(DRV_ENABLE_B_PIN, true);

  ledc_update_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INA_1_CH);
  ledc_update_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INA_2_CH);
  ledc_update_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INB_1_CH);
  ledc_update_duty(MOTOR_TIMER_MODE, MOTOR_TIMER_INB_2_CH);
  
  vPortExitCriticalSafe(&SEC_PWMMutex);
}

/*- - SEC_PIDLeftRPMCalculateError() - - - - - - - - - - - - - - *//*!

 \brief Calculate Rpm Error by PID

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SEC_PIDLeftRPMCalculateError( GlobalData_t *Data, float Curr_RPM ){
  float target_rpm = Data->rpm_setup + Data->position_error;

  static float acummulated_error = 0;
  static float last_error = 0;
  float current_error = Curr_RPM - target_rpm;
  float error_delta =  current_error - last_error;
  acummulated_error += current_error;

  if(acummulated_error > (target_rpm / 10) )
    acummulated_error = (target_rpm / 10);
  else 
    if(acummulated_error < -(target_rpm / 10) )
      acummulated_error = -(target_rpm / 10);

  last_error = current_error;

  float result =  Data->SEC_PID.kp * ( (float)current_error ) + 
                  Data->SEC_PID.ki * ( (float)error_delta )   +
                  Data->SEC_PID.kd * ( (float)acummulated_error );


  #ifdef ENABLE_SEC_PID_LOG
  ESP_LOGI(SEC_TAG, "Left Rpm : %f", result );
  #endif //ENABLE_SEC_PID_LOG


  Data->rpm_error.left = result;
}

/*- - SEC_PIDLeftRPMCalculateError() - - - - - - - - - - - - - - *//*!

 \brief Calculate Rpm Error by PID

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SEC_PIDRightRPMCalculateError( GlobalData_t *Data, float Curr_RPM ){
  float target_rpm = Data->rpm_setup - Data->position_error;

  static float acummulated_error = 0;
  static float last_error = 0;
  float current_error = Curr_RPM - target_rpm;
  float error_delta =  current_error - last_error;
  acummulated_error += current_error;

  if(acummulated_error > (target_rpm / 10) )
    acummulated_error = (target_rpm / 10);
  else 
    if(acummulated_error < -(target_rpm / 10) )
      acummulated_error = -(target_rpm / 10);

  last_error = current_error;

  float result =  Data->SEC_PID.kp * ( (float)current_error ) + 
                  Data->SEC_PID.ki * ( (float)error_delta )   +
                  Data->SEC_PID.kd * ( (float)acummulated_error );


  #ifdef ENABLE_SEC_PID_LOG
  ESP_LOGI(SEC_TAG, "Right Rpm : %f", result );
  #endif //ENABLE_SEC_PID_LOG

  Data->rpm_error.right = result;
}

/*- - SEC_EncoderInit() - - - - - - - - - - - - - - - - - - - - - *//*!
 \brief Initialization of Quadrature Encoder

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SEC_EncoderInit( void ){
  memset(&SEC_ENCA_data, 0, sizeof(SECEncData_t));
  memset(&SEC_ENCB_data, 0, sizeof(SECEncData_t));

  SEC_ENCA_data.a_pin = QENC_A_A_PIN;
  SEC_ENCA_data.b_pin = QENC_A_B_PIN;
  SEC_ENCB_data.a_pin = QENC_B_A_PIN;
  SEC_ENCB_data.b_pin = QENC_B_B_PIN;
  
  //zero-initialize the config structure.
  gpio_config_t io_conf = {};
  //interrupt of rising edge
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  //set as input mode
  io_conf.mode = GPIO_MODE_INPUT;
  //bit mask of the pins, use GPIO of ENC here
  io_conf.pin_bit_mask = QENC_MASK_PIN;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //enable pull-up mode
  io_conf.pull_up_en = 1;
  //configure GPIO with the given settings
  gpio_config(&io_conf);

  //install gpio isr service
  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL2);
  
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(QENC_A_A_PIN, SEC_ISR_ENCA_A, NULL);
  gpio_isr_handler_add(QENC_A_B_PIN, SEC_ISR_ENCA_B, NULL);
  gpio_isr_handler_add(QENC_B_A_PIN, SEC_ISR_ENCB_A, NULL);
  gpio_isr_handler_add(QENC_B_B_PIN, SEC_ISR_ENCB_B, NULL);
}

/*- - SEC_MotorInit() - - - - - - - - - - - - - - - - - - - - - - *//*!
 \brief Initialization of motor driver pins

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SEC_MotorInit( void ){
  // Configure the input driver pins   
  gpio_config_t io_conf = {}; //zero-initialize the config structure.
  
  io_conf.intr_type = GPIO_INTR_DISABLE; //disable interrupt
  io_conf.mode = GPIO_MODE_OUTPUT; //set as output mode
  io_conf.pin_bit_mask = DRV_EN_MASK_PIN; //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pull_down_en = 0; //disable pull-down mode
  io_conf.pull_up_en = 0; //disable pull-up mode
  gpio_config(&io_conf); //configure GPIO with the given settings

  // set to defaul values
  gpio_set_level(DRV_ENABLE_A_PIN, false);
  gpio_set_level(DRV_ENABLE_B_PIN, false);
  

  // Prepare and then apply the LEDC PWM timer configuration
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = MOTOR_TIMER_MODE,
    .timer_num        = MOTOR_TIMER,
    .duty_resolution  = MOTOR_TIMER_RES,
    .freq_hz          = MOTOR_TIMER_FREQ,  // Set output frequency at 5 kHz
    .clk_cfg          = MOTOR_TIMER_CLK
  };
  ledc_timer_config(&ledc_timer);

  // Prepare and then apply the ENA channel configuration
  ledc_channel_config_t ledc_channel = {
    .speed_mode     = MOTOR_TIMER_MODE,
    .channel        = MOTOR_TIMER_INA_1_CH,
    .timer_sel      = MOTOR_TIMER,
    .intr_type      = LEDC_INTR_DISABLE,
    .gpio_num       = DRV_IN_A_1_PIN,
    .duty           = 0, // Set duty to 0%
    .hpoint         = 0
  };
  ledc_channel_config(&ledc_channel);

  ledc_channel.channel = MOTOR_TIMER_INA_2_CH;
  ledc_channel.gpio_num = DRV_IN_A_2_PIN;
  ledc_channel_config(&ledc_channel);

  ledc_channel.channel = MOTOR_TIMER_INB_1_CH;
  ledc_channel.gpio_num = DRV_IN_B_1_PIN;
  ledc_channel_config(&ledc_channel);

  ledc_channel.channel = MOTOR_TIMER_INB_2_CH;
  ledc_channel.gpio_num = DRV_IN_B_2_PIN;
  ledc_channel_config(&ledc_channel);
}

/*- - SEC_ISR_ENCA_A() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief ISR of Quadrature Encoder pin A

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SEC_ISR_ENCA_A( void *pvParameter ){
  portENTER_CRITICAL_ISR(&ENCADataMutex);
  if (SEC_ENCA_data.enable){
    if ( gpio_get_level(SEC_ENCA_data.b_pin) )
      SEC_ENCA_data.count++;
    else
      SEC_ENCA_data.count--;

    SEC_ENCA_data.enable = false;
  }
  portEXIT_CRITICAL_ISR(&ENCADataMutex);
}

/*- - SEC_ISR_ENCB_A() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief ISR of Quadrature Encoder pin A

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SEC_ISR_ENCB_A( void *pvParameter ){
  portENTER_CRITICAL(&ENCBDataMutex);
  if (SEC_ENCB_data.enable){
    if ( gpio_get_level(SEC_ENCB_data.b_pin) )
      SEC_ENCB_data.count++;
    else
      SEC_ENCB_data.count--;
        
    SEC_ENCB_data.enable = false;
  }
  portEXIT_CRITICAL(&ENCBDataMutex);
}

/*- - SEC_ISR_ENCA_B() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief ISR of Quadrature Encoder pin B

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SEC_ISR_ENCA_B( void *pvParameter ){
  portENTER_CRITICAL(&ENCADataMutex);
  SEC_ENCA_data.enable = true;
  portEXIT_CRITICAL(&ENCADataMutex);
}

/*- - SEC_ISR_ENCB_B() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief ISR of Quadrature Encoder pin B

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void SEC_ISR_ENCB_B( void *pvParameter ){
  portENTER_CRITICAL(&ENCBDataMutex);
  SEC_ENCB_data.enable = true;
  portEXIT_CRITICAL(&ENCBDataMutex);
}


