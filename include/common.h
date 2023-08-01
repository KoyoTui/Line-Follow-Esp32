#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>

#include "esp_event.h"
#include "esp_log.h"

#include <esp_adc/adc_oneshot.h>
#include <driver/gptimer.h>
#include <driver/ledc.h>

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Types
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
typedef struct {
  float kp;   
  float ki;   
  float kd;
} GlobalPID_t;

typedef struct {
  float curr;
  float last;
} GlobalCurLastF_t;

typedef struct {
  uint16_t curr;
  uint16_t last;
} GlobalCurLastU16_t;

typedef struct {
  float right;
  float  left;
} GlobalRigLefF_t;

typedef struct {
  GlobalCurLastF_t right;
  GlobalCurLastF_t  left;
} GlobalRigLefCurLast_t;

typedef struct {
  uint64_t    time; // timer counter         
  uint16_t  sector; //  sector marker counter
  uint16_t starter; // starter marker counter
} GlobalCounterU64_t;

typedef struct {
  int16_t right; // right pwm setups
  int16_t  left; //  left pwm setups
  int16_t setup;
} GlobalPWM_t;

typedef struct {
  float right;
  float  left;
  float   avg;
} GlobalRigLefAvgF_t;

typedef struct {
  int min;
  int max;
} GlobalADCCalibrate_t;

typedef struct {
  GlobalRigLefCurLast_t rpm; // rpm params

  GlobalRigLefF_t rpm_error; // rpm params

  GlobalRigLefAvgF_t distance; // distance params

  GlobalPWM_t pwm; // pwm params

  float position_error; // position params

  float rpm_setup; // setup params

  GlobalCounterU64_t counter; // counters

  GlobalPID_t PSC_PID; // PID params

  GlobalPID_t SEC_PID; // PID params

  // Global State 
  uint16_t global_state;

  GlobalADCCalibrate_t PSC_Cal;

  GlobalADCCalibrate_t SCT_Cal;

} GlobalData_t;



/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Variables
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

// Global Sempahores 
extern SemaphoreHandle_t GlobalDataMutex;
extern SemaphoreHandle_t ADC1Mutex;
extern SemaphoreHandle_t SEC_semaphore;
extern SemaphoreHandle_t PSC_semaphore;
extern SemaphoreHandle_t SCT_semaphore;

// Global Handle 
extern adc_oneshot_unit_handle_t ADCHandle;
extern gptimer_handle_t GPTIMERHandle;

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Defines
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

// locomotion 
#define PPR_MOTOR                 15.0
#define REDUCTION_MOTOR           1.0
#define REDUCTION_WHEEL           3.0
#define PPR_WHEEL                 PPR_MOTOR * REDUCTION_MOTOR * REDUCTION_WHEEL
#define WHEEL_RADIUS              12.0
#define PI                        3.1415
#define MOTOR_TIMER               LEDC_TIMER_0
#define MOTOR_TIMER_RES           LEDC_TIMER_13_BIT
#define MOTOR_TIMER_MODE          LEDC_LOW_SPEED_MODE
#define MOTOR_TIMER_FREQ          (5000)
#define MOTOR_TIMER_CLK           LEDC_AUTO_CLK
#define MOTOR_TIMER_INA_1_CH      LEDC_CHANNEL_0
#define MOTOR_TIMER_INA_2_CH      LEDC_CHANNEL_1
#define MOTOR_TIMER_INB_1_CH      LEDC_CHANNEL_2
#define MOTOR_TIMER_INB_2_CH      LEDC_CHANNEL_3
#define MOTOR_TIMER_MAX_DUTY      (8191) 
#define MOTOR_TIMER_MIN_DUTY      -MOTOR_TIMER_MAX_DUTY
#define MOTOR_PWM_SETUP           5000


// Frontal Sensor Pins 
#define FRONTAL_SENSOR_PIN        1
#define FRONTAL_SENSOR_CH         ADC_CHANNEL_0
#define FRONTAL_SENSOR_NUM_MAX    16
#define FRONTAL_CTRL_0_PIN        2
#define FRONTAL_CTRL_1_PIN        4
#define FRONTAL_CTRL_2_PIN        5
#define FRONTAL_CTRL_3_PIN        6
//#define FRONTAL_CTRL_VCC_PIN      13
#define FRONTAL_CTRL_MASK         ( (1ULL << FRONTAL_CTRL_0_PIN) | \
                                          (1ULL << FRONTAL_CTRL_1_PIN) | \
                                          (1ULL << FRONTAL_CTRL_2_PIN) | \
                                          (1ULL << FRONTAL_CTRL_3_PIN) )

// Lateral Sensor Pins 
#define LATERAL_SENSOR_4_PIN      7
#define LATERAL_SENSOR_4_CH       ADC_CHANNEL_6
#define LATERAL_SENSOR_3_PIN      8
#define LATERAL_SENSOR_3_CH       ADC_CHANNEL_7
#define LATERAL_SENSOR_2_PIN      9
#define LATERAL_SENSOR_2_CH       ADC_CHANNEL_8
#define LATERAL_SENSOR_1_PIN      10
#define LATERAL_SENSOR_1_CH       ADC_CHANNEL_9

// Buzz Pins 
#define BUZZER_PIN                11
#define BUZZER_TIMER              LEDC_TIMER_1
#define BUZZER_TIMER_RES          LEDC_TIMER_10_BIT
#define BUZZER_TIMER_MODE         LEDC_LOW_SPEED_MODE
#define BUZZER_TIMER_FREQ         (4000)
#define BUZZER_TIMER_CLK          LEDC_AUTO_CLK
#define BUZZER_TIMER_CH           LEDC_CHANNEL_0

// IR Pins 
#define IR_PIN                    12

// Drivers Pins  
#define DRV_ENABLE_A_PIN          14
#define DRV_IN_A_1_PIN            15
#define DRV_IN_A_2_PIN            16
#define DRV_ENABLE_B_PIN          21
#define DRV_IN_B_1_PIN            18
#define DRV_IN_B_2_PIN            17
#define DRV_EN_MASK_PIN           ( (1ULL << DRV_ENABLE_A_PIN) | \
                                          (1ULL << DRV_ENABLE_B_PIN) )
#define DRV_IN_MASK_PIN           ( (1ULL << DRV_IN_A_1_PIN) | \
                                          (1ULL << DRV_IN_A_2_PIN) | \
                                          (1ULL << DRV_IN_B_1_PIN) | \
                                          (1ULL << DRV_IN_B_2_PIN) )
#define DRV_INA_MASK_PIN          ( (1ULL << DRV_IN_A_1_PIN) | \
                                          (1ULL << DRV_IN_A_2_PIN) )
#define DRV_INB_MASK_PIN          ( (1ULL << DRV_IN_B_1_PIN) | \
                                          (1ULL << DRV_IN_B_2_PIN) )

// USB Pins 
#define USB_DP_PIN                19
#define USB_DN_PIN                20

// Quadrature Encoder Pins 
#define QENC_A_A_PIN              47
#define QENC_A_B_PIN              26
#define QENC_B_A_PIN              36
#define QENC_B_B_PIN              37
#define QENC_MASK_PIN             ( (1ULL << QENC_A_A_PIN) | \
                                          (1ULL << QENC_A_B_PIN) | \
                                          (1ULL << QENC_B_A_PIN) | \
                                          (1ULL << QENC_B_B_PIN) )

// I/O Pins 
#define BUTTON_1_PIN              39
#define BUTTON_2_PIN              38
#define BUTTON_MASK_PIN           ( (1ULL << BUTTON_1_PIN) | \
                                          (1ULL << BUTTON_2_PIN) )
#define LED_1_PIN                 40
#define LED_2_PIN                 41
#define LED_3_PIN                 42
#define LED_MASK_PIN              ( (1ULL << LED_1_PIN) | \
                                          (1ULL << LED_2_PIN) | \
                                          (1ULL << LED_3_PIN) )

// Brushless Pins
#define ESC_M1_PIN                35
#define ESC_M2_PIN                48
#define ESC_M3_PIN                34
#define ESC_M4_PIN                33
#define ESC_MASK_PIN              ( (1ULL << ESC_M1_PIN) | \
                                          (1ULL << ESC_M2_PIN) | \
                                          (1ULL << ESC_M3_PIN) | \
                                          (1ULL << ESC_M4_PIN) )

// Global State defines 
#define STATE_ENABLE_MOTORS_POS   0
#define STATE_ENABLE_MOTORS_MSK   (0x1u << STATE_ENABLE_MOTORS_POS)
#define STATE_ENABLE_MOTORS       STATE_ENABLE_MOTORS_MSK

// Buzzer Notes
#define Note_C 261
#define Note_D 294
#define Note_E 329
#define Note_F 349
#define Note_G 391
#define Note_Gs 415
#define Note_A 440
#define Note_As 455
#define Note_B 466
#define Note_Ch 523
#define Note_Csh 554
#define Note_Dh 587
#define Note_Dsh 622
#define Note_Eh 659
#define Note_Fh 698
#define Note_Fsh 740
#define Note_Gh 784
#define Note_Gsh 830
#define Note_Ah 880

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Global Function
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

int remap_int(int uc_val, int uc_min, int uc_max, int ue_min, int ue_max);

#endif //COMMON_H