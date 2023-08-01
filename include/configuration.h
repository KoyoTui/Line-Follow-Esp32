#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Defines
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

// LOGs 
//#define ENABLE_PSC_ADC_LOG        // log adc reads
//#define ENABLE_PSC_POS_LOG        // log position
//#define ENABLE_PSC_PID_LOG        // log PSC pid result
//#define ENABLE_PSC_CAL_ADC_LOG    // log calibration adc sensors

//#define ENABLE_SEC_DTC_LOG        // log distance
//#define ENABLE_SEC_RPM_LOG        // log SEC rpm calculation
#define ENABLE_SEC_PWM_LOG        // log SEC rpm calculation
//#define ENABLE_SEC_PID_LOG        // log SEC pid result

//#define ENABLE_SCT_MKR_LOG        // log marker reads
//#define ENABLE_SCT_ADC_LOG        // log adc reads

//#define ENABLE_BLE

#define ENABLE_MOTOR
//#define ENABLE_BRUSHLESS
//#define ENABLE_SEC

// Times 
#define SEC_TIME_TO_ENTER_MS      2
#define SCT_TIME_TO_ENTER_MS      1
#define PSC_TIME_TO_ENTER_MS      2

// Sensor Line 
#define ADC_SAMPLES               1
#define TRACK_TYPE                      1 //0 BLACK_TRACK 1 WHITE_TRACK
#define TARGET_SENSOR_READING           (int32_t) 8500 
#define FRONTAL_THRESHOLD         700
#define LATERAL_THRESHOLD         700
#define LATERAL_CIRC_BUFF_LEN     1
#define LATERAL_RIGHT_SERIAL_READ
//#define LATERAL_LEFT_SERIAL_READ
//#define USE_RAILUX_INTERFACE

// Const
#define RPM_TO_DUTY                (float)2.1

// Position PID 
#define PSC_INIT_KP               (float)3.000
#define PSC_INIT_KI               (float)0.000
#define PSC_INIT_KD               (float)0.000

// Speed PID
#define SEC_INIT_KP               (float)0.000
#define SEC_INIT_KI               (float)0.000
#define SEC_INIT_KD               (float)0.000


#if TRACK_TYPE == 0
  #define BLACK_LINE
#else
  #define WHITE_LINE
#endif

#endif //CONFIGURATION_H