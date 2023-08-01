#ifndef POSITION_SENSOR_CONTROL_H
#define POSITION_SENSOR_CONTROL_H


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Global Prototypes
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void PSC_Task(void *arg);
void PSC_Calibration( GlobalData_t *Data );


#endif //POSITION_SENSOR_CONTROL_H