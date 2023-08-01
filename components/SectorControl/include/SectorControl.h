#ifndef SECTOR_CONTROL_H
#define SECTOR_CONTROL_H


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Global Prototypes
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void SCT_Task(void *pvParameter );
void SCT_Calibration( GlobalData_t *Data );


#endif //SECTOR_CONTROL_H