#ifndef INTERFACE_CONTROL_H
#define INTERFACE_CONTROL_H


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Global Prototypes
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void GPIO_Init(void);
void LED_OperSet(void);
void LED_OperClear(void);
void LED_OperToggle(void);
void LED_SectorSet(void);
void LED_SectorClear(void);
void LED_SectorToggle(void);
void LED_CommSet(void);
void LED_CommClear(void);
void LED_CommToggle(void);
bool BUTTON_GetB1(void);
bool BUTTON_GetB2(void);


#endif //INTERFACE_CONTROL_H