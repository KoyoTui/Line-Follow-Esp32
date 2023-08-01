#ifdef ENABLE_BLE

#ifndef BLUETOOTH_LOW_ENERGY_H
#define BLUETOOTH_LOW_ENERGY_H


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Global Prototypes
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*- - BLE_Task() - - - - - - - - - - - - - - - - - - - - - - - - -*//*!

 \brief Task of Control Loop Sector

 \note Attention, it is necessary to release the semGlobalSector
        resource to release the task

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void BLE_Task(void *pvParameter );

/*- - BLE_Send() - - - - - - - - - - - - - - - - - - - - - - - - -*//*!

 \brief Send a msg to BLE

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void BLE_Send(char* Msg);


#endif //BLUETOOTH_LOW_ENERGY_H

#endif //ENABLE_BLE