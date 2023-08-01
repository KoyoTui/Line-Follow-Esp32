#ifndef LOG_IN_FLASH_H
#define LOG_IN_FLASH_H


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Global Prototypes
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
void LIF_Init(void);
void LIF_Open(void);
void LIF_Read(void);
void LIF_Write(void);
void LIF_Close(void);


#endif //LOG_IN_FLASH_H