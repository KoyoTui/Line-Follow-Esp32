#include "common.h"
#include "configuration.h"
#include "Interface_GPIO.h"
#include <driver/gpio.h>


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Defines
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Variables
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
bool LED_OperState = false;
bool LED_SectorState = false;
bool LED_CommState = false;


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Locals Prototypes
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Global Functions
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*- - GPIO_Init() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Initialization of Leds and Button GPIO

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void GPIO_Init(void){  
  // Configure the front sensors mux control pins   
  gpio_config_t io_conf = {}; //zero-initialize the config structure.
  
  io_conf.intr_type = GPIO_INTR_DISABLE; //disable interrupt
  io_conf.mode = GPIO_MODE_OUTPUT; //set as output mode
  io_conf.pin_bit_mask = LED_MASK_PIN; //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pull_down_en = 0; //disable pull-down mode
  io_conf.pull_up_en = 0; //disable pull-up mode
  gpio_config(&io_conf); //configure GPIO with the given settings

  // set to default
  LED_OperSet();
  LED_SectorSet();
  LED_CommSet();
  
  // Configure the BUTTON pins   
  io_conf.intr_type = GPIO_INTR_DISABLE; //disable interrupt
  io_conf.mode = GPIO_MODE_INPUT; //set as output mode
  io_conf.pin_bit_mask = BUTTON_MASK_PIN; //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pull_down_en = 0; //disable pull-down mode
  io_conf.pull_up_en = 0; //disable pull-up mode
  gpio_config(&io_conf); //configure GPIO with the given settings  
}

/*- - LED_OperSet() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Set the state of Oper LED

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void LED_OperSet(void){  
  LED_OperState = true;  

  gpio_set_level(LED_1_PIN, LED_OperState);
}

/*- - LED_OperClear() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Clear the state of Oper LED

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void LED_OperClear(void){  
  LED_OperState = false;  

  gpio_set_level(LED_1_PIN, LED_OperState);
}

/*- - LED_OperToggle() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Toggle the state of Oper LED

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void LED_OperToggle(void){
  if (LED_OperState)
    LED_OperState = false;
  else
    LED_OperState = true;

  gpio_set_level(LED_1_PIN, LED_OperState);
}

/*- - LED_SectorSet() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Set the state of Sector LED

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void LED_SectorSet(void){  
  LED_SectorState = true;  

  gpio_set_level(LED_2_PIN, LED_SectorState);
}

/*- - LED_SectorClear() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Clear the state of Sector LED

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void LED_SectorClear(void){  
  LED_SectorState = false;  

  gpio_set_level(LED_2_PIN, LED_SectorState);
}

/*- - LED_SectorToggle() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Toggle the state of Sector LED

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void LED_SectorToggle(void){
  if (LED_SectorState)
    LED_SectorState = false;
  else
    LED_SectorState = true;

  gpio_set_level(LED_2_PIN, LED_SectorState);
}

/*- - LED_CommSet() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Set the state of Comm LED

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void LED_CommSet(void){  
  LED_CommState = true;  

  gpio_set_level(LED_3_PIN, LED_CommState);
}

/*- - LED_CommClear() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Clear the state of Comm LED

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void LED_CommClear(void){  
  LED_CommState = false;  

  gpio_set_level(LED_3_PIN, LED_CommState);
}

/*- - LED_CommToggle() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Toggle the state of Comm LED

 \return nothing
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void LED_CommToggle(void){
  if (LED_CommState)
    LED_CommState = false;
  else
    LED_CommState = true;

  gpio_set_level(LED_3_PIN, LED_CommState);
}

/*- - BUTTON_GetB1() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Get state of button 1

 \return state of button 1
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
bool BUTTON_GetB1(void){
  return gpio_get_level(BUTTON_1_PIN);
}

/*- - BUTTON_GetB2() - - - - - - - - - - - - - - - - - - - - - -*//*!
 \brief Get state of button 2

 \return state of button 2
*///- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
bool BUTTON_GetB2(void){
  return gpio_get_level(BUTTON_2_PIN);
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Local Functions
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
