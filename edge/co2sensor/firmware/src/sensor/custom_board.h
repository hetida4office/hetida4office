#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#include "nrf_gpio.h"
#include "pins.h"

#define LEDS_NUMBER    2

#define LED_1          PIN_LED1
#define LED_2          PIN_LED2
#define LED_START      LED_1
#define LED_STOP       LED_2

#define LEDS_ACTIVE_STATE 0

#define LEDS_LIST { LED_1, LED_2 }

#define LEDS_INV_MASK  LEDS_MASK

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2

#define BUTTONS_NUMBER 1

#ifdef PIN_SWITCH1
#define BUTTON_1       PIN_SWITCH1
#else
#define BUTTON_1       PIN_SWITCH
#endif
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

#define RX_PIN_NUMBER  PIN_RX_D0
#define TX_PIN_NUMBER  PIN_TX_D1
#define HWFC           false

#endif // CUSTOM_BOARD_H
