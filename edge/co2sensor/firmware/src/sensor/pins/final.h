#ifndef PINS_H
#define PINS_H

#include "hal/nrf_gpio.h"
#include <nrfx_lpcomp.h>

// Custom Board
#define PIN_LED1 NRF_GPIO_PIN_MAP(0, 30)
#define PIN_LED2 NRF_GPIO_PIN_MAP(0, 31)
#define PIN_SWITCH NRF_GPIO_PIN_MAP(0, 10)

// UART
#define PIN_UART_RX NRF_GPIO_PIN_MAP(0, 5)
#define PIN_UART_TX NRF_GPIO_PIN_MAP(1, 9)

// LP8
#define PIN_LP8_EN_CHARGE NRF_GPIO_PIN_MAP(0, 9) // digital out, D0S1, no pull
#define PIN_LP8_EN_REV_BLOCK NRF_GPIO_PIN_MAP(0, 3) // digital out, S0D1, no pull
#define PIN_LP8_EN_PWR NRF_GPIO_PIN_MAP(1, 13) // digital out, D0S1, no pull
#define PIN_LP8_EN_MEAS NRF_GPIO_PIN_MAP(1, 11) // digital out, D0S1, no pull
#define PIN_LP8_VCAP_LPCOMP NRF_LPCOMP_INPUT_0 // analog in
#define PIN_LP8_MEAS_RDY NRF_GPIO_PIN_MAP(1, 10) // digital in
#define PIN_LP8_RXD PIN_UART_TX // UART
#define PIN_LP8_TXD PIN_UART_RX // UART

// I2C
#define PIN_I2C_SDA NRF_GPIO_PIN_MAP(0, 13)
#define PIN_I2C_SCL NRF_GPIO_PIN_MAP(0, 24)

// BAT
#define PIN_BAT_nEN_MEAS NRF_GPIO_PIN_MAP(0, 29) // digital out, S0D1, no pull
#define PIN_BAT_MEAS_SAADC NRF_SAADC_INPUT_AIN4 // analog in

#define NUM_DIGITAL_IN 1 // required to configure resources for GPIOTE event listeners

#endif // PINS_H