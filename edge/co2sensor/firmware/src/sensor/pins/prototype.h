#ifndef PINS_H
#define PINS_H

#include "hal/nrf_gpio.h"
#include <nrfx_lpcomp.h>

#define PIN_RX_D0 NRF_GPIO_PIN_MAP(0, 30)
#define PIN_TX_D1 NRF_GPIO_PIN_MAP(1, 9)

#define PIN_D2 NRF_GPIO_PIN_MAP(1, 11)
#define PIN_D3 NRF_GPIO_PIN_MAP(1, 10)
#define PIN_D4 NRF_GPIO_PIN_MAP(0, 3)
#define PIN_D6 NRF_GPIO_PIN_MAP(1, 13)
#define PIN_D7 NRF_GPIO_PIN_MAP(0, 9)
#define PIN_A5_LPCOMP NRF_LPCOMP_INPUT_0 // AIN0 -- reserved for LPCOMP peripheral

#define PIN_LED1 NRF_GPIO_PIN_MAP(0, 5)
#define PIN_LED2 NRF_GPIO_PIN_MAP(0, 6)
#define PIN_SWITCH1 NRF_GPIO_PIN_MAP(0, 10)
#define PIN_SWITCH2 NRF_GPIO_PIN_MAP(0, 31)

#define PIN_SDA NRF_GPIO_PIN_MAP(0, 13)
#define PIN_SCL NRF_GPIO_PIN_MAP(0, 4)

#define PIN_MEAS_BAT_SAADC NRF_SAADC_INPUT_AIN4
#define PIN_nEN_MEAS_BAT NRF_GPIO_PIN_MAP(0, 29)


// UART
#define PIN_UART_RX PIN_RX_D0
#define PIN_UART_TX PIN_TX_D1

// LP8
#define PIN_LP8_EN_CHARGE PIN_D7 // digital out, D0S1, no pull
#define PIN_LP8_EN_REV_BLOCK PIN_D4 // digital out, S0D1, no pull
#define PIN_LP8_EN_PWR PIN_D6 // digital out, D0S1, no pull
#define PIN_LP8_EN_MEAS PIN_D2 // digital out, D0S1, no pull
#define PIN_LP8_VCAP_LPCOMP PIN_A5_LPCOMP // analog in
#define PIN_LP8_MEAS_RDY PIN_D3 // digital in
#define PIN_LP8_RXD PIN_UART_TX // UART
#define PIN_LP8_TXD PIN_UART_RX // UART

// I2C
#define PIN_I2C_SDA PIN_SDA
#define PIN_I2C_SCL PIN_SCL

// BAT
#define PIN_BAT_nEN_MEAS PIN_nEN_MEAS_BAT // digital out, S0D1, no pull
#define PIN_BAT_MEAS_SAADC PIN_MEAS_BAT_SAADC // analog in

#define NUM_DIGITAL_IN 1 // required to configure resources for GPIOTE event listeners

#endif // PINS_H