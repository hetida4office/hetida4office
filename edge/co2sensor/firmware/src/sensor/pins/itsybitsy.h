#ifndef PINS_H
#define PINS_H

#include "hal/nrf_gpio.h"
#include <nrfx_lpcomp.h>

#define PIN_RX_D0 NRF_GPIO_PIN_MAP(0, 25)
#define PIN_TX_D1 NRF_GPIO_PIN_MAP(0, 24)

#define PIN_D2 NRF_GPIO_PIN_MAP(1, 2)
#define PIN_D5_5V NRF_GPIO_PIN_MAP(0, 27)
#define PIN_D7 NRF_GPIO_PIN_MAP(1, 8)
#define PIN_D9 NRF_GPIO_PIN_MAP(0, 7)
#define PIN_D10 NRF_GPIO_PIN_MAP(0, 5) // AIN3
#define PIN_D11 NRF_GPIO_PIN_MAP(0, 26)
#define PIN_D12 NRF_GPIO_PIN_MAP(0, 11)
#define PIN_D13 NRF_GPIO_PIN_MAP(0, 12)
#define PIN_A0 NRF_GPIO_PIN_MAP(0, 4) // AIN2
#define PIN_A1 NRF_GPIO_PIN_MAP(0, 30) // AIN6
#define PIN_A2 NRF_GPIO_PIN_MAP(0, 28) // AIN4
#define PIN_A3 NRF_GPIO_PIN_MAP(0, 31) // AIN7
#define PIN_A4 NRF_GPIO_PIN_MAP(0, 2) // AIN0
#define PIN_A5 NRF_GPIO_PIN_MAP(0, 3) // AIN1
#define PIN_A5_LPCOMP NRF_LPCOMP_INPUT_1 // AIN1 -- reserved for LPCOMP peripheral

#define PIN_LED1 NRF_GPIO_PIN_MAP(0, 6)
#define PIN_LED2 NRF_GPIO_PIN_MAP(0, 4)
#define PIN_SWITCH NRF_GPIO_PIN_MAP(0, 29) // AIN5

#define PIN_DOT_DATA NRF_GPIO_PIN_MAP(0, 8)
#define PIN_DOT_CLK  NRF_GPIO_PIN_MAP(1, 9)

#define PIN_QSPI_CS NRF_GPIO_PIN_MAP(0, 23)
#define PIN_QSPI_DATA0 NRF_GPIO_PIN_MAP(0, 21)
#define PIN_QSPI_DATA1 NRF_GPIO_PIN_MAP(0, 22)
#define PIN_QSPI_DATA2 NRF_GPIO_PIN_MAP(1, 0)
#define PIN_QSPI_DATA3 NRF_GPIO_PIN_MAP(0, 17)
#define PIN_QSPI_SCK NRF_GPIO_PIN_MAP(0, 19)

#define PIN_SDA NRF_GPIO_PIN_MAP(0, 16)
#define PIN_SCL NRF_GPIO_PIN_MAP(0, 14)

#define PIN_MISO NRF_GPIO_PIN_MAP(0, 20)
#define PIN_MOSI NRF_GPIO_PIN_MAP(0, 15)
#define PIN_SCK NRF_GPIO_PIN_MAP(0, 13)

// UART
#define PIN_UART_RX PIN_RX_D0
#define PIN_UART_TX PIN_TX_D1

// LP8
#define PIN_LP8_EN_CHARGE PIN_D7 // digital out, D0S1, no pull
#define PIN_LP8_EN_REV_BLOCK PIN_MOSI // digital out, S0D1, no pull
#define PIN_LP8_EN_PWR PIN_SCK // digital out, D0S1, no pull
#define PIN_LP8_EN_MEAS PIN_D2 // digital out, D0S1, no pull
#define PIN_LP8_VCAP_LPCOMP PIN_A5_LPCOMP // analog in
#define PIN_LP8_MEAS_RDY PIN_MISO // digital in
#define PIN_LP8_RXD PIN_UART_TX // UART
#define PIN_LP8_TXD PIN_UART_RX // UART

// I2C
#define PIN_I2C_SDA PIN_SDA
#define PIN_I2C_SCL PIN_SCL

#define NUM_DIGITAL_IN 1 // required to configure resources for GPIOTE event listeners

#endif // PINS_H