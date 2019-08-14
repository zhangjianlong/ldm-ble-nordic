/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

//custom IO
#define SI4432_NIRQ     16  /*input*/
#define HPP_BAT_DETECT  29  /*input*/
#define VBAT_ADC        5   /*input*/
#define WLS_PWR_EN      31  /*output*/
#define BLE_APP_DETECT  23  /*output*/
#define HPP_PLOCK       22  /*output or input*/
#define P_CTRL          24  /*output*/

//LEDs
#define LED_BT   28  /*output*/
#define LED_WLS  30  /*output*/

#define LEDS_NUMBER    2
#define LED_START      28
#define LED_1          28
#define LED_2          30//29
//#define LED_3          30
//#define LED_4          31
#define LED_STOP       30
   
#define LEDS_ACTIVE_STATE  1

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST  { LED_1, LED_2 }//, LED_3, LED_4 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
//#define BSP_LED_2      LED_3
//#define BSP_LED_3      LED_4

#define BUTTONS_NUMBER  1//0//4

//pin4 -- NC
#define BUTTON_START   4//12
#define BUTTON_1       4//12  /*NC Pin*/
//#define BUTTON_2       13
//#define BUTTON_3       14
//#define BUTTON_4       15
#define BUTTON_STOP    4//15
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

//#define BUTTONS_LIST { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4 }
#define BUTTONS_LIST { BUTTON_1 /*, BUTTON_2, BUTTON_3, BUTTON_4*/ }

#define BSP_BUTTON_0   BUTTON_1
//#define BSP_BUTTON_1   BUTTON_2
//#define BSP_BUTTON_2   BUTTON_3
//#define BSP_BUTTON_3   BUTTON_4

#define RX_PIN_NUMBER   27
#define TX_PIN_NUMBER   26
#define CTS_PIN_NUMBER  15  /*NC Pin*/
#define RTS_PIN_NUMBER  25  /*NC Pin*/
#define HWFC            false//true

#define SPIM0_SCK_PIN   18  // SPI clock GPIO pin number.
#define SPIM0_MOSI_PIN  19  // SPI Master Out Slave In GPIO pin number.
#define SPIM0_MISO_PIN  20  // SPI Master In Slave Out GPIO pin number.
#define SPIM0_SS_PIN    17  // SPI Slave Select GPIO pin number.


// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H
