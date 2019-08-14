/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

/** @file
 *
 * @defgroup bootloader_secure main.c
 * @{
 * @ingroup dfu_bootloader_api
 * @brief Bootloader project main file for secure DFU.
 *
 */

#include <stdint.h>
#include "boards.h"
#include "nrf_mbr.h"
#include "nrf_bootloader.h"
#include "nrf_bootloader_app_start.h"
#include "nrf_dfu.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"
#include "app_error_weak.h"
#include "nrf_bootloader_info.h"
#include "nrf_drv_timer.h"
#include "nrf_wdt.h"
#include "app_uart.h"
#include "app_fifo.h"

const uint32_t UICR_ADDR_0x18 __attribute__((at(0x10001018))) __attribute__((used)) = 0x0007E000;

#define UART_TX_BUF_SIZE                  256                                    	/**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                  256                                        /**< UART RX buffer size. */
#define MAX_UART_DATA_LEN                 256                                        //100

typedef struct{
	uint8_t idx;
	uint8_t len;
	uint8_t data[MAX_UART_DATA_LEN];
	uint8_t rx_idle_count;
	uint8_t rx_idle_count_en;
}stru_usart_data_t;

/*****************************************************************************/							
void uart_send_string(uint8_t *pstr, uint32_t len)
{
	uint32_t i;
	for(i = 0; i < len; i++)
	{
		app_uart_put(pstr[i]);
	}
}

stru_usart_data_t g_uart_data;

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    NRF_LOG_ERROR("received a fault! id: 0x%08x, pc: 0x&08x\r\n", id, pc);
    NVIC_SystemReset();
}

void app_error_handler_bare(uint32_t error_code)
{
    (void)error_code;
    NRF_LOG_ERROR("received an error: 0x%08x!\r\n", error_code);
    NVIC_SystemReset();
}

void uart_event_handle(app_uart_evt_t * p_event)
{
//	static uint8_t data_array[256];
//	static uint8_t index = 0;
	//uint32_t err_code;

	switch(p_event->evt_type)
	{
		case APP_UART_DATA_READY:
		{
			app_uart_get(&g_uart_data.data[g_uart_data.idx]);
			g_uart_data.idx++;

			if((g_uart_data.data[g_uart_data.idx-1] == '\n') || (g_uart_data.idx >= (MAX_UART_DATA_LEN)) \
				|| (g_uart_data.data[g_uart_data.idx-1] == '}') || (g_uart_data.data[g_uart_data.idx-1] == ']'))
			{
				g_uart_data.len = g_uart_data.idx;
				uart_send_string("ok", 2);
				g_uart_data.idx = 0;
			}
			g_uart_data.rx_idle_count_en = 1;
			g_uart_data.rx_idle_count = 0;
		}
		break;

		case APP_UART_COMMUNICATION_ERROR:
			APP_ERROR_HANDLER(p_event->data.error_communication);
		break;

		case APP_UART_FIFO_ERROR:
			APP_ERROR_HANDLER(p_event->data.error_code);
		break;

		default:
		break;
	}
}


/**@brief Function for initialization of LEDs.
 */
static void leds_init(void)
{
    bsp_board_leds_init();
    bsp_board_led_on(BSP_BOARD_LED_2);
}


/**@brief Function for initializing the button module.
 */
static void buttons_init(void)
{
    nrf_gpio_cfg_sense_input(BOOTLOADER_BUTTON,
                             BUTTON_PULL,
                             NRF_GPIO_PIN_SENSE_LOW);
}

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
	uint32_t err_code;
	const app_uart_comm_params_t comm_params = {
		RX_PIN_NUMBER,
		TX_PIN_NUMBER,
		RTS_PIN_NUMBER,
		CTS_PIN_NUMBER,
		APP_UART_FLOW_CONTROL_DISABLED,
		false,
		UART_BAUDRATE_BAUDRATE_Baud115200
	};

	APP_UART_FIFO_INIT(&comm_params,
						UART_RX_BUF_SIZE,
						UART_TX_BUF_SIZE,
						uart_event_handle,
						APP_IRQ_PRIORITY_LOW,
						err_code);
	APP_ERROR_CHECK(err_code);
}

uint32_t err_code1 = NRF_SUCCESS;


/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t ret_val;

    (void) NRF_LOG_INIT(NULL);

    NRF_LOG_INFO("Inside main\r\n");
    leds_init();
    buttons_init();
    nrf_gpio_cfg_output(BLE_APP_DETECT);
    nrf_gpio_pin_set(BLE_APP_DETECT);  /*unconnected, High*/
    uart_init();
    ret_val = nrf_bootloader_init();
    APP_ERROR_CHECK(ret_val);
    // Either there was no DFU functionality enabled in this project or the DFU module detected
    // no ongoing DFU operation and found a valid main application.
    // Boot the main application.
    nrf_bootloader_app_start(MAIN_APPLICATION_START_ADDR);

    // Should never be reached.
    NRF_LOG_INFO("After main\r\n");
}

/**
 * @}
 */
