/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_app_proximity_main main.c
 * @{
 * @ingroup ble_sdk_app_proximity_eval
 * @brief Proximity Application main file.
 *
 * This file contains is the source code for a sample proximity application using the
 * Immediate Alert, Link Loss and Tx Power services.
 *
 * This application would accept pairing requests from any peer device.
 *
 * It demonstrates the use of fast and slow advertising intervals.
 */

#include <stdint.h>
#include <string.h>

#include "ble_advertising.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_tps.h"
#include "ble_ias.h"
#include "ble_lls.h"
#include "ble_bas.h"
#include "ble_dis.h"
//timer
#include "nrf_drv_timer.h"
//user add +
#include "ble_customer.h"
#include "app_uart.h"
#include "common.h"
#include "protocol.h"
//
//----------
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "ble_ias_c.h"
#include "app_util.h"

#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble_db_discovery.h"
#include "peer_manager.h"
#include "fds.h"
#include "fstorage.h"
#include "ble_conn_state.h"
#include "nrf_drv_rtc.h"
#include "ble_dfu.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "nrf_peripherals.h"
//#ifdef ADC_PRESENT
//#include "nrf_drv_adc.h"
//#else
#include "nrf_drv_saadc.h"
//#endif //ADC_PRESENT

//spi for si4432
#include "nrf_drv_spi.h"

#include "nrf_drv_ppi.h"
#include "si4432.h"
#include "nrf_drv_wdt.h"
#include "ble_gap.h"
#include "ble_adv_beacon.h"
#include "id_manager.h"
#include "ble_hts.h"
#include "nrf_dfu_settings.h"

//const uint32_t UICR_ADDR_0x18 __attribute__((at(0x10001018))) __attribute__((used)) = 0x0007E000;

#define HPP_NEW_FUNCTION_CONTROL TURE
#define SPI_INSTANCE  0 /**< SPI instance index. */
const nrf_drv_spi_t m_spi_master_0 = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(1);

//volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                             /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
//#define IS_SRVC_CHANGED_CHARACT_PRESENT  0                                             /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define COMPARE_COUNTERTIME  (3UL)                                        /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE              GATT_MTU_SIZE_DEFAULT                        /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED         BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2         /**< Reply when unsupported features are requested. */

#define PTI_BRAND_NWI		0X4E
#define PTI_BRAND_PLOTT		0X50
#define PTI_BRAND_STANLEY	0XD0
#define PTI_BRAND_DEWALT	     0XFE
#define PTI_NWITOOL           0X00

#define PTI_DEVICE_DW080LR    		0X0409
#define PTI_DEVICE_DW080LG    		0X0509
#define PTI_DEVICE_DW0743RS   		0X0414
#define PTI_DEVICE_DW0743GS   		0X0514
#define PTI_DEVICE_TLM165s   			0X0306
#define PTI_DEVICE_TLM165si   		0X0406
#define PTI_DEVICE_TLM330s			0X0506
#define PTI_DEVICE_DW0165S_DW03050   	0X0207
#define PTI_DEVICE_DW0330S_DW03101   	0X0307
#define PTI_DEVICE_NWITOOL   			0X0000

#define BRAND_NAME_NWI		"Northwest Instrument Inc."	
#define BRAND_NAME_PLOTT		"Plott Inc."	
#define BRAND_NAME_STANLEY	"Stanley Black and Decker"
#define BRAND_NAME_DEWALT	"Stanley Black and Decker"

#define CENTRAL_LINK_COUNT                0                                            /**<number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT             1                                            /**<number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

/*About advertising params*/
#define APP_ADV_INTERVAL_FAST_20_MS       MSEC_TO_UNITS(20, UNIT_0_625_MS)				/**< Fast advertising interval (in units of 0.625 ms.). */
#define APP_ADV_INTERVAL_FAST_152_MS      MSEC_TO_UNITS(152, UNIT_0_625_MS) 
#define APP_ADV_INTERVAL_FAST_200_MS      MSEC_TO_UNITS(200, UNIT_0_625_MS)				/**< Fast advertising interval (in units of 0.625 ms.). */
#define APP_ADV_INTERVAL_FAST_400_MS      MSEC_TO_UNITS(400, UNIT_0_625_MS)  
#define APP_ADV_INTERVAL_SLOW_2000_MS     MSEC_TO_UNITS(2000, UNIT_0_625_MS)           	/**< Slow advertising interval (in units of 0.625 ms.). */

#define APP_ADV_FAST_TIMEOUT_IN_SECONDS   (20)  		/*20s*/
#define APP_ADV_FAST_TIMEOUT_IN_SECONDS_LDM   (7)  	/*7s*/
#define APP_ADV_NORMAL_TIMEOUT_IN_SECONDS (60)		/*60s*/
#define APP_ADV_SLOW_TIMEOUT_IN_SECONDS   (0)  		/*forever*/

#define APP_TIMER_PRESCALER               0                                            /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE           6                                            /**< Size of timer operation queues. */

//#define BATTERY_LEVEL_MEAS_INTERVAL       APP_TIMER_TICKS(120000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). This value corresponds to 120 seconds. */
//#define CTRL_PROT_NOTIF_INTERVAL  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)
//#define NOTI_CHAN_NOTIF_INTERVAL  APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)
#define WIRELESS_RECV_INTERVAL  APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)
#define DELAY_COUNT_INTERVAL    APP_TIMER_TICKS(20, APP_TIMER_PRESCALER)
#define TASK_LOOP_INTERVAL      APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)
#define SYSMODE_CHECK_INTERVAL  APP_TIMER_TICKS(25, APP_TIMER_PRESCALER)

APP_TIMER_DEF(m_wireless_recv_timer_id);
APP_TIMER_DEF(m_delay_count_timer_id);
APP_TIMER_DEF(m_task_loop_timer_id);
APP_TIMER_DEF(m_sysmode_check_timer_id);

#define MIN_CONN_INTERVAL                 MSEC_TO_UNITS(20, UNIT_1_25_MS)              /**< Minimum acceptable connection interval (20ms).  */
#define MAX_CONN_INTERVAL                 MSEC_TO_UNITS(75, UNIT_1_25_MS)              /**< Maximum acceptable connection interval (75ms). */
#define SLAVE_LATENCY                     0                                            /**< Slave latency. */
#define CONN_SUP_TIMEOUT                  MSEC_TO_UNITS(4000, UNIT_10_MS)              /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY     APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT      3                                            /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                    1                                            /**< Perform bonding. */
#define SEC_PARAM_MITM                    0                                            /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                    0                                            /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                0                                            /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES         BLE_GAP_IO_CAPS_NONE                         /**< No I/O capabilities. */
#define SEC_PARAM_OOB                     0                                            /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE            7                                            /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE            16                                           /**< Maximum encryption key size. */

#define INITIAL_LLS_ALERT_LEVEL           BLE_CHAR_ALERT_LEVEL_NO_ALERT                /**< Initial value for the Alert Level characteristic in the Link Loss service. */
//-40, -20, -16, -12, -8, -4, 0, 3, and 4 dBm
#define TX_POWER_LEVEL                    (4)                                          /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS     600                                          /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION      6                                            /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS    270                                          /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */

#define DEAD_BEEF                         0xDEADBEEF                                   /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                  1024//256                                    /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                  256                                          /**< UART RX buffer size. */

ble_custom_t m_custom;
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

#define SAMPLES_IN_BUFFER 5
volatile uint8_t state = 1;

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(2);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;

nrf_drv_wdt_channel_id m_channel_id;
g_ble_device_type_t g_ble_device_type;
uint8_t g_batt_level = 100;
uint8_t g_enter_boot_flag ;

#define BLE_FW_REV                            "NBT-MB-002_A20_B02 V1.10.0"
#define BLE_HW_REV                            "NBT-MB-002_A20 V0.2"

								  
static uint8_t manuf_data_u8[10] ;

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#ifdef ADC_PRESENT
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
		((((ADC_VALUE) * ADC_REF_VBG_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_INPUT_PRESCALER)
#else // SAADC_PRESENT
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
		((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)
#endif // ADC_PRESENT

#define TEMP_TYPE_AS_CHARACTERISTIC     0                                           /**< Determines if temperature type is given as characteristic (1) or as a field of measurement (0). */

//static ble_db_discovery_t               m_ble_db_discovery;                      /**< Structure used to identify the DB Discovery module. */
ble_bas_t                          m_bas;                                   /**< Structure used to identify the battery service. */
static ble_hts_t m_hts;                                       /**< Structure used to identify the health thermometer service. */

static bool              m_hts_meas_ind_conf_pending = false; /**< Flag to keep track of when an indication confirmation is pending. */

static sensorsim_cfg_t   m_temp_celcius_sim_cfg;              /**< Temperature simulator configuration. */
static sensorsim_state_t m_temp_celcius_sim_state;            /**< Temperature simulator state. */

#define SERVICE_UUID_DEWALT   0xFACE
#define SERVICE_UUID_STANLEY  0xCAFE  //first PTI Byte 0xD0 is for Stanley, others are for Dewalt!

#define CUSTOM_SERVICE_UUID   SERVICE_UUID_DEWALT

ble_uuid_t m_adv_uuid0 = {CUSTOM_SERVICE_UUID, BLE_UUID_TYPE_BLE};

ble_advdata_manuf_data_t m_adv_manuf_data;

static uint32_t m_app_adv_interval_fast;
static uint32_t m_app_adv_fast_timeout;

uint32_t g_adv_fast_timeout_cnt;
uint8_t g_is_advertising;
uint8_t g_is_adv_interval_changed;
uint8_t g_status_code = DEVICE_STATUS_CODE_DEFAULT;
uint8_t g_is_connected = 0;
common_infomation_t common_infomation ;
device_brand_t device_brand ;
ble_adv_mode_t ble_adv_mode ;
static uint32_t read_LRL_power_delay;
static uint8_t read_LRL_power_flag;
uint8_t g_LRL_is_power_on;
uint8_t g_coincell_level ;
uint16_t safe_login_timeout;
uint8_t wait_safe_login_flag;
g_device_time_t g_device_time =
{
	2018, 1, 1, 0, 0, 0
};

#if USER_MEM_EN
uint8_t m_user_mem[1024];
ble_user_mem_block_t m_mem_block;
#endif

static void find_device_infomation_record(void) ;
static void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t * p_evt);
static void on_hts_evt(ble_hts_t * p_hts, ble_hts_evt_t * p_evt);
static void advertising_init(void);
void g_init_and_start_normal_adv(void);
extern ret_code_t update_user_flash(uint8_t *p_data, uint16_t length, uint16_t file_id, uint16_t record_key);
extern void flash_word_write(uint32_t * address, uint32_t value);
//void advertising_start(void);

//#ifdef ADC_PRESENT
//static nrf_adc_value_t adc_buf[1];
//#else
//static nrf_saadc_value_t adc_buf[2];
//#endif //ADC_PRESENT

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
	ret_code_t err_code;

	switch (p_evt->evt_id)
	{
		case PM_EVT_BONDED_PEER_CONNECTED:
		{
			NRF_LOG_INFO("Connected to a previously bonded device.\r\n");
		} break;

		case PM_EVT_CONN_SEC_SUCCEEDED:
		{
			NRF_LOG_INFO("Connection secured. Role: %d. conn_handle: %d, Procedure: %d, Peer_id: %d\r\n",
						 ble_conn_state_role(p_evt->conn_handle),
						 p_evt->conn_handle,
						 p_evt->params.conn_sec_succeeded.procedure,
			             p_evt->peer_id);
		} break;

		case PM_EVT_CONN_SEC_FAILED:
		{
			/* Often, when securing fails, it shouldn't be restarted, for security reasons.
			 * Other times, it can be restarted directly.
			 * Sometimes it can be restarted, but only after changing some Security Parameters.
			 * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
			 * Sometimes it is impossible, to secure the link, or the peer device does not support it.
			 * How to handle this error is highly application dependent. */
			//gongsheng
			NRF_LOG_INFO("PM_EVT_CONN_SEC_FAILED!\r\n");
		} break;

		case PM_EVT_CONN_SEC_CONFIG_REQ:
		{
			// Reject pairing request from an already bonded peer.
			pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
			pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
		} break;

		case PM_EVT_STORAGE_FULL:
		{
			// Run garbage collection on the flash.
			err_code = fds_gc();
			if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
			{
				// Retry.
			}
			else
			{
				APP_ERROR_CHECK(err_code);
			}
		} break;

		case PM_EVT_PEERS_DELETE_SUCCEEDED:
		{
			if(RFsystem.SYS_Mode != RF_OPEN && RFsystem.SYS_Mode != NOTHING)
			{
				//advertising_start();
			}
		} break;

		case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
		{
			// The local database has likely changed, send service changed indications.
			pm_local_database_has_changed();
		} break;

		case PM_EVT_PEER_DATA_UPDATE_FAILED:
		{
			// Assert.
			APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
		} break;

		case PM_EVT_PEER_DELETE_FAILED:
		{
			// Assert.
			APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
		} break;

		case PM_EVT_PEERS_DELETE_FAILED:
		{
			// Assert.
			APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
		} break;

		case PM_EVT_ERROR_UNEXPECTED:
		{
			// Assert.
			APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
		} break;

		case PM_EVT_CONN_SEC_START:
		{
//			NRF_LOG_INFO("PM_EVT_CONN_SEC_START!\r\n");
//			//check peer_count
//			uint32_t peer_cnt = pm_peer_count();
//			NRF_LOG_INFO("  peer_cnt=%d\r\n", peer_cnt);
//			if(peer_cnt <= MAX_PEER_COUNT)
//			{//the current peri included
//				g_peer_new_en = true;
//			}
//			else
//			{
//				g_peer_new_en = false;
//			}
		}
		break;
		case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
		case PM_EVT_PEER_DELETE_SUCCEEDED:
		case PM_EVT_LOCAL_DB_CACHE_APPLIED:
		case PM_EVT_SERVICE_CHANGED_IND_SENT:
		case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
		default:
			break;
	}
}

//static void current_time_error_handler(uint32_t nrf_error)
//{
//    APP_ERROR_HANDLER(nrf_error);
//}

/*
static void ctrl_prot_notif_timeout_handle(void *p_context)
{
	ble_custom_t *p_custom = &m_custom;
	
	UNUSED_PARAMETER(p_context);
	
	if(p_custom->is_ctrl_prot_notification_enabled)
	{
	}
}

static void noti_chan_notif_timeout_handle(void *p_context)
{
	ble_custom_t *p_custom = &m_custom;
	
	UNUSED_PARAMETER(p_context);
	
	if(p_custom->is_noti_chan_notification_enabled)
	{
	}
}*/

static void led_status(void)
{
	switch(RFsystem.SYS_Mode)
	{
		case BT_OPEN:
			RFsystem.runRF_LED = off;
		break;
		case RF_OPEN:	
			RFsystem.runRF_LED = flash;
		break;
		case ALL_OPEN:
			RFsystem.runRF_LED = flash;
		break;			
		default:
			RFsystem.runRF_LED = off;
		break;
	}
}

void g_disconnect_ble(void)
{
	if(g_is_connected && (m_conn_handle != BLE_CONN_HANDLE_INVALID))
	{
		nrf_drv_wdt_channel_feed(m_channel_id);
		sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		nrf_delay_ms(10);
		g_is_connected = 0;
	}
}

/**
 * @brief Handler for timer events.
 */
void timer_led_event_handler(nrf_timer_event_t event_type, void *p_context)
{
//	static uint32_t i;
//	uint32_t led_to_invert = ((i++) % LEDS_NUMBER);
	//static uint16_t timer1_count = 0;
	extern unsigned int hopping_time;
	
//	timer1_count++;

//	if(timer1_count >= 5)  //5*20ms=100ms
//	{
//		if(g_is_advertising && (RFsystem.SYS_Mode == NOTHING))
//		{
//			g_disconnect_ble();
//			RFsystem.sysmode_changed |= SYSMODE_CHANGED_FLAG_STOP_ADV;
//		}
//	}
	switch(event_type)
	{
		case NRF_TIMER_EVENT_COMPARE0:
		{
			//bsp_board_led_invert(led_to_invert);
			//
			if(hopping_time)
			{
				hopping_time--;
			}
			//check si4432 interrupt..
			if(nrf_gpio_pin_read(SI4432_NIRQ) == 0)
			{
				if(RF_flag & 0x08)	 
				{
					RF_flag |= 0x01;
					RF_flag &= ~0x08;
				}
//				else
//				{
//					RFReceive();
//				}
			}
		}
		break;

		default:
			//Do nothing.
		break;
	}
}

//100ms
static void wireless_recv_timeout_handle(void *p_context)
{
//	ble_custom_t *p_custom = &m_custom;
	
	UNUSED_PARAMETER(p_context);
	//wireless_receive
	uint16_t cnt = 0;
//	static uint8_t recv_buf[200];
//	memset(recv_buf, 0, 200);
	static uint8_t hopping_noise[4];  //4 Bytes, same, LSByte of GET_DEVICE_NAME_ID_FROM_SUB
	memset(hopping_noise, GET_DEVICE_NAME_ID_FROM_SUB & 0xFF, 4);

	if((RFsystem.SYS_Mode != RF_OPEN) && (RFsystem.SYS_Mode != ALL_OPEN))
	{return;}

	if((RF_flag & 0x05) == 0x05)
	{
		if(0x04 == SpiRfReadRegister(InterruptEnable1))
		{
			RF_flag &= 0xFE;
			RFIdle();
			RFReceive();
			return;
		}
		cnt = RFPacketReceived(g_rf_buffer.recv_data);
		//test
		static uint16_t dbg_cnt2 = 0;
		static uint8_t ItStatus1, ItStatus2, ItEn1, ItEn2;
		if((g_rf_buffer.recv_data[1] == 0x05/*g_rf_buffer.recv_data[2]*/) \
		&& (g_rf_buffer.recv_data[2] == 0x05/*g_rf_buffer.recv_data[3]*/) \
		&& (g_rf_buffer.recv_data[3] == 0x05/*g_rf_buffer.recv_data[4]*/))
		{
			ItStatus1 = SpiRfReadRegister(InterruptStatus1);
			ItStatus2 = SpiRfReadRegister(InterruptStatus2);
			ItEn1 = SpiRfReadRegister(InterruptEnable1);
			ItEn2 = SpiRfReadRegister(InterruptEnable2);
			dbg_cnt2++;
		}
		if((!cnt) || ((cnt == 4) && (memcmp(g_rf_buffer.recv_data, hopping_noise, 4) == 0)))
		{
			RF_flag &= 0xFE;
			RFIdle();
			RFReceive();
			return;
		}
		//if(RFsystem.SYS_Mode == ALL_OPEN || RFsystem.SYS_Mode == RF_OPEN)
		{
			g_rf_buffer.recv_count = cnt;
		//	memcpy(g_rf_buffer.recv_data, recv_buf, cnt);
		}

		RF_flag &= 0xFE;
		RFIdle();
		RFReceive();
	}
}

//20ms
static void delay_count_timeout_handle(void *p_context)
{
//	ble_custom_t *p_custom = &m_custom;
	
	UNUSED_PARAMETER(p_context);
	//
	static unsigned char flash_set;
	static unsigned char flash_once_save;
	
	static uint32_t count = 0;
	static uint32_t timeout = 0;
	
	count++;
	timeout++;
	
	if(read_LRL_power_delay < 100)
	{
		read_LRL_power_delay++;
	}
	else
	{
		read_LRL_power_flag = 1;
		read_LRL_power_delay = 0;
	}
	
	if(wait_safe_login_flag)
	{
		safe_login_timeout--;
		if(safe_login_timeout==0)
		{
			g_disconnect_ble() ;
			wait_safe_login_flag = 0 ;
		}
	}
	
	if(g_uart_data.rx_idle_count_en)
	{
		g_uart_data.rx_idle_count++;
		if(g_uart_data.rx_idle_count >= 2)
		{//recv over!!
			g_uart_data.rx_idle_count_en = 0;
			g_uart_data.rx_idle_count = 0;
			//
			g_uart_data.len = g_uart_data.idx;
			uart_cmd_event(&g_uart_data);  //uart event
			g_uart_data.idx = 0;
			memset(&g_uart_data, 0, sizeof(g_uart_data));
		} 
	}
	
	if(g_is_adv_interval_changed)
	{
		if(g_adv_fast_timeout_cnt)
		{
			g_adv_fast_timeout_cnt--;
		}
		else
		{
			if(g_ble_device_type == LDM)
			{//LDM meets FAST 20s
				common_infomation.manufacturer_byte_11_flag &= ~DEVICE_STATUS_CODE_BUTTON_PUSHED;
				sd_ble_gap_adv_stop();
				nrf_delay_ms(20);
				g_init_and_start_normal_adv() ;				
			}
			else 
			{//SUB,HUB
				if(m_app_adv_interval_fast == APP_ADV_INTERVAL_FAST_20_MS)
				{//meets FAST 20s
					common_infomation.manufacturer_byte_11_flag &= ~DEVICE_STATUS_CODE_BUTTON_PUSHED;
					sd_ble_gap_adv_stop();
					nrf_delay_ms(20);
					g_init_and_start_normal_adv() ;
				}
				else if(m_app_adv_interval_fast == APP_ADV_INTERVAL_FAST_152_MS)
				{//meets NORMAL 60s
					ble_adv_mode = BLE_ADV_MODE_SLOW;
					sd_ble_gap_adv_stop();
					g_is_advertising = 0;
					g_is_adv_interval_changed = 0;
				}
			}
		}
	}/*adjust adv interval*/
	
	if(timeout >= 5)  //100/20
	{
		if((RFsystem.ch_scan_time != 0xFF) && (RFsystem.ch_scan_time > 0))
		{
			RFsystem.ch_scan_time--;
		}

		if(g_rf_buffer.Re_Wait_delay)
		{
			g_rf_buffer.Re_Wait_delay--;
		}
		timeout = 0;
	}
	
	if(count >= 5)  //100/20
	{
		flash_set++;
		flash_set %= 10;

		switch(RFsystem.runRF_LED)
		{
			case on:
				wls_led_on();
			break;
			case off:
				wls_led_off();
			break;
			case flash:
				if(flash_set >= 5)
				{
					wls_led_on();
				}
				else
				{
					wls_led_off();
				}
			break;
			case once_flash:
				wls_led_on();
				flash_once_save++;
				if(flash_once_save > 10)
				{
					wls_led_off();
					RFsystem.runRF_LED = off;
					flash_once_save = 0;
				}
			break;
			default:
			break;
		}

		switch(RFsystem.event_LED)
		{
			case on:
				bt_led_on();
			break;
			case off:
				bt_led_off();
			break;
			case flash:
				if(flash_set >= 5)
				{
					bt_led_on();
				}
				else
				{
					bt_led_off();
				}
			break;
			case once_flash:
				bt_led_on();
				flash_once_save++;
				if(flash_once_save > 10)
				{
					bt_led_off();
					RFsystem.event_LED = off;
					flash_once_save = 0;
				}
			break;
			default:
			break;
		}
		count = 0;
	}
	
}

//25ms
static void sysmode_check_timeout_handle(void *p_context)
{
	UNUSED_PARAMETER(p_context);
	
	static uint32_t check_adv_count = 0;

	
	if(RFsystem.sysmode_changed)
	{
		if(RFsystem.sysmode_changed & SYSMODE_CHANGED_FLAG_STOP_ADV)
		{//check adv, we will stop adv
			if(g_is_advertising)
			{
				sd_ble_gap_adv_stop();
				g_is_advertising = 0;
			}
			check_adv_count++;
			if(check_adv_count >= 12)
			{
				check_adv_count = 0;
				RFsystem.sysmode_changed &= ~SYSMODE_CHANGED_FLAG_STOP_ADV;
			}
		}/*check adv*/
		else if(RFsystem.sysmode_changed & SYSMODE_CHANGED_FLAG_INIT_RF)
		{//if we need to stop adv, stop adv at first, then init rf!!
			if(RFsystem.SYS_Mode == RF_OPEN)
			{//init rf!!
				memset(RFsystem.device, 0, 20 * 4);
				RFsystem.device_count = 0;
				g_rf_chnl_nbr = 3;
				RFsystem.ch_scan_time = RF_SCAN_PERIOD;
				
				WLS_enable();
				si4432_init();
				RFIdle();
				RFReceive();
				RF_offset_event();
			}
			RFsystem.sysmode_changed &= ~SYSMODE_CHANGED_FLAG_INIT_RF;
		}/*need to initialize RF*/
		else
		{//SYSMODE_CHANGED_FLAG_NORMAL
			RFsystem.sysmode_changed = 0;
		}
	}
}

//50ms
static void task_loop_timeout_handle(void *p_context)
{
//	ble_custom_t *p_custom = &m_custom;
	
	UNUSED_PARAMETER(p_context);
	
	//hub, hopping
	frequency_hopping();
	//wireless recv
	wireless_receive_handler();
	g_rf_buffer.recv_count = 0;
	//repeat
	wireless_repeat();
	
	led_status();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
	uint32_t err_code;

	// Initialize timer module.
	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

/*	// Create battery timer.
//	err_code = app_timer_create(&m_battery_timer_id,
//								APP_TIMER_MODE_REPEATED,
//								battery_level_meas_timeout_handler);
//	APP_ERROR_CHECK(err_code);
//	
//	err_code = app_timer_create(&m_ctrl_prot_notif_timer_id,
//								APP_TIMER_MODE_REPEATED,
//								ctrl_prot_notif_timeout_handle);
//	APP_ERROR_CHECK(err_code);

//	err_code = app_timer_create(&m_noti_chan_notif_timer_id,
//								APP_TIMER_MODE_REPEATED,
//								noti_chan_notif_timeout_handle);
//	APP_ERROR_CHECK(err_code);*/
	
	err_code = app_timer_create(&m_wireless_recv_timer_id,
								APP_TIMER_MODE_REPEATED,
								wireless_recv_timeout_handle);
	APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_create(&m_delay_count_timer_id,
								APP_TIMER_MODE_REPEATED,
								delay_count_timeout_handle);
	APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_create(&m_task_loop_timer_id,
								APP_TIMER_MODE_REPEATED,
								task_loop_timeout_handle);
	APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_create(&m_sysmode_check_timer_id,
								APP_TIMER_MODE_REPEATED,
								sysmode_check_timeout_handle);
	APP_ERROR_CHECK(err_code);

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

//			if((g_uart_data.data[g_uart_data.idx-1] == '\n') || (g_uart_data.idx >= (MAX_UART_DATA_LEN)) \
//				|| (g_uart_data.data[g_uart_data.idx-1] == '}') || (g_uart_data.data[g_uart_data.idx-1] == ']'))
//			{
//				g_uart_data.len = g_uart_data.idx;
//				uart_cmd_event(&g_uart_data);  //uart event
//				g_uart_data.idx = 0;
//			}
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
/**@snippet [UART Initialization] */

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
	uint32_t                err_code;
	ble_gap_conn_params_t   gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode,
										  (const uint8_t *)common_infomation.device_name,
										  strlen((char *)common_infomation.device_name));
	APP_ERROR_CHECK(err_code);

//    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_KEYRING);
//    APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency     = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);

	//Set TX Power!!!
	err_code = sd_ble_gap_tx_power_set(TX_POWER_LEVEL);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
	uint32_t err_code;
	
//	err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//    APP_ERROR_CHECK(err_code);

	// Prepare wakeup buttons.
	err_code = bsp_btn_ble_sleep_mode_prepare();
	APP_ERROR_CHECK(err_code);

	// Go to system-off mode (this function will not return; wakeup will cause a reset).
	err_code = sd_power_system_off();
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
//    uint32_t err_code;

	switch (ble_adv_evt)
	{
		case BLE_ADV_EVT_FAST:
			NRF_LOG_INFO("Fast advertising.\r\n");
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//            APP_ERROR_CHECK(err_code);
			break; // BLE_ADV_EVT_FAST

		case BLE_ADV_EVT_IDLE:
			sleep_mode_enter();
			break; // BLE_ADV_EVT_IDLE

		default:
			break;
	}
}

/*
static void custom_data_handler(ble_custom_t *p_custom, uint8_t *p_data, uint16_t length)
{
	uint16_t op_code;
	
	op_code = (p_data[0] << 8) | p_data[1];
	
	switch(op_code)
	{
		case 0x0012:
		{
			uart_send_string(p_data + 2, length - 2);
		}
		break;
	}
}
*/

extern void protocol_handler(ble_custom_t *p_custom, uint8_t *p_data, uint16_t length);



/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
	uint32_t               err_code;
	ble_advdata_t          advdata;
	ble_adv_modes_config_t options;

	// Build and set advertising data.
	memset(&advdata, 0, sizeof(advdata));

	advdata.name_type               = BLE_ADVDATA_FULL_NAME;
	advdata.short_name_len          = MAX_DEV_NAME_LEN ;

	advdata.include_appearance      = false;//true;
	advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	advdata.uuids_more_available.uuid_cnt = 1;
	advdata.uuids_more_available.p_uuids  = &m_adv_uuid0;
	
	m_adv_manuf_data.company_identifier = common_infomation.company_identifier_code[0]<<8 | common_infomation.company_identifier_code[1];
	memcpy(manuf_data_u8, common_infomation.device_pti, 1) ;
	memcpy(&manuf_data_u8[1], common_infomation.deviceid, 6) ;
	memcpy(&manuf_data_u8[7], &common_infomation.device_pti[1], 2) ;
	memcpy(&manuf_data_u8[9], &common_infomation.manufacturer_byte_11_flag, 1) ;
	m_adv_manuf_data.data.size = sizeof(manuf_data_u8) / sizeof(manuf_data_u8[0]);
	m_adv_manuf_data.data.p_data = manuf_data_u8;
	advdata.p_manuf_specific_data = &m_adv_manuf_data;

	memset(&options, 0, sizeof(options));
	options.ble_adv_fast_enabled  = true;
	options.ble_adv_fast_interval = m_app_adv_interval_fast;
	options.ble_adv_fast_timeout  = m_app_adv_fast_timeout;
	options.ble_adv_slow_enabled  = true;
	options.ble_adv_slow_interval = APP_ADV_INTERVAL_SLOW_2000_MS;
	options.ble_adv_slow_timeout  = APP_ADV_SLOW_TIMEOUT_IN_SECONDS;

	err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
	APP_ERROR_CHECK(err_code);
}

 
/**@brief Function for initializing the Battery Service.
 */
static void bas_init(void)
{
	uint32_t       err_code;
	ble_bas_init_t bas_init_obj;

	memset(&bas_init_obj, 0, sizeof(bas_init_obj));

	bas_init_obj.evt_handler          = on_bas_evt;
	bas_init_obj.support_notification = true;
	bas_init_obj.p_report_ref         = NULL;
	bas_init_obj.initial_batt_level   = g_batt_level;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_char_attr_md.cccd_write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_char_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init_obj.battery_level_char_attr_md.write_perm);

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_report_read_perm);

	err_code = ble_bas_init(&m_bas, &bas_init_obj);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Temperature Service.
 */
static void hts_init(void)
{
    uint32_t         err_code;
    ble_hts_init_t   hts_init;

    // Initialize Health Thermometer Service
    memset(&hts_init, 0, sizeof(hts_init));

    hts_init.evt_handler                 = on_hts_evt;
    hts_init.temp_type_as_characteristic = TEMP_TYPE_AS_CHARACTERISTIC;
    hts_init.temp_type                   = BLE_HTS_TEMP_TYPE_BODY;

    // Here the sec level for the Health Thermometer Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hts_init.hts_meas_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_meas_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_meas_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.hts_temp_type_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_temp_type_attr_md.write_perm);

    err_code = ble_hts_init(&m_hts, &hts_init);
    APP_ERROR_CHECK(err_code);
}


static void dis_init(void)
{
	uint32_t         err_code;
	ble_dis_init_t   dis_init_obj;
	ble_dis_sys_id_t sys_id;
	
	memset(&dis_init_obj, 0, sizeof(dis_init_obj));

	ble_srv_ascii_to_utf8(&dis_init_obj.model_num_str, common_infomation.model);
	ble_srv_ascii_to_utf8(&dis_init_obj.serial_num_str, common_infomation.sn);  /*SERIAL_NUM*/
	ble_srv_ascii_to_utf8(&dis_init_obj.fw_rev_str,     BLE_FW_REV);
	ble_srv_ascii_to_utf8(&dis_init_obj.hw_rev_str,     BLE_HW_REV);
	ble_srv_ascii_to_utf8(&dis_init_obj.sw_rev_str,   (char *) common_infomation.main_mcu_fw_version);
	ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, (char *)common_infomation.manufact_name);

	uint8_t str_sys_id[6];
	str_sys_id[0] = common_infomation.deviceid[4];  
	str_sys_id[1] = common_infomation.deviceid[3];  
	str_sys_id[2] = common_infomation.deviceid[2]; 
	str_sys_id[3] = common_infomation.deviceid[1];  
	str_sys_id[4] = common_infomation.deviceid[0]; 
	str_sys_id[5] = common_infomation.deviceid[5]; 
	
	sys_id.manufacturer_id = 0;//MANUFACTURER_ID;
	for(uint8_t i=0;i<5;i++)
	{
		sys_id.manufacturer_id <<= 8;
		sys_id.manufacturer_id |= str_sys_id[i];  
	}
	sys_id.organizationally_unique_id = 0;//ORG_UNIQUE_ID;
	for(uint8_t i=0;i<1;i++)
	{
		sys_id.organizationally_unique_id <<= 8;
		sys_id.organizationally_unique_id |= str_sys_id[i+5];  
	}
	dis_init_obj.p_sys_id = &sys_id;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init_obj.dis_attr_md.read_perm);
	//test for key-pairing
	//BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&dis_init_obj.dis_attr_md.read_perm);
	//BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&dis_init_obj.dis_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init_obj.dis_attr_md.write_perm);

	err_code = ble_dis_init(&dis_init_obj);
	APP_ERROR_CHECK(err_code);
}

static void custom_init(void)
{
	uint32_t          err_code;
	ble_custom_init_t custom_init_obj;
	
	memset(&custom_init_obj, 0, sizeof(custom_init_obj));
	
	custom_init_obj.data_handler = protocol_handler;//custom_data_handler;
	
	m_custom.uuid_type    = m_adv_uuid0.type;  //BLE_UUID_TYPE_BLE;
	m_custom.service_uuid = m_adv_uuid0.uuid;  //CUSTOM_SERVICE_UUID
	
	err_code = ble_custom_init(&m_custom, &custom_init_obj);
	
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the services that will be used by the application.
 */
static void services_init(void)
{	
	//Initialize Device Information Service
	dis_init();
	//Battery service
	bas_init();
	//Temperature service
	if(g_ble_device_type == SUB)
	{
		hts_init();
	}
	//user add +
	custom_init();
}

static void application_timers_start(void)
{
	uint32_t err_code;
	
//	err_code = app_timer_start(m_ctrl_prot_notif_timer_id, CTRL_PROT_NOTIF_INTERVAL, NULL);
//	APP_ERROR_CHECK(err_code);
//	
//	err_code = app_timer_start(m_noti_chan_notif_timer_id, NOTI_CHAN_NOTIF_INTERVAL, NULL);
//	APP_ERROR_CHECK(err_code);
	
	{
		//100ms
		err_code = app_timer_start(m_wireless_recv_timer_id, WIRELESS_RECV_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code);
	}
	//20ms
	err_code = app_timer_start(m_delay_count_timer_id, DELAY_COUNT_INTERVAL, NULL);
	APP_ERROR_CHECK(err_code);
	//50ms
	err_code = app_timer_start(m_task_loop_timer_id, TASK_LOOP_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code);
	//25ms
	err_code = app_timer_start(m_sysmode_check_timer_id, SYSMODE_CHECK_INTERVAL, NULL);
	APP_ERROR_CHECK(err_code);
}



/** @brief Database discovery module initialization.
 */
//static void db_discovery_init(void)
//{
//    uint32_t err_code = ble_db_discovery_init(db_disc_handler);

//    APP_ERROR_CHECK(err_code);
//}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
	uint32_t               err_code;
	ble_conn_params_init_t cp_init;

	memset(&cp_init, 0, sizeof(cp_init));

	cp_init.p_conn_params                  = NULL;
	cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
	cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
	cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
	cp_init.disconnect_on_fail             = true;
	cp_init.evt_handler                    = NULL;
	cp_init.error_handler                  = conn_params_error_handler;

	err_code = ble_conn_params_init(&cp_init);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Battery Service events.
 *
 * @details This function will be called for all Battery Service events which are passed to the
 |          application.
 *
 * @param[in] p_bas  Battery Service structure.
 * @param[in] p_evt  Event received from the Battery Service.
 */
static void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t * p_evt)
{
	//uint32_t err_code;

	switch (p_evt->evt_type)
	{
		case BLE_BAS_EVT_NOTIFICATION_ENABLED:
			// Start battery timer
//			err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
//			APP_ERROR_CHECK(err_code);
			break; // BLE_BAS_EVT_NOTIFICATION_ENABLED

		case BLE_BAS_EVT_NOTIFICATION_DISABLED:
//			err_code = app_timer_stop(m_battery_timer_id);
//			APP_ERROR_CHECK(err_code);
			break; // BLE_BAS_EVT_NOTIFICATION_DISABLED

		default:
			// No implementation needed.
			break;
	}
}

/**@brief Function for populating simulated health thermometer measurement.
 */
static void hts_sim_measurement(ble_hts_meas_t * p_meas)
{
    static ble_date_time_t time_stamp = { 2012, 12, 5, 11, 50, 0 };

    uint32_t celciusX100;

    p_meas->temp_in_fahr_units = false;
    p_meas->time_stamp_present = true;
    p_meas->temp_type_present  = (TEMP_TYPE_AS_CHARACTERISTIC ? false : true);

    celciusX100 = sensorsim_measure(&m_temp_celcius_sim_state, &m_temp_celcius_sim_cfg);

    p_meas->temp_in_celcius.exponent = -2;
    p_meas->temp_in_celcius.mantissa = celciusX100;
    p_meas->temp_in_fahr.exponent    = -2;
    p_meas->temp_in_fahr.mantissa    = (32 * 100) + ((celciusX100 * 9) / 5);
    p_meas->time_stamp               = time_stamp;
    p_meas->temp_type                = BLE_HTS_TEMP_TYPE_FINGER;

    // update simulated time stamp
    time_stamp.seconds += 27;
    if (time_stamp.seconds > 59)
    {
        time_stamp.seconds -= 60;
        time_stamp.minutes++;
        if (time_stamp.minutes > 59)
        {
            time_stamp.minutes = 0;
        }
    }
}

/**@brief Function for simulating and sending one Temperature Measurement.
 */
static void temperature_measurement_send(void)
{
    ble_hts_meas_t simulated_meas;
    uint32_t       err_code;

    if (!m_hts_meas_ind_conf_pending)
    {
        hts_sim_measurement(&simulated_meas);

        err_code = ble_hts_measurement_send(&m_hts, &simulated_meas);

        switch (err_code)
        {
            case NRF_SUCCESS:
                // Measurement was successfully sent, wait for confirmation.
                m_hts_meas_ind_conf_pending = true;
                break;

            case NRF_ERROR_INVALID_STATE:
                // Ignore error.
                break;

            default:
                APP_ERROR_HANDLER(err_code);
                break;
        }
    }
}

/**@brief Function for handling the Health Thermometer Service events.
 *
 * @details This function will be called for all Health Thermometer Service events which are passed
 *          to the application.
 *
 * @param[in] p_hts  Health Thermometer Service structure.
 * @param[in] p_evt  Event received from the Health Thermometer Service.
 */
static void on_hts_evt(ble_hts_t * p_hts, ble_hts_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HTS_EVT_INDICATION_ENABLED:
            // Indication has been enabled, send a single temperature measurement
            temperature_measurement_send();
            break;

        case BLE_HTS_EVT_INDICATION_CONFIRMED:
            m_hts_meas_ind_conf_pending = false;
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
	uint32_t err_code = NRF_SUCCESS;

	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_DISCONNECTED:
			NRF_LOG_INFO("Disconnected.\r\n");
		
			m_conn_handle = BLE_CONN_HANDLE_INVALID;
			nrf_gpio_pin_set(BLE_APP_DETECT);  /*unconnected, High*/
//			common_infomation.manufacturer_byte_11_flag &= ~DEVICE_STATUS_CODE_CLIENT_CONNECTED;		
			sd_ble_gap_adv_stop();
			g_is_connected = 0;
			find_device_infomation_record(); //must after peer_manager_init and before gap_params_init,advertising_init		
			gap_params_init();
			advertising_init();
			advertising_start();
				
			if(g_enter_boot_flag == 1)
			{
				NVIC_SystemReset();	
			}				
			
		break; // BLE_GAP_EVT_DISCONNECTED

		case BLE_GAP_EVT_CONNECTED:
		
			NRF_LOG_INFO("Connected.\r\n");
			//conn handle!
			m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			nrf_gpio_pin_clear(BLE_APP_DETECT);  /*connected, Low*/
//			common_infomation.manufacturer_byte_11_flag |= DEVICE_STATUS_CODE_CLIENT_CONNECTED;			
			g_is_connected = 1;
			g_is_advertising = 0;
//			g_advertising_init_beacon();
//			g_advertising_start_beacon();
			if(g_ble_device_type ==SUB &&\
				(common_infomation.manufacturer_byte_11_flag & DEVICE_STATUS_CODE_UNCOMMISSIONED))
			{
				safe_login_timeout = 1500 ; //1500*20ms = 30s
				wait_safe_login_flag = 1 ;
			}
		break; // BLE_GAP_EVT_CONNECTED

		case BLE_GATTC_EVT_TIMEOUT:
			// Disconnect on GATT Client timeout event.
			NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
			err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
											 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(err_code);
			break; // BLE_GATTC_EVT_TIMEOUT

		case BLE_GATTS_EVT_TIMEOUT:
			// Disconnect on GATT Server timeout event.
			NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
			err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
											 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(err_code);
			break; // BLE_GATTS_EVT_TIMEOUT

		case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
		{
			ble_gatts_evt_rw_authorize_request_t  req;
			ble_gatts_rw_authorize_reply_params_t auth_reply;

			req = p_ble_evt->evt.gatts_evt.params.authorize_request;

			if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
			{
				if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
					(req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
					(req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
				{
					if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
					{
						auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
					}
					else
					{
						auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
					}
					auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
					err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
															   &auth_reply);
					APP_ERROR_CHECK(err_code);
				}
			}
		} break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
		case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
			err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
													   NRF_BLE_MAX_MTU_SIZE);
			APP_ERROR_CHECK(err_code);
			break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

		default:
			// No implementation needed.
			break;
	}
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	ble_conn_state_on_ble_evt(p_ble_evt);
	pm_on_ble_evt(p_ble_evt);
//    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
	ble_conn_params_on_ble_evt(p_ble_evt);
//    ble_ias_on_ble_evt(&m_ias, p_ble_evt);
//    ble_lls_on_ble_evt(&m_lls, p_ble_evt);
	ble_bas_on_ble_evt(&m_bas, p_ble_evt);
	//user add +
	ble_custom_on_ble_evt(&m_custom, p_ble_evt);
	//----------
//    ble_ias_c_on_ble_evt(&m_ias_c, p_ble_evt);
//    ble_tps_on_ble_evt(&m_tps, p_ble_evt);
	bsp_btn_ble_on_ble_evt(p_ble_evt);
//	ble_advertising_on_ble_evt(p_ble_evt);
	on_ble_evt(p_ble_evt);
//	ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
	// Dispatch the system event to the fstorage module, where it will be
	// dispatched to the Flash Data Storage (FDS) module.
	fs_sys_event_handler(sys_evt);

	// Dispatch to the Advertising module last, since it will check if there are any
	// pending flash operations in fstorage. Let fstorage process system events first,
	// so that it can report correctly to the Advertising module.
	ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
	uint32_t err_code;

//	nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
	//new lf -- RC
	nrf_clock_lf_cfg_t clock_lf_cfg;
	clock_lf_cfg.source = NRF_CLOCK_LF_SRC_RC;
	clock_lf_cfg.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM;
	clock_lf_cfg.rc_ctiv = 16;
	clock_lf_cfg.rc_temp_ctiv = 2;

	// Initialize the SoftDevice handler module.
	SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

	ble_enable_params_t ble_enable_params;
	err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
													PERIPHERAL_LINK_COUNT,
													&ble_enable_params);
	APP_ERROR_CHECK(err_code);

	// Check the ram settings against the used number of links
	CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

	// Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
	ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
	err_code = softdevice_enable(&ble_enable_params);
	APP_ERROR_CHECK(err_code);

	// Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	APP_ERROR_CHECK(err_code);

	// Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
	ble_gap_sec_params_t sec_param;
	ret_code_t           err_code;

	err_code = pm_init();
	APP_ERROR_CHECK(err_code);

	if (erase_bonds)
	{
		err_code = pm_peers_delete();
		APP_ERROR_CHECK(err_code);
	}

	memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

	// Security parameters to be used for all security procedures.
	sec_param.bond           = SEC_PARAM_BOND;
	sec_param.mitm           = SEC_PARAM_MITM;
	sec_param.lesc           = SEC_PARAM_LESC;
	sec_param.keypress       = SEC_PARAM_KEYPRESS;
	sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
	sec_param.oob            = SEC_PARAM_OOB;
	sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
	sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
	sec_param.kdist_own.enc  = 1;
	sec_param.kdist_own.id   = 1;
	sec_param.kdist_peer.enc = 1;
	sec_param.kdist_peer.id  = 1;

	err_code = pm_sec_params_set(&sec_param);
	APP_ERROR_CHECK(err_code);

	err_code = pm_register(pm_evt_handler);
	APP_ERROR_CHECK(err_code);
}


void spi_init(void)
{
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
//	spi_config.ss_pin   = SPIM0_SS_PIN;
	spi_config.miso_pin = SPIM0_MISO_PIN;
	spi_config.mosi_pin = SPIM0_MOSI_PIN;
	spi_config.sck_pin  = SPIM0_SCK_PIN;
	spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
	spi_config.mode = NRF_DRV_SPI_MODE_0;
	spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
	APP_ERROR_CHECK(nrf_drv_spi_init(&m_spi_master_0, &spi_config, NULL/*spi_event_handler*/));
}

void myTimerInit()
{
	uint32_t time_ms = 20; //Time(in miliseconds) between consecutive compare events.
	uint32_t time_ticks;
	uint32_t err_code = NRF_SUCCESS;

	//Configure all leds on board.


	//Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
	err_code = nrf_drv_timer_init(&TIMER_LED, NULL, timer_led_event_handler);
	APP_ERROR_CHECK(err_code);

	time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_LED, time_ms);

	nrf_drv_timer_extended_compare(&TIMER_LED, NRF_TIMER_CC_CHANNEL0, \
								time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

	nrf_drv_timer_enable(&TIMER_LED);
}

void system_config(void)
{	
	if(g_ble_device_type == HUB||g_ble_device_type == SUB)
	{//for si4432 wireless
		spi_init();
	}
	led_gpio_init();
	
	//bluetooth
	memset(RFsystem.device, 0, 20 * 4);
	RFsystem.device_count = 0;
	g_rf_chnl_nbr = 3;
	RFsystem.ch_scan_time = RF_SCAN_PERIOD;
}



/**@brief Function for the Power manager.
 */
void power_manage(void)
{
	uint32_t err_code = sd_app_evt_wait();

	APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
 void advertising_start(void)
{
	uint32_t err_code;
	err_code = ble_advertising_start(ble_adv_mode);
	APP_ERROR_CHECK(err_code);
	g_is_advertising = 1;	
}

void timer_handler(nrf_timer_event_t event_type, void * p_context)
{

}


void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 60000ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 60);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
	   uint16_t coincell_batt = 0;
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            coincell_batt += p_event->data.done.p_buffer[i];
        }
	   coincell_batt = coincell_batt/SAMPLES_IN_BUFFER - 512;//1.8V - 512
	   g_coincell_level = coincell_batt*100/(938- 512) ; //3.6v-1024    3.3v-938
	   if(g_coincell_level > 100) 
	   {
		 g_coincell_level = 100 ;  
	   }
        m_adc_evt_counter++;
    }
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
	if (int_type == NRF_DRV_RTC_INT_TICK)
	{
		g_device_time.second++;
		if(g_device_time.second >= 60)
		{
			g_device_time.minute++;
			g_device_time.second = 0;
		}
		if(g_device_time.minute>=60)
		{
			g_device_time.hour++;
			g_device_time.minute = 0;
		}
		if(g_device_time.hour>=24)
		{
			g_device_time.day++;
			g_device_time.hour = 0;
		}
		switch(g_device_time.year)
		{
			case JANUARY:
			case	MARCH:
			case	MAY:
			case	JULY:
			case	AUGUST:
			case	OCTOBER:
			case	DECEMBER:
				if(g_device_time.day >= 31)
				{
					g_device_time.month++;
					g_device_time.day = 1;
				}
			break;
			case FEBRUARY:
				if(g_device_time.year%4 == 0)
				{
					if(g_device_time.day >= 29)
					{
						g_device_time.month++;
						g_device_time.day = 1;
					}
				}
				else
				{
					if(g_device_time.day >= 28)
					{
						g_device_time.month++;
						g_device_time.day = 1;
					}
				}
			break;
			case APRIL:
			case	JUNE:
			case SEPTEMBER:
			case	NOVEMBER:
				if(g_device_time.day >= 30)
				{
					g_device_time.month++;
					g_device_time.day = 1;
				}					
			break;
		}

		if(g_device_time.month >= DECEMBER)
		{
			g_device_time.year++;
			g_device_time.month = JANUARY;
		}
    }
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc, true);

    //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
    err_code = nrf_drv_rtc_cc_set(&rtc, 0, COMPARE_COUNTERTIME * 8, true);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}

/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void)
{
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

static void wdt_config(void)
{
	uint32_t err_code;
	//Configure WDT.
	nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
	config.reload_value = 30000;  //20s
	err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
	APP_ERROR_CHECK(err_code);
	nrf_drv_wdt_enable();
}

void g_init_and_start_fast_adv(void)
{
	if(g_ble_device_type == LDM)
	{
		m_app_adv_interval_fast = APP_ADV_INTERVAL_FAST_200_MS;
	}
	else	
	{
		m_app_adv_interval_fast = APP_ADV_INTERVAL_FAST_20_MS;
	}
	g_adv_fast_timeout_cnt  = APP_ADV_FAST_TIMEOUT_IN_SECONDS * 1000 / 20;		
	m_app_adv_fast_timeout  = 22;		//more than 20 s  
	g_is_adv_interval_changed = 1;
	ble_adv_mode = BLE_ADV_MODE_FAST;
	advertising_init();
	nrf_delay_ms(10);
	advertising_start();
}

void g_init_and_start_normal_adv(void)
{
	if(g_ble_device_type == LDM)
	{
		m_app_adv_interval_fast = APP_ADV_INTERVAL_FAST_400_MS;
	}
	else	
	{
		m_app_adv_interval_fast = APP_ADV_INTERVAL_FAST_152_MS;
	}
	m_app_adv_fast_timeout = 0;		/*forever*/ 
	if(g_ble_device_type == SUB && \
		common_infomation.manufacturer_byte_11_flag & DEVICE_STATUS_CODE_UNCOMMISSIONED)
	{
		g_adv_fast_timeout_cnt = APP_ADV_NORMAL_TIMEOUT_IN_SECONDS  * 1000 / 20;
		g_is_adv_interval_changed = 1;
	}
	else
	{
		g_is_adv_interval_changed = 0;
	}
	ble_adv_mode = BLE_ADV_MODE_FAST;
	advertising_init();
	nrf_delay_ms(10);
	advertising_start();
}

void g_init_and_start_slow_adv(void)
{
	ble_adv_mode = BLE_ADV_MODE_SLOW;
	advertising_init();
	nrf_delay_ms(10);
	advertising_start();
}


static void find_device_name(void)
{
	fds_record_desc_t  p_desc;
	fds_flash_record_t rec_flash;
	ret_code_t         ret;

	fds_find_token_t ftok;
	memset(&ftok, 0x00, sizeof(fds_find_token_t));
	memcpy(common_infomation.device_name, "NWiTool ", sizeof(common_infomation.device_name));
     ret = fds_record_find(BLE_INFO_FILE_ID, BLE_RECORD_KEY_DEVICE_NAME, &p_desc, &ftok);
	if (ret != FDS_SUCCESS)
	{
		return;
	}
	else
	{
		fds_record_open(&p_desc, &rec_flash);
		memcpy(common_infomation.device_name, rec_flash.p_data, sizeof(common_infomation.device_name));
	}
}

static void find_company_identifier_code(void)
{
	fds_record_desc_t  p_desc;
	fds_flash_record_t rec_flash;
	ret_code_t         ret;

	fds_find_token_t ftok;
	memset(&ftok, 0x00, sizeof(fds_find_token_t));
	memset(common_infomation.company_identifier_code, 0, sizeof(common_infomation.company_identifier_code));
     ret = fds_record_find(BLE_INFO_FILE_ID, BLE_RECORD_KEY_DEVICE_CIDC, &p_desc, &ftok);
	if (ret != FDS_SUCCESS)
	{
		return;
	}
	else
	{
		fds_record_open(&p_desc, &rec_flash);
		memcpy(common_infomation.company_identifier_code, rec_flash.p_data, sizeof(common_infomation.company_identifier_code));
	}
}

static void find_device_pti(void)
{
	fds_record_desc_t  p_desc;
	fds_flash_record_t rec_flash;
	ret_code_t         ret;
	
	fds_find_token_t ftok;
	memset(&ftok, 0x00, sizeof(fds_find_token_t));
	memset(common_infomation.device_pti, 0, sizeof(common_infomation.device_pti));
     ret = fds_record_find(BLE_INFO_FILE_ID, BLE_RECORD_KEY_DEVICE_PTI, &p_desc, &ftok);
	if (ret != FDS_SUCCESS)
	{
		device_brand = DB_NWI ;
		memcpy(common_infomation.manufact_name, (char *)BRAND_NAME_NWI, sizeof(BRAND_NAME_NWI));
		RFsystem.SYS_Mode = NOTHING;
		g_ble_device_type = NWITOOL;
		ble_adv_mode = BLE_ADV_MODE_FAST;
		m_app_adv_interval_fast = APP_ADV_INTERVAL_FAST_20_MS;
		m_app_adv_fast_timeout = 0;		/*forever*/
		return;
	}
	else
	{
		fds_record_open(&p_desc, &rec_flash);
		memcpy(common_infomation.device_pti, rec_flash.p_data, sizeof(common_infomation.device_pti));
		switch(common_infomation.device_pti[0])
		{
			case PTI_BRAND_NWI:
			device_brand = DB_NWI ;
			m_adv_uuid0.uuid = CUSTOM_SERVICE_UUID;
			memcpy(common_infomation.manufact_name,BRAND_NAME_NWI, sizeof(BRAND_NAME_NWI));
			break ;
			case PTI_BRAND_PLOTT:
			device_brand = DB_PLOTT ;
			m_adv_uuid0.uuid = CUSTOM_SERVICE_UUID;
			memcpy(common_infomation.manufact_name, BRAND_NAME_PLOTT, sizeof(BRAND_NAME_PLOTT));
			break ;
			case PTI_BRAND_STANLEY:
			device_brand = DB_STANLEY ;
			m_adv_uuid0.uuid = SERVICE_UUID_STANLEY;
			memcpy(common_infomation.manufact_name, BRAND_NAME_STANLEY , sizeof(BRAND_NAME_STANLEY ));			
			break ;
			case PTI_BRAND_DEWALT:
			device_brand = DB_DEWALT ;
			m_adv_uuid0.uuid = SERVICE_UUID_DEWALT;
			memcpy(common_infomation.manufact_name, BRAND_NAME_DEWALT , sizeof(BRAND_NAME_DEWALT ));		
			break ;
			case PTI_NWITOOL:
			device_brand = DB_NWI ;	
			memcpy(common_infomation.manufact_name, (char *)BRAND_NAME_NWI, sizeof(BRAND_NAME_NWI));		
			default:
			device_brand = DB_UNKNOWN ;
			memcpy(common_infomation.manufact_name, (char *)BRAND_NAME_NWI, sizeof(BRAND_NAME_NWI));			
			break ;
		}
		switch((uint16_t)common_infomation.device_pti[1]<<8 | common_infomation.device_pti[2])	
			{
				case PTI_DEVICE_DW080LR:	
				case PTI_DEVICE_DW080LG:
					RFsystem.SYS_Mode = NOTHING;
					g_ble_device_type = SUB;
					ble_adv_mode = BLE_ADV_MODE_SLOW;
				break;
				case PTI_DEVICE_DW0743RS:
				case PTI_DEVICE_DW0743GS:
					RFsystem.SYS_Mode = ALL_OPEN;
					g_ble_device_type = HUB;
					ble_adv_mode = BLE_ADV_MODE_FAST;
					m_app_adv_interval_fast = APP_ADV_INTERVAL_FAST_152_MS;
					m_app_adv_fast_timeout = 0;		/*forever*/ 
				break;
				case PTI_DEVICE_TLM165s:
				case PTI_DEVICE_TLM165si:
				case PTI_DEVICE_TLM330s:
				case PTI_DEVICE_DW0165S_DW03050:
				case PTI_DEVICE_DW0330S_DW03101:
					g_ble_device_type = LDM;
					ble_adv_mode = BLE_ADV_MODE_FAST;
					m_app_adv_interval_fast = APP_ADV_INTERVAL_FAST_400_MS;
					m_app_adv_fast_timeout = 0 ;		/*forever*/
				break;
				case PTI_DEVICE_NWITOOL:
					RFsystem.SYS_Mode = NOTHING;
					g_ble_device_type = NWITOOL;
					ble_adv_mode = BLE_ADV_MODE_FAST;
					m_app_adv_interval_fast = APP_ADV_INTERVAL_FAST_20_MS;
					m_app_adv_fast_timeout = 0;		/*forever*/ 				
				break;
				default: 
					RFsystem.SYS_Mode = NOTHING;
					g_ble_device_type = NWITOOL;
					ble_adv_mode = BLE_ADV_MODE_FAST;
					m_app_adv_interval_fast = APP_ADV_INTERVAL_FAST_20_MS;
					m_app_adv_fast_timeout = 0;		/*forever*/ 				
				break;
			}		
	}
}
static void find_deviceid(void)
{
	fds_record_desc_t  p_desc;
	fds_flash_record_t rec_flash;
	ret_code_t         ret;
	ble_gap_addr_t mac_addr;
	ble_gap_addr_t ble_gap_addr;
	uint32_t err_code;

	fds_find_token_t ftok;
	memset(&ftok, 0x00, sizeof(fds_find_token_t));
     ret = fds_record_find(BLE_INFO_FILE_ID, BLE_RECORD_KEY_DEVICE_ID, &p_desc, &ftok);
	if (ret != FDS_SUCCESS)
	{
		//	addr_test.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC ;
		err_code = sd_ble_gap_addr_get(&mac_addr);
		APP_ERROR_CHECK(err_code);
		memcpy(common_infomation.deviceid, mac_addr.addr, sizeof(common_infomation.deviceid));
		update_user_flash(common_infomation.deviceid, sizeof(common_infomation.deviceid), BLE_INFO_FILE_ID, BLE_RECORD_KEY_DEVICE_ID);
	}
	else
	{
		fds_record_open(&p_desc, &rec_flash);
		memcpy(common_infomation.deviceid, rec_flash.p_data, sizeof(common_infomation.deviceid));
	}
}

static void find_devicesn(void)
{
	fds_record_desc_t  p_desc;
	fds_flash_record_t rec_flash;
	ret_code_t         ret;

	fds_find_token_t ftok;
	memset(&ftok, 0x00, sizeof(fds_find_token_t));
	memset(common_infomation.sn, 0, sizeof(common_infomation.sn));
     ret = fds_record_find(BLE_INFO_FILE_ID, BLE_RECORD_KEY_DEVICE_SN, &p_desc, &ftok);
	if (ret != FDS_SUCCESS)
	{
		memset(common_infomation.sn, 0x20, sizeof(common_infomation.sn));
	}
	else
	{
		fds_record_open(&p_desc, &rec_flash);
		memcpy(common_infomation.sn, rec_flash.p_data, sizeof(common_infomation.sn));
	}
}

static void find_device_modelnumber(void)
{
	fds_record_desc_t  p_desc;
	fds_flash_record_t rec_flash;
	ret_code_t         ret;

	fds_find_token_t ftok;
	memset(&ftok, 0x00, sizeof(fds_find_token_t));
	memset(common_infomation.model, 0, sizeof(common_infomation.model));
     ret = fds_record_find(BLE_INFO_FILE_ID, BLE_RECORD_KEY_DEVICE_MODEL, &p_desc, &ftok);
	if (ret != FDS_SUCCESS)
	{
		memcpy(common_infomation.model, "NWiTool", sizeof("NWiTool"));
	}
	else
	{
		fds_record_open(&p_desc, &rec_flash);
		memcpy(common_infomation.model, rec_flash.p_data, sizeof(common_infomation.model));
	}
}

static void find_ble_manufacturer_byte_11_flag(void)
{
	fds_record_desc_t  p_desc;
	fds_flash_record_t rec_flash;
	ret_code_t         ret;

	fds_find_token_t ftok;
	memset(&ftok, 0x00, sizeof(fds_find_token_t));
	memset(&common_infomation.manufacturer_byte_11_flag, 0, sizeof(common_infomation.manufacturer_byte_11_flag));
     ret = fds_record_find(DEVICE_INFO_FILE_ID, DEVICE_RECORD_KEY_MANUFACTURER_FLAG, &p_desc, &ftok);
	if (ret != FDS_SUCCESS)
	{
		common_infomation.manufacturer_byte_11_flag |= DEVICE_STATUS_CODE_UNCOMMISSIONED;
	}
	else
	{
		fds_record_open(&p_desc, &rec_flash);
		memcpy(&common_infomation.manufacturer_byte_11_flag, rec_flash.p_data, sizeof(common_infomation.manufacturer_byte_11_flag));
	}
}

static void find_main_mcu_fw_version(void)
{
	fds_record_desc_t  p_desc;
	fds_flash_record_t rec_flash;
	ret_code_t         ret;

	fds_find_token_t ftok;
	memset(&ftok, 0x00, sizeof(fds_find_token_t));
	memset(common_infomation.main_mcu_fw_version, 0, sizeof(common_infomation.main_mcu_fw_version));
     ret = fds_record_find(DEVICE_INFO_FILE_ID, DEVICE_RECORD_KEY_MAIN_MCU_FW_VERSION, &p_desc, &ftok);
	if (ret != FDS_SUCCESS)
	{
		memset(common_infomation.main_mcu_fw_version, 0x20, sizeof(common_infomation.main_mcu_fw_version));
	}
	else
	{
		fds_record_open(&p_desc, &rec_flash);
		memcpy(common_infomation.main_mcu_fw_version, rec_flash.p_data, sizeof(common_infomation.main_mcu_fw_version));
	}
}

static void find_device_infomation_record(void)
{
	find_device_name();
	find_company_identifier_code();
	find_deviceid();
	find_devicesn();
	find_device_modelnumber();
	find_ble_manufacturer_byte_11_flag();
	find_main_mcu_fw_version();
	find_device_pti(); //in the end
}

/**@brief Function for application main entry.
 */
int main(void)
{
	
	bool erase_bonds = 1;

	//timer
	timers_init();
#if 1//0
	//
	uart_init();
#else
	//new
	nrf_gpio_cfg_input(TX_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);
#endif
	
#if USER_MEM_EN
	m_mem_block.p_mem = &m_user_mem[0];
	m_mem_block.len = sizeof(m_user_mem) / sizeof(uint8_t);
#endif
	
	//buttons_leds_init(&erase_bonds);
	
#ifdef BOARD_CUSTOM
	//
	nrf_gpio_cfg_output(P_CTRL);
#endif
	if(*((uint32_t*)NRF_UICR_MBR_PARAMS_PAGE_ADDRESS) == 0xFFFFFFFF)
	{		
		flash_word_write((uint32_t*)NRF_UICR_MBR_PARAMS_PAGE_ADDRESS, (uint32_t)NRF_MBR_PARAMS_PAGE_ADDRESS);
	}
	ble_stack_init();
	peer_manager_init(erase_bonds);
	fds_gc();
	find_device_infomation_record(); //must after peer_manager_init and before gap_params_init,advertising_init

	gap_params_init() ;
	advertising_init();
	services_init();
	conn_params_init();
	//system config
	system_config();
	my_ble_dfu_init();
	application_timers_start();
	//my timer
	myTimerInit();
	if(g_ble_device_type != SUB)
	{
		//Start execution.
		advertising_start();
	}
	else
	{
		if(!(common_infomation.manufacturer_byte_11_flag & DEVICE_STATUS_CODE_UNCOMMISSIONED))
		{
			advertising_start();
		}
		saadc_init();
		saadc_sampling_event_init();
		saadc_sampling_event_enable();			
	}
	rtc_config();
#ifdef BOARD_CUSTOM
	//
	nrf_gpio_pin_write(P_CTRL, 1);
#endif
	wdt_config() ;

	// Enter main loop.
	for(;;)
	{
		//	if(NRF_LOG_PROCESS() == false)
		{
			power_manage();
			nrf_drv_wdt_channel_feed(m_channel_id);
			if(read_LRL_power_flag && g_ble_device_type == SUB)
			{
				if(g_LRL_is_power_on)
				{
					if(!nrf_gpio_pin_read(HPP_PLOCK))
					{//detected LRL power down
						g_LRL_is_power_on = 0;
						if(common_infomation.manufacturer_byte_11_flag & DEVICE_STATUS_CODE_UNCOMMISSIONED)
						{
							g_disconnect_ble();
							nrf_delay_ms(20);
							sd_ble_gap_adv_stop();
							g_is_advertising = 0;
							g_is_adv_interval_changed = 0;
						}
						else
						{
							sd_ble_gap_adv_stop();
							nrf_delay_ms(20);
							g_init_and_start_slow_adv();
						}
					}
				}
				else
				{
					if(nrf_gpio_pin_read(HPP_PLOCK))
					{
					}
				}
				read_LRL_power_flag = 0;	
			}
		}
	} 
}

/**
 * @}
 */
