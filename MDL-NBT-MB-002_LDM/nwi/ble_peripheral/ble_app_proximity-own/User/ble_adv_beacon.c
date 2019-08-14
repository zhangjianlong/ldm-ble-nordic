#include "ble_adv_beacon.h"
#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "common.h"

#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(152, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

extern common_infomation_t common_infomation ;
extern ble_uuid_t m_adv_uuid0;
extern ble_advdata_manuf_data_t m_adv_manuf_data;
static ble_gap_adv_params_t m_adv_params;                                 /**< Parameters to be passed to the stack when starting advertising. */

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
void g_advertising_init_beacon(void)
{
	uint32_t      err_code;
	ble_advdata_t advdata;
	uint8_t manuf_data_u8[10] ;

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

	err_code = ble_advdata_set(&advdata, NULL);
	APP_ERROR_CHECK(err_code);

	// Initialize advertising parameters (used when starting advertising).
	memset(&m_adv_params, 0, sizeof(m_adv_params));

	m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
	m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
	m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
	m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
	m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}


/**@brief Function for starting advertising.
 */
void g_advertising_start_beacon(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
}
