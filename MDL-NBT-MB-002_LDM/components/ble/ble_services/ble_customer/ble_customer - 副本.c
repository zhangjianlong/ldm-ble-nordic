#include "sdk_common.h"
//#if NRF_MODULE_ENABLED(BLE_BAS)
#include "ble_customer.h"
#include <stdio.h>
#include <string.h>
#include "ble_srv_common.h"

#ifndef NRF_LOG_MODULE_NAME
#define NRF_LOG_MODULE_NAME "UART"
#endif
#include "nrf_log.h"

uint8_t char_1_desc[] = {"Character1"};
uint8_t char_1_value = 0x38;

uint16_t g_customer_server_handle;
ble_gatts_char_handles_t customer_char_1_handle;

static uint32_t customer_char_add()
{
	uint32_t            err_code;
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	
	char_md.char_props.read = 1;
	char_md.char_props.write = 1;
	char_md.char_user_desc_max_size = 16;
	char_md.char_user_desc_size = sizeof("Character1");
	char_md.p_char_user_desc = char_1_desc;
	char_md.p_char_pf = NULL;
	char_md.p_cccd_md = NULL;
	char_md.p_sccd_md = NULL;
	char_md.p_user_desc_md = NULL;
	
	memset(&attr_md, 0, sizeof(attr_md));
	attr_md.read_perm.lv = 1;
	attr_md.read_perm.sm = 1;
	attr_md.write_perm.lv = 1;
	attr_md.write_perm.sm = 1;
	attr_md.vloc = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen = 0;
	
	ble_uuid.type = BLE_UUID_TYPE_BLE;
	ble_uuid.uuid = 0xFF01;
	
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	attr_char_value.p_uuid = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len = sizeof(uint8_t) * 1;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len = sizeof(uint8_t) * 1;
	attr_char_value.p_value = &char_1_value;
	
	err_code = sd_ble_gatts_characteristic_add(g_customer_server_handle, &char_md, \
	                                           &attr_char_value, \
											   &customer_char_1_handle);
	if(err_code != NRF_SUCCESS)
	{return err_code;}
	return NRF_SUCCESS;
}



static void on_connect(/*ble_lbs_t * p_lbs, */ble_evt_t * p_ble_evt)
{
//	p_lbs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
//	printf("on_connect!\r\n");
	NRF_LOG_DEBUG("on_connect customer_service!\r\n");
}

void ble_customer_on_ble_evt(ble_evt_t* p_ble_evt)
{
	switch(p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			on_connect(p_ble_evt);
		break;
		case BLE_GAP_EVT_DISCONNECTED:
			//
		break;
		case BLE_GATTS_EVT_WRITE:
			//
		break;
		default:
			//
		break;
	}
}

uint32_t ble_customer_init(void)
{
	uint32_t   err_code;
	ble_uuid_t ble_uuid;
	
	ble_uuid.type = BLE_UUID_TYPE_BLE;
	ble_uuid.uuid = 0xFFA2;
	
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &g_customer_server_handle);
	if(err_code != NRF_SUCCESS)
	{return err_code;}
	return customer_char_add();
}
