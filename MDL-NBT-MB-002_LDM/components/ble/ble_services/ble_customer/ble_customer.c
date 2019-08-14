#include "sdk_common.h"
//#if NRF_MODULE_ENABLED(BLE_BAS)
#include "ble_customer.h"
#include <stdio.h>
#include <string.h>
#include "ble_srv_common.h"
#include "common.h"

extern g_ble_device_type_t g_ble_device_type;
//#ifndef NRF_LOG_MODULE_NAME
//#define NRF_LOG_MODULE_NAME "UART"
//#endif
//#include "nrf_log.h"

#define BLE_UUID_CUSTOM_CTRL_PROT_CHARACTERISTIC           0xFFF0
#define BLE_UUID_CUSTOM_BLE_SPEC_VER_CHARACTERISTIC        0xFFF1
#define BLE_UUID_CUSTOM_NOTI_CHAN_CHARACTERISTIC           0xFFF2
#define BLE_UUID_CUSTOM_ANGLE_NOTI_CHAN_CHARACTERISTIC     0xFFF3
#define BLE_UUID_CUSTOM_LRL_SETTINGS_NOTI_CHAN_CHARACTERISTIC     0xFFF4

#if USER_MEM_EN
#define BLE_CUSTOM_MAX_CTRL_PROT_CHAR_LEN     512//500//2049
uint8_t g_long_char_buf[BLE_CUSTOM_MAX_CTRL_PROT_CHAR_LEN];
#else
#define BLE_CUSTOM_MAX_CTRL_PROT_CHAR_LEN     20
#endif

#define BLE_CUSTOM_MAX_BLE_SPEC_VER_CHAR_LEN  20
#define BLE_CUSTOM_MAX_NOTI_CHAN_CHAR_LEN     20


uint8_t char_1_desc[] = {"Control Protocol\0"};
uint8_t char_1_value = 0;
uint8_t char_2_desc[] = {"BLE Spec Version\0"};
uint8_t char_2_value[] = {"1.7.1\0"};
uint8_t char_3_desc[] = {"Common Notification\0"};
uint8_t char_3_value = 0 ;
uint8_t char_4_desc[] = {"Angle Notification\0"};
uint8_t char_4_value = 0;
uint8_t char_5_desc[] = {"LRL current settings\0"};
uint8_t char_5_value = 0 ;


//uint16_t g_custom_server_handle;
//ble_gatts_char_handles_t custom_char_1_handle;
//ble_gatts_char_handles_t custom_char_2_handle;
//ble_gatts_char_handles_t custom_char_3_handle;


/*0xFFF0, NWi BLE Control Protocol, write and notify*/
static uint32_t ctrl_prot_char_add(ble_custom_t *p_custom, ble_custom_init_t *p_custom_init)
{
	uint32_t            err_code;
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	
	memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	
	memset(&char_md, 0, sizeof(char_md));
//	char_md.char_props.read = 1;
	char_md.char_props.write = 1;
	char_md.char_props.notify = 1;
	char_md.char_user_desc_max_size = sizeof(char_1_desc);
	char_md.char_user_desc_size = sizeof(char_1_desc);
	char_md.p_char_user_desc = char_1_desc;//char_1_desc;
	char_md.p_char_pf = NULL;
	char_md.p_user_desc_md = NULL;
	char_md.p_cccd_md = &cccd_md;
	char_md.p_sccd_md = NULL;
	
	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

 	attr_md.vloc = BLE_GATTS_VLOC_STACK;  /*BLE_GATTS_VLOC_USER*/
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen = 1;
	
	ble_uuid.type = BLE_UUID_TYPE_BLE;
	ble_uuid.uuid = BLE_UUID_CUSTOM_CTRL_PROT_CHARACTERISTIC;  //0xFFF0;
	
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	attr_char_value.p_uuid = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len = 1;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len = BLE_CUSTOM_MAX_CTRL_PROT_CHAR_LEN;
	attr_char_value.p_value = &char_1_value;
	
	err_code = sd_ble_gatts_characteristic_add(p_custom->service_handle, &char_md, \
	                                           &attr_char_value, &p_custom->ctrl_prot_handles);
	return err_code;
}

/*0xFFF1, BLE Spec Version, read only*/
static uint32_t ble_spec_ver_char_add(ble_custom_t *p_custom, ble_custom_init_t *p_custom_init)
{
	uint32_t            err_code;
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read = 1;
//	char_md.char_props.write = 1;
	char_md.char_user_desc_max_size = sizeof(char_2_desc) ;
	char_md.char_user_desc_size = sizeof(char_2_desc) ;
	char_md.p_char_user_desc = char_2_desc;
	char_md.p_char_pf = NULL;
	char_md.p_user_desc_md = NULL;
	char_md.p_cccd_md = NULL;
	char_md.p_sccd_md = NULL;
	
	memset(&attr_md, 0, sizeof(attr_md));
	//
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	
	attr_md.vloc = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen = 1;//0;
	
	ble_uuid.type = BLE_UUID_TYPE_BLE;
	ble_uuid.uuid = BLE_UUID_CUSTOM_BLE_SPEC_VER_CHARACTERISTIC;  //0xFFF1;
	
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	attr_char_value.p_uuid = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len = sizeof(char_2_value);//sizeof(uint8_t) * 1;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len = BLE_CUSTOM_MAX_BLE_SPEC_VER_CHAR_LEN;
	attr_char_value.p_value = char_2_value;//char_2_value;
	err_code = sd_ble_gatts_characteristic_add(p_custom->service_handle, &char_md, \
	                                           &attr_char_value, &p_custom->ble_spec_ver_handles);
	return err_code;
}

/*0xFFF2, Notification Channel, notify and read*/
static uint32_t noti_chan_char_add(ble_custom_t *p_custom, ble_custom_init_t *p_custom_init)
{
	uint32_t            err_code;
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	
	memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);  //this line cannot be marked!!

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read = 1;
	char_md.char_props.notify = 1;
	char_md.char_user_desc_max_size = sizeof(char_3_desc) ;
	char_md.char_user_desc_size = sizeof(char_3_desc) ;
	char_md.p_char_user_desc = char_3_desc;
	char_md.p_char_pf = NULL;
	char_md.p_user_desc_md = NULL;
	char_md.p_cccd_md = &cccd_md;
	char_md.p_sccd_md = NULL;
	
	memset(&attr_md, 0, sizeof(attr_md));
	//
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
	
	attr_md.vloc = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen = 1;//0;
	
	ble_uuid.type = BLE_UUID_TYPE_BLE;
	ble_uuid.uuid = BLE_UUID_CUSTOM_NOTI_CHAN_CHARACTERISTIC;  //0xFFF2;
	
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	attr_char_value.p_uuid = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len = 1;  //1;//sizeof(uint8_t) * 1;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len = BLE_CUSTOM_MAX_NOTI_CHAN_CHAR_LEN;//4;//strlen(char_4_vals);//sizeof(uint8_t) * 1;
	attr_char_value.p_value = &char_3_value;
	err_code = sd_ble_gatts_characteristic_add(p_custom->service_handle, &char_md, \
	                                           &attr_char_value, &p_custom->noti_chan_handles);
	return err_code;
}

/*0xFFF3, angle Notification Channel, notify and read*/
static uint32_t angle_noti_chan_char_add(ble_custom_t *p_custom, ble_custom_init_t *p_custom_init)
{
	uint32_t            err_code;
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	
	memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);  //this line cannot be marked!!

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read = 1;
	char_md.char_props.notify = 1;
	char_md.char_user_desc_max_size = sizeof(char_4_desc) ;
	char_md.char_user_desc_size = sizeof(char_4_desc) ;
	char_md.p_char_user_desc = char_4_desc;
	char_md.p_char_pf = NULL;
	char_md.p_user_desc_md = NULL;
	char_md.p_cccd_md = &cccd_md;
	char_md.p_sccd_md = NULL;
	
	memset(&attr_md, 0, sizeof(attr_md));
	//
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
	
	attr_md.vloc = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen = 1;//0;
	
	ble_uuid.type = BLE_UUID_TYPE_BLE;
	ble_uuid.uuid = BLE_UUID_CUSTOM_ANGLE_NOTI_CHAN_CHARACTERISTIC;  //0xFFF3;
	
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	attr_char_value.p_uuid = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len = 1;  //1;//sizeof(uint8_t) * 1;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len = BLE_CUSTOM_MAX_NOTI_CHAN_CHAR_LEN;//4;//strlen(char_4_vals);//sizeof(uint8_t) * 1;
	attr_char_value.p_value = &char_4_value;//char_4_value;
	err_code = sd_ble_gatts_characteristic_add(p_custom->service_handle, &char_md, \
	                                           &attr_char_value, &p_custom->noti_angle_handles);
	return err_code;
}

/*0xFFF4, LRL current settings Notification Channel, notify and read*/
static uint32_t LRL_current_settings_noti_chan_char_add(ble_custom_t *p_custom, ble_custom_init_t *p_custom_init)
{
	uint32_t            err_code;
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	
	memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);  //this line cannot be marked!!

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read = 1;
	char_md.char_props.notify = 1;
	char_md.char_user_desc_max_size = sizeof(char_5_desc) ;
	char_md.char_user_desc_size = sizeof(char_5_desc) ;
	char_md.p_char_user_desc = char_5_desc;
	char_md.p_char_pf = NULL;
	char_md.p_user_desc_md = NULL;
	char_md.p_cccd_md = &cccd_md;
	char_md.p_sccd_md = NULL;
	
	memset(&attr_md, 0, sizeof(attr_md));
	//
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
	
	attr_md.vloc = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen = 1;//0;
	
	ble_uuid.type = BLE_UUID_TYPE_BLE;
	ble_uuid.uuid = BLE_UUID_CUSTOM_LRL_SETTINGS_NOTI_CHAN_CHARACTERISTIC;  //0xFFF4;
	
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	attr_char_value.p_uuid = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len = 1;  //1;//sizeof(uint8_t) * 1;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len = BLE_CUSTOM_MAX_NOTI_CHAN_CHAR_LEN;
	attr_char_value.p_value = &char_5_value;//char_5_value;
	err_code = sd_ble_gatts_characteristic_add(p_custom->service_handle, &char_md, \
	                                           &attr_char_value, &p_custom->noti_angle_handles);
	return err_code;
}

static void on_connect(ble_custom_t *p_custom, ble_evt_t *p_ble_evt)
{
	p_custom->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
//	printf("on_connect!\r\n");
	//NRF_LOG_DEBUG("on_connect custom_service!\r\n");
}

static void on_disconnect(ble_custom_t *p_custom, ble_evt_t *p_ble_evt)
{
	UNUSED_PARAMETER(p_ble_evt);
    p_custom->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_write(ble_custom_t *p_custom, ble_evt_t *p_ble_evt)
{
	uint32_t err_code;
	
	ble_gatts_evt_write_t *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	
	if((p_evt_write->handle == p_custom->ctrl_prot_handles.cccd_handle) \
		&& (p_evt_write->len == 2))
	{
		if(ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_custom->is_ctrl_prot_notification_enabled = true;
        }
        else
        {
            p_custom->is_ctrl_prot_notification_enabled = false;
        }
	}/*notification state, enable or disable?*/
	else if((p_evt_write->handle == p_custom->noti_chan_handles.cccd_handle) \
		&& (p_evt_write->len == 2))
	{
		if(ble_srv_is_notification_enabled(p_evt_write->data))
		{
			p_custom->is_noti_chan_notification_enabled = true;
		}
		else
		{
			p_custom->is_noti_chan_notification_enabled = false;
		}
	}/*notification state, enable or disable?*/
	else if((p_evt_write->handle == p_custom->noti_angle_handles.cccd_handle) \
		&& (p_evt_write->len == 2))
	{
		if(ble_srv_is_notification_enabled(p_evt_write->data))
		{
			p_custom->is_noti_angle_notification_enabled = true;
		}
		else
		{
			p_custom->is_noti_angle_notification_enabled = false;
		}
	}/*notification state, enable or disable?*/
	else if((p_evt_write->handle == p_custom->noti_LRL_settings_handles.cccd_handle) \
		&& (p_evt_write->len == 2))
	{
		if(ble_srv_is_notification_enabled(p_evt_write->data))
		{
			p_custom->is_noti_LRL_settings_notification_enabled = true;
		}
		else
		{
			p_custom->is_noti_LRL_settings_notification_enabled = false;
		}
	}/*notification state, enable or disable?*/		
	else if((p_evt_write->handle == p_custom->ctrl_prot_handles.value_handle) \
		&& (p_custom->data_handler != NULL))
	{
		p_custom->data_handler(p_custom, p_evt_write->data, p_evt_write->len);
	}
#if USER_MEM_EN
	else if((*((uint16_t *)m_mem_block.p_mem) == p_custom->ctrl_prot_handles.value_handle) \
		&& (p_custom->data_handler != NULL))
	{
		//
		//g_long_char_buf
		static ble_gatts_value_t gatt_val;
		gatt_val.offset = 0;
		gatt_val.len = m_mem_block.len;
		gatt_val.p_value = g_long_char_buf;
		err_code = sd_ble_gatts_value_get(p_custom->conn_handle, p_custom->ctrl_prot_handles.value_handle, &gatt_val);
		p_custom->data_handler(p_custom, gatt_val.p_value, gatt_val.len);
	}
#endif
}

void ble_custom_on_ble_evt(ble_custom_t *p_custom, ble_evt_t *p_ble_evt)
{
	uint32_t err_code;
	
	if((p_custom == NULL) || (p_ble_evt == NULL))
	{
		return;
	}
	
	switch(p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			on_connect(p_custom, p_ble_evt);
		break;
		
		case BLE_GAP_EVT_DISCONNECTED:
			on_disconnect(p_custom, p_ble_evt);
		break;
		
		case BLE_GATTS_EVT_WRITE:
			on_write(p_custom, p_ble_evt);
		break;
#if USER_MEM_EN
		case BLE_EVT_USER_MEM_REQUEST:
			err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, &m_mem_block);
//			if(err_code != NRF_SUCCESS)
//			{
//				printf("ERROR sd_ble_user_mem_reply: %u\r\n", (unsigned int)err_code);
//			}
//			else
//			{
//				printf("USER_MEM_REQUEST OK\r\n");
//			}
        break;  //BLE_EVT_USER_MEM_REQUESTs
#endif	
		default:
			//
		break;
	}
}

uint32_t ble_custom_init(ble_custom_t *p_custom, ble_custom_init_t *p_custom_init)
{
	uint32_t   err_code;
	ble_uuid_t ble_uuid;
	
	p_custom->conn_handle = BLE_CONN_HANDLE_INVALID;
	p_custom->data_handler = p_custom_init->data_handler;
//	p_custom->is_notification_enabled = false;
	p_custom->is_ctrl_prot_notification_enabled = false;
	p_custom->is_noti_chan_notification_enabled = false;
	
	//p_custom->uuid_type = BLE_UUID_TYPE_BLE;
	ble_uuid.type = p_custom->uuid_type;
	ble_uuid.uuid = p_custom->service_uuid; //0xFACE; or 0xCAFE
	
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, \
	                                    &p_custom->service_handle);
	if(err_code != NRF_SUCCESS)
	{return err_code;}
	
	err_code = ctrl_prot_char_add(p_custom, p_custom_init);
	if(err_code != NRF_SUCCESS)
	{return err_code;}
	
	err_code = ble_spec_ver_char_add(p_custom, p_custom_init);
	if(err_code != NRF_SUCCESS)
	{return err_code;}
	
	err_code = noti_chan_char_add(p_custom, p_custom_init);
	if(err_code != NRF_SUCCESS)
	{return err_code;}
	
	if(g_ble_device_type == LDM)
	{
		err_code = angle_noti_chan_char_add(p_custom, p_custom_init);
		if(err_code != NRF_SUCCESS)
		{return err_code;}
	}
	else	if(g_ble_device_type == SUB || g_ble_device_type == HUB)
	{
		err_code = LRL_current_settings_noti_chan_char_add(p_custom, p_custom_init);
		if(err_code != NRF_SUCCESS)
		{return err_code;}
	}
	return NRF_SUCCESS;
}

uint32_t ble_custom_string_send(ble_custom_t *p_custom, uint8_t *p_string, uint16_t length, enum_noti_char_t noti_char)
{
	bool is_notification_enabled;
	ble_gatts_hvx_params_t hvx_params;
	uint16_t max_char_len;

     VERIFY_PARAM_NOT_NULL(p_custom);
	
	if(noti_char == NOTIFICATION_CTRL_PROT)
	{
		is_notification_enabled = p_custom->is_ctrl_prot_notification_enabled;
		max_char_len = BLE_CUSTOM_MAX_CTRL_PROT_CHAR_LEN;
	}
	else if(noti_char == NOTIFICATION_NOTI_CHAN)
	{
		is_notification_enabled = p_custom->is_noti_chan_notification_enabled;
		max_char_len = BLE_CUSTOM_MAX_NOTI_CHAN_CHAR_LEN;
	}
	else if(noti_char == NOTIFICATION_ANGLE_CHAN)
	{
		is_notification_enabled = p_custom->is_noti_angle_notification_enabled;
		max_char_len = BLE_CUSTOM_MAX_NOTI_CHAN_CHAR_LEN;
	}
	else if(noti_char == NOTIFICATION_LRL_SETTINGS_CHAN)
	{
		is_notification_enabled = p_custom->is_noti_LRL_settings_notification_enabled;
		max_char_len = BLE_CUSTOM_MAX_NOTI_CHAN_CHAR_LEN;
	}	
	if((p_custom->conn_handle == BLE_CONN_HANDLE_INVALID) \
		|| (!is_notification_enabled))
	{
		return NRF_ERROR_INVALID_STATE;
	}
	if(length > max_char_len)
     {
	   return NRF_ERROR_INVALID_PARAM;
     }

	memset(&hvx_params, 0, sizeof(hvx_params));

	if(noti_char == NOTIFICATION_CTRL_PROT)
	{hvx_params.handle = p_custom->ctrl_prot_handles.value_handle;}
	else if(noti_char == NOTIFICATION_NOTI_CHAN)
	{hvx_params.handle = p_custom->noti_chan_handles.value_handle;}
	else if(noti_char == NOTIFICATION_ANGLE_CHAN)
	{hvx_params.handle = p_custom->noti_angle_handles.value_handle;}
	else if(noti_char == NOTIFICATION_LRL_SETTINGS_CHAN)
	{hvx_params.handle = p_custom->noti_LRL_settings_handles.value_handle;}	
	
	hvx_params.p_data = p_string;
	hvx_params.p_len  = &length;
	hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

	return sd_ble_gatts_hvx(p_custom->conn_handle, &hvx_params);
}
