#include "protocol.h"
#include "app_uart.h"
#include "common.h"
#include "boards.h"
#include "ble_customer.h"
#include "fds.h"
#include "ble_dfu.h"
#include "nrf_drv_wdt.h"
#include "nrf_delay.h"
#include "nrf_dfu_settings.h"

extern common_infomation_t common_infomation ;
extern ret_code_t  update_user_flash(uint8_t *p_data, uint16_t length, uint16_t file_id, uint16_t record_key) ;
extern uint8_t g_LRL_is_power_on ;
extern g_device_time_t g_device_time ;
extern int my_atoi(char *ptr, int len) ; 
extern g_ble_device_type_t g_ble_device_type ;
extern uint8_t g_enter_boot_flag ;

packet_t sent_data;
uint8_t is_enter_app_download_mode ;
uint8_t g_is_sub_hub_paired ;

/** @brief Function for filling a page in flash with a value.
 *
 * @param[in] address Address of the first word in the page to be filled.
 * @param[in] value Value to be written to flash.
 */
void flash_word_write(uint32_t * address, uint32_t value)
{
    // Turn on flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    *address = value;

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Turn off flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
}

typedef struct
{
	uint8_t  value[1024];
	uint16_t len;
}stru_receive_buffer_t, *pstru_receive_buffer_t;

static stru_receive_buffer_t temp_buffer = {0, };
static ble_ctlpro_frame_t *ctlpro_inst = 0 ;	//control protocol frame instance.
static ble_ctlpro_frameex_t* ctlpro_instex = 0 ;	//extended length control protocol frame instance.

stru_protocol_inst_t g_protocol_resp;
//const uint8_t c_protocol_version[8] = {"3.5.0"};
//const uint8_t c_sub_dev_firmware_version[20] = {"BTv2.0"};


/******************************************************************************/
void protocol_handler(ble_custom_t *p_custom, uint8_t *p_data, uint16_t length)
{
	
	ret_code_t         ret;
	size_t len_tmp ;
	uint16_t op_code;
	
	uint8_t i;
	
	//store into buffer
	memcpy(&temp_buffer.value[temp_buffer.len], p_data, length);
	temp_buffer.len += length;
	if(temp_buffer.len >= 1024)
	{
		temp_buffer.len = 0 ;
	}
	ctlpro_inst = (ble_ctlpro_frame_t* )(temp_buffer.value);
	len_tmp = ctlpro_inst->datlen ;
	op_code = GET_OPCODE() ;
		
	if(ctlpro_inst->flag & CP_CFLAG_EXLENGTH)
	{
		ctlpro_instex = (ble_ctlpro_frameex_t* )(temp_buffer.value);
		len_tmp = GET_EXLENGTH() ;
		op_code = GET_EXOPCODE() ;
	}
	
	if(is_enter_app_download_mode)	
	{
		if(op_code == CP_OPCODE_EXITAPPDOWNLOAD && len_tmp == 0 &&temp_buffer.len == 4)
		{
			is_enter_app_download_mode = 0 ;
		}		
		
		g_send_data((uint8_t *)temp_buffer.value, temp_buffer.len, USART);
		memset(temp_buffer.value, 0, 1024);
		temp_buffer.len = 0 ;
		return ;		
	}
	
	if(op_code == CP_OPCODE_ENTERAPPDOWNLOAD && len_tmp == 256)
	{
		is_enter_app_download_mode = 1 ;
		g_send_data((uint8_t *)temp_buffer.value, temp_buffer.len, USART);
		memset(temp_buffer.value, 0, 1024);
		temp_buffer.len = 0 ;
		return ;				
	}
	
	switch(op_code)
	{	
		case CP_OPCODE_SETDEVICEID:
			if(len_tmp != 0x06)
			{
				ctlpro_inst->datlen = 0 ;
				return ;
			}
			memcpy(common_infomation.deviceid, ctlpro_inst->dat, sizeof(common_infomation.deviceid)) ;
			ret = update_user_flash(common_infomation.deviceid, sizeof(common_infomation.deviceid), BLE_INFO_FILE_ID, BLE_RECORD_KEY_DEVICE_ID);
			if(ret != FDS_SUCCESS)
			{
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE|CP_ECODE_ERR;	
			}
			else
			{
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE;
			}
			ctlpro_inst->datlen = 0 ; 
			g_send_data((uint8_t *)ctlpro_inst, ctlpro_inst->datlen + ADD_LENGTH , BLUETOOTH_CTL_PTL);
			break ;
		case BCPC_SET_DEVICENAME:
			if(len_tmp != 0x08)
			{
				ctlpro_inst->datlen = 0 ;
				return ;
			}
			memcpy(common_infomation.device_name, ctlpro_inst->dat, sizeof(common_infomation.device_name)) ;
			ret = update_user_flash(common_infomation.device_name, sizeof(common_infomation.device_name), BLE_INFO_FILE_ID, BLE_RECORD_KEY_DEVICE_NAME);
			if(ret != FDS_SUCCESS)
			{
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE|CP_ECODE_ERR;	
			}
			else
			{
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE;
			}
			ctlpro_inst->datlen = 0 ; 
			g_send_data((uint8_t *)ctlpro_inst, ctlpro_inst->datlen + ADD_LENGTH , BLUETOOTH_CTL_PTL);
			break ;
		case BCPC_SET_PTI:
			if(len_tmp != 0x03)
			{
				ctlpro_inst->datlen = 0 ;
				return ;
			}
			memcpy(common_infomation.device_pti, ctlpro_inst->dat, sizeof(common_infomation.device_pti)) ;
			ret = update_user_flash(common_infomation.device_pti, sizeof(common_infomation.device_pti), BLE_INFO_FILE_ID, BLE_RECORD_KEY_DEVICE_PTI);
			if(ret != FDS_SUCCESS)
			{
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE|CP_ECODE_ERR;	
			}
			else
			{
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE;
			}
			ctlpro_inst->datlen = 0 ; 
			g_send_data((uint8_t *)ctlpro_inst, ctlpro_inst->datlen + ADD_LENGTH , BLUETOOTH_CTL_PTL);
			break ;
		case BCPC_SET_CPNCODE:
			if(len_tmp != 0x02)
			{
				ctlpro_inst->datlen = 0 ;
				return ;
			}
			memcpy(common_infomation.company_identifier_code, ctlpro_inst->dat, sizeof(common_infomation.company_identifier_code)) ;
			ret = update_user_flash((unsigned char *)common_infomation.company_identifier_code, sizeof(common_infomation.company_identifier_code), BLE_INFO_FILE_ID, BLE_RECORD_KEY_DEVICE_CIDC);
			if(ret != FDS_SUCCESS)
			{
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE|CP_ECODE_ERR;	
			}
			else
			{
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE;
			}
			ctlpro_inst->datlen = 0 ; 
			g_send_data((uint8_t *)ctlpro_inst, ctlpro_inst->datlen + ADD_LENGTH , BLUETOOTH_CTL_PTL);
			break ;
		case BCPC_SET_MODEL:
			memset(common_infomation.model, 0, sizeof(common_infomation.model)) ;
			memcpy(common_infomation.model, ctlpro_inst->dat, len_tmp) ;
			ret = update_user_flash((unsigned char *)common_infomation.model, sizeof(common_infomation.model), BLE_INFO_FILE_ID, BLE_RECORD_KEY_DEVICE_MODEL);
			if(ret != FDS_SUCCESS)
			{
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE|CP_ECODE_ERR;	
			}
			else
			{
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE;
			}
			ctlpro_inst->datlen = 0 ; 
			g_send_data((uint8_t *)ctlpro_inst, ctlpro_inst->datlen + ADD_LENGTH , BLUETOOTH_CTL_PTL);
			break ;
		case BCPC_SET_SN:
			if(len_tmp != sizeof(common_infomation.sn))
			{
				ctlpro_inst->datlen = 0 ;
				return ;
			}
			memcpy(common_infomation.sn, ctlpro_inst->dat, sizeof(common_infomation.sn)) ;
			ret = update_user_flash((unsigned char *)common_infomation.sn, sizeof(common_infomation.sn), BLE_INFO_FILE_ID, BLE_RECORD_KEY_DEVICE_SN);
			if(ret != FDS_SUCCESS)
			{
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE|CP_ECODE_ERR;	
			}
			else
			{
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE;
			}
			ctlpro_inst->datlen = 0 ; 
			g_send_data((uint8_t *)ctlpro_inst, ctlpro_inst->datlen + ADD_LENGTH , BLUETOOTH_CTL_PTL);
			break ;
		case BCPC_RESET_BLE_INFO:
			if(len_tmp != 7)
			{
				ctlpro_inst->datlen = 0 ;
				return ;
			}
			if(0!=strcmp(ctlpro_inst->dat, "NWiTool"))
			{
				ctlpro_inst->datlen = 0 ;
				return ;
			}
			ret = fds_file_delete(BLE_INFO_FILE_ID);
			
			if(ret != FDS_SUCCESS)
			{
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE|CP_ECODE_ERR;	
			}
			else
			{
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE;
			}
			ctlpro_inst->datlen = 0 ; 
			g_send_data((uint8_t *)ctlpro_inst, ctlpro_inst->datlen + ADD_LENGTH , BLUETOOTH_CTL_PTL);
			break ;				
		case SYNCHRONIZE_TIME:
			if(len_tmp != 14)
			{
				ctlpro_inst->datlen = 0 ;
				return ;
			}
			g_device_time.year =  my_atoi(ctlpro_inst->dat, 4) ;
			g_device_time.month = (month_calendar_t)my_atoi(&ctlpro_inst->dat[4], 2) ;
			g_device_time.day = my_atoi(&ctlpro_inst->dat[6], 2) ;
			g_device_time.hour = my_atoi(&ctlpro_inst->dat[8], 2) ;
			g_device_time.minute = my_atoi(&ctlpro_inst->dat[10], 2) ;
			g_device_time.second = my_atoi(&ctlpro_inst->dat[12], 2) ;
			ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE;
			ctlpro_inst->datlen = 0 ; 
			g_send_data((uint8_t *)ctlpro_inst, ctlpro_inst->datlen + ADD_LENGTH , BLUETOOTH_CTL_PTL);
			break ;
			case CP_OPCODE_ENTERDFU:
			if(len_tmp != 0x00)
			{
				ctlpro_inst->datlen = 0 ;
				return ;
			}
			if(g_ble_device_type == SUB)
			{
				if(!(common_infomation.manufacturer_byte_11_flag & DEVICE_STATUS_CODE_UNCOMMISSIONED))
				{			
					g_enter_boot_flag =	1;
					my_enter_bootloader();
				}
				else
				{
					ctlpro_inst->flag |= CP_ECODE_ERR;
				}
			}
			else
			{	
				g_enter_boot_flag =	1;
				my_enter_bootloader();
			}
			ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE;
			ctlpro_inst->datlen = 0 ; 
			g_send_data((uint8_t *)ctlpro_inst, ctlpro_inst->datlen + ADD_LENGTH , BLUETOOTH_CTL_PTL);
			nrf_delay_ms(200);
			g_disconnect_ble();
			break ;			
			default:
			if(RFsystem.SYS_Mode != BT_OPEN && RFsystem.SYS_Mode != ALL_OPEN)
			{	
				g_send_data((uint8_t *)temp_buffer.value, temp_buffer.len, USART);
				memset(temp_buffer.value, 0, 1024);
				temp_buffer.len = 0 ;
			}
			break ;
	}
			
	
	if(RFsystem.SYS_Mode == BT_OPEN)
	{//send to LRL by uart
		switch(op_code)
		{
			case POWER_ON_OFF_LRL_DEVICE:
			{			
				if(len_tmp != 1)
				{
					ctlpro_inst->datlen = 0 ;
					return ;
				}
				
				if(ctlpro_inst->dat[0] == 0x01)
				{//power on
					if(!g_LRL_is_power_on)
					{
						nrf_gpio_cfg_output(HPP_PLOCK);
						nrf_gpio_pin_set(HPP_PLOCK);
					}
					else
					{
						ctlpro_inst->flag |= CP_ECODE_ERR;	
					}
				}
				else
				{//power off
					if(g_LRL_is_power_on)
					{
						g_send_data(temp_buffer.value, temp_buffer.len, USART);
					}
					else
					{
						ctlpro_inst->flag |= CP_ECODE_ERR;	
					}
				}
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE;
				ctlpro_inst->datlen = 0 ; 
				g_send_data((uint8_t *)ctlpro_inst, ctlpro_inst->datlen + ADD_LENGTH , BLUETOOTH_CTL_PTL);
			}
			break;
			default:
			{
				g_send_data(temp_buffer.value, temp_buffer.len, USART);
				memset(temp_buffer.value, 0, 1024);
				temp_buffer.len = 0 ;
			}
			break;
		}
	}
	else if(RFsystem.SYS_Mode == ALL_OPEN)
	{//hub mode
		switch(op_code)
		{
			case GET_HUB_STATUS:
			{					
				uint8_t dev_count, temp;
				if(len_tmp != 0)
				{
					ctlpro_inst->datlen = 0 ;
					return ;
				}				
				ctlpro_inst->datlen = 1 ; 
				temp = RFsystem.device_count;
				dev_count = 0;
				for(i = 0; i < 4; i++)
				{
					if(temp & 0x01)
					{
						dev_count++;
					}
					temp >>= 1;
				}
				ctlpro_inst->dat[0] = dev_count;
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE;
				g_send_data((uint8_t *)ctlpro_inst, ctlpro_inst->datlen + ADD_LENGTH , BLUETOOTH_CTL_PTL);
			}
			break;
			case GET_LRL_NAME_AND_DEVICE_ID:
			{
				if(len_tmp != 1)
				{
					ctlpro_inst->datlen = 0 ;
					return ;
				}
				uint8_t chnl = ctlpro_inst->dat[0];  //channel idx the APP wants, 0x01~0x08
				ctlpro_inst->datlen = 0x0E;  //14
				if(RFsystem.device_count & (0x01 << (chnl-1)))
				{
					//local name and device id...
					//local name is in adv data
					//device id, reqst from sub 
					memcpy(ctlpro_inst->dat, RFsystem.device[chnl-1], ctlpro_inst->datlen);
				}
				else
				{//no device!
					ctlpro_inst->flag |= B_Err_DATA;
					memset(ctlpro_inst->dat, 0, ctlpro_inst->datlen);
				}
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE;
				g_send_data((uint8_t *)ctlpro_inst, ctlpro_inst->datlen + ADD_LENGTH , BLUETOOTH_CTL_PTL);
			}
			break;
			case ESTABLISH_OR_DISCONNECT_HUB_CHANNEL_TO_LRL:
			{
				if(len_tmp != 1)
				{
					ctlpro_inst->datlen = 0 ;
					return ;
				}
				uint8_t chnl = ctlpro_inst->dat[0];		
				if(chnl == 0)
				{//disconnect all HUB channels
					memset(RFsystem.device, 0, 20 * 4);
					RFsystem.device_count = 0;
					g_rf_chnl_nbr = 3;
					RFsystem.ch_scan_time = RF_SCAN_PERIOD;
					g_is_rf_connected = 0;
				}
				else
				{//0x01~0x08, select different HUB channels
					RFsystem.ch_scan_time = 0xFF;
					if(RFsystem.device_count & (0x01 << (chnl-1)))
					{
						RF_channel_set(chnl);
						g_is_rf_connected = 1;
						if(0 == memcmp(common_infomation.deviceid_pair, &RFsystem.device[chnl-1][8], sizeof(common_infomation.deviceid_pair)))
						{
							g_is_sub_hub_paired = 0;
						}
					}
					else
					{//no device!
						ctlpro_inst->flag |= B_Err_DATA;
						RFsystem.ch_scan_time = RF_SCAN_PERIOD;
						g_is_rf_connected = 0;
					}
				}
				ctlpro_inst->datlen	= 2;	
				memset(ctlpro_inst->dat, 0, ctlpro_inst->datlen);
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE;
				g_send_data((uint8_t *)ctlpro_inst, ctlpro_inst->datlen + ADD_LENGTH , BLUETOOTH_CTL_PTL);
			}
			break;
			case PAIR_LRL_TO_HUB:
			{
				if(len_tmp != 6)
				{
					ctlpro_inst->datlen = 0 ;
					return ;
				}
				g_is_sub_hub_paired = 1;
				memcpy(common_infomation.deviceid_pair, ctlpro_inst->dat, sizeof(common_infomation.deviceid_pair)) ;
				ret = update_user_flash(common_infomation.deviceid_pair, sizeof(common_infomation.deviceid_pair), DEVICE_INFO_FILE_ID, DEVICE_RECORD_KEY_DEVICE_ID_PAIR);
				if(ret != FDS_SUCCESS)
				{
					ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE|CP_ECODE_ERR;	
				}
				else
				{
					ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE;
				}				
				ctlpro_inst->datlen	= 2;	
				memset(ctlpro_inst->dat, 0, ctlpro_inst->datlen);
				g_send_data((uint8_t *)ctlpro_inst, ctlpro_inst->datlen + ADD_LENGTH , BLUETOOTH_CTL_PTL);
			}
			break;
			case UNPAIR_LRL_FROM_HUB:
			{
				if(len_tmp != 0)
				{
					ctlpro_inst->datlen = 0 ;
					return ;
				}	
				g_is_sub_hub_paired = 0;				
				ctlpro_inst->datlen	= 2;	
				memset(ctlpro_inst->dat, 0, ctlpro_inst->datlen);
				ctlpro_inst->flag |= DEFAULT_RESP_RESULT_CODE;
				g_send_data((uint8_t *)ctlpro_inst, ctlpro_inst->datlen + ADD_LENGTH , BLUETOOTH_CTL_PTL);
			}
			break;
			default:
			{
				g_send_data(temp_buffer.value, temp_buffer.len, USART);
				//send by rf
				if(g_is_rf_connected)
				{
					g_send_data(temp_buffer.value, temp_buffer.len, WIRELESS);
				}				
			}
			break;
		}

		//test, re_count=0. !!!!!!
		g_rf_buffer.Re_count = 0;//4;
	}	
	memset(&temp_buffer, 0, sizeof(stru_receive_buffer_t));
}
/******************************************************************************/


