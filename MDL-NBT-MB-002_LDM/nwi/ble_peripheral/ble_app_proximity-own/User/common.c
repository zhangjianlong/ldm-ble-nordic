#include <string.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_dfu_types.h"
#include "boards.h"
#include "nrf_drv_spi.h"
#include "system_nrf52.h"
#include "app_uart.h"
#include "app_scheduler.h"
#include "common.h"
#include "si4432.h"
#include "protocol.h"
#include "fds.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_customer.h"


stru_rf_data_t g_rf_buffer;
SYSTEset RFsystem;

stru_usart_data_t g_uart_data;
extern g_ble_device_type_t g_ble_device_type;
extern common_infomation_t common_infomation;
extern uint32_t g_adv_fast_timeout_cnt;
extern ble_adv_mode_t ble_adv_mode;
extern uint8_t g_LRL_is_power_on;
extern uint8_t g_batt_level ;
extern g_device_time_t g_device_time ;
extern uint8_t g_coincell_level ;
extern uint8_t g_is_adv_interval_changed;
extern uint8_t g_is_sub_hub_paired ;
extern ble_bas_t  m_bas;  /**< Structure used to identify the battery service. */

unsigned char offset_words[2][4][14] = {{"{Haveany915M0}", "{Haveany915M1}", "{Haveany915M2}", "{Haveany915M3}"}, \
										{"[Ihave915M0]", "[Ihave915M1]", "[Ihave915M2]", "[Ihave915M3]"}};

//extern unsigned char g_rf_chnl_nbr;

extern void power_manage(void);
extern void g_init_and_start_fast_adv(void);
extern void g_init_and_start_normal_adv(void);										
extern void g_init_and_start_slow_adv(void);										

unsigned int hopping_time;
										
ret_code_t  update_user_flash(uint8_t *p_data, uint16_t length, uint16_t file_id, uint16_t record_key)
{
	ret_code_t         ret;
	fds_record_t       rec;
	fds_record_desc_t  rec_desc;
	fds_record_chunk_t rec_chunk;
	fds_record_desc_t  p_desc;
	fds_find_token_t ftok;
	
	// Prepare chunk.
	rec_chunk.p_data       =  p_data ;
	rec_chunk.length_words =  length%4 > 0 ? length/4+1: length/4;

	// Prepare the record to be stored in flash.
	rec.file_id         = file_id ;
	rec.key             = record_key ;
	rec.data.p_chunks   = &rec_chunk;
	rec.data.num_chunks = 1;
	memset(&ftok, 0x00, sizeof(fds_find_token_t));
	ret = fds_record_find(file_id, record_key, &p_desc, &ftok);
	if (ret == FDS_ERR_NOT_FOUND)
	{
		ret = fds_record_write(&rec_desc, &rec);
	}
	else// Update existing record.
	{
		ret = fds_record_update(&p_desc, &rec);
	}
	
	if (ret == FDS_ERR_NO_SPACE_IN_FLASH)
	{
		fds_gc();
	}
	
	return ret;
}										

void wireless_receive_handler(void)
{
	stru_protocol_inst_t *p_prot = 0;

	if(g_rf_buffer.recv_count < ADD_LENGTH)
	{//min data packet is 4Bytes
		return;
	}
	
	RFsystem.event_LED = once_flash;
	
//	if(memcmp(g_rf_buffer.recv_data+1, "reset", 5) == 0)
//	{
//		//reset system!!!
//		NVIC_SystemReset();
//	}
//	else
	{
		//check data...
		if(RFsystem.SYS_Mode == ALL_OPEN)
		{
			if(memcmp(g_rf_buffer.recv_data, offset_words[0][0], 11) == 0)
			{//reqst freq
				return;
			}
			
			memcpy(g_rf_buffer.buffer_data, g_rf_buffer.recv_data, g_rf_buffer.recv_count);
			
			g_rf_buffer.Re_count = 0;  //recv ack, turn off repeat!
		}
		else if(RFsystem.SYS_Mode == RF_OPEN)
		{//sub mode
			if(memcmp(g_rf_buffer.recv_data, offset_words[0][g_rf_chnl_nbr], 14) == 0)
			{//reqst freq
				RFTransmit(offset_words[1][g_rf_chnl_nbr], 12);
			}
			
			memcpy(g_rf_buffer.buffer_data, g_rf_buffer.recv_data, g_rf_buffer.recv_count);
		}

		//B_dataPush(g_rf_buffer.buffer_data+1, g_rf_buffer.buffer_data, 8);	   //op+flag+len
		p_prot = (stru_protocol_inst_t *)(g_rf_buffer.buffer_data);
		
		switch((p_prot->p_protypeH << 8) | p_prot->p_protypeL)
		{
//			case READ_PRODUCT_ID:
//			case BOOTLOADER_READ_VERSIONS:
//			break;
//			case READ_SUB_ID:
//				if(RFsystem.SYS_Mode == ALL_OPEN)
//				{//hub, recv through wireless, upload device ID
//					memcpy(RFsystem.device[g_rf_chnl_nbr],g_rf_buffer.recv_data+9,20);
//					RFsystem.device_count |= (0x01 << g_rf_chnl_nbr);
//					return;
//				}
//			break;
				
			case GET_DEVICE_NAME_ID_FROM_SUB:
			{
				if(RFsystem.SYS_Mode == ALL_OPEN)
				{//hub, store device name and id from sub
					//RFsystem.device[g_rf_chnl_nbr]
					//RFsystem.device_count |= (0x01 << g_rf_chnl_nbr);
					memcpy(RFsystem.device[g_rf_chnl_nbr], g_rf_buffer.recv_data+4, 14);
					RFsystem.device_count |= (0x01 << g_rf_chnl_nbr);
					if(g_is_sub_hub_paired)
					{						
						if(0 == memcmp(common_infomation.deviceid_pair, &RFsystem.device[g_rf_chnl_nbr][8],\
							sizeof(common_infomation.deviceid_pair)))
						{
							RF_channel_set(g_rf_chnl_nbr);
							g_is_rf_connected = 1;
						}
					}
					
					return;
				}
				else if(RFsystem.SYS_Mode == RF_OPEN)
				{//sub, resp device name and id
					p_prot->p_flag = DEFAULT_RESP_RESULT_CODE;
					p_prot->p_length = 14;
					memcpy(p_prot->p_data, common_infomation.device_name, 8);
					memcpy(p_prot->p_data+8, common_infomation.deviceid, 6);
					//
					g_rf_buffer.send_count = p_prot->p_length + ADD_LENGTH;
					memcpy(g_rf_buffer.send_data, (void *)p_prot, g_rf_buffer.send_count);
					RFTransmit(g_rf_buffer.send_data, g_rf_buffer.send_count);
					return;
				}
			}
			break;  /*GET_DEVICE_NAME_ID_FROM_SUB*/
				
			default:
			break;
		}

		if(RFsystem.SYS_Mode == ALL_OPEN)  //hub
		{
			//test
			static uint16_t dbg_cnt2 = 0;
			if((g_rf_buffer.recv_data[1] == 0x05/*g_rf_buffer.recv_data[2]*/) \
			&& (g_rf_buffer.recv_data[2] == 0x05/*g_rf_buffer.recv_data[3]*/) \
			&& (g_rf_buffer.recv_data[3] == 0x05/*g_rf_buffer.recv_data[4]*/))
			{
				dbg_cnt2++;
			}
			g_send_data(g_rf_buffer.recv_data, g_rf_buffer.recv_count, BLUETOOTH_CTL_PTL);  //to ble
		}
		else if(RFsystem.SYS_Mode == RF_OPEN)  //device
		{
			//test
			static uint16_t dbg_cnt = 0;
			if((g_rf_buffer.recv_data[1] == 0x05/*g_rf_buffer.recv_data[2]*/) \
			&& (g_rf_buffer.recv_data[2] == 0x05/*g_rf_buffer.recv_data[3]*/) \
			&& (g_rf_buffer.recv_data[3] == 0x05/*g_rf_buffer.recv_data[4]*/))
			{
				dbg_cnt++;
			}
			
			g_send_data(g_rf_buffer.recv_data, g_rf_buffer.recv_count, USART);  //to uart
		}
	}
}
/*****************************************************************************/

//wireless repeat
/*****************************************************************************/
void wireless_repeat(void)
{
	if(RFsystem.SYS_Mode == ALL_OPEN)
	{
		if(g_rf_buffer.Re_count)
		{
			if(!g_rf_buffer.Re_Wait_delay)  //timeout, repeat
			{
				RFTransmit(g_rf_buffer.send_data, g_rf_buffer.send_count);
				g_rf_buffer.Re_Wait_delay = 10;
				g_rf_buffer.Re_count--;  //repeat count--
			}
		}
	}
}
/*****************************************************************************/

//wireless mode, hopping for idle channel, 1~2s
/*****************************************************************************/
void RF_offset_event(void)
{
	unsigned char flag = 1;

	while(flag)
	{
		flag = 1;
		hopping_time = 1000 / 20;
		memcpy(g_rf_buffer.send_data,offset_words[0][g_rf_chnl_nbr], 14);
		g_rf_buffer.send_count = 14;
		RFTransmit(g_rf_buffer.send_data, g_rf_buffer.send_count);
		
		while(hopping_time)
		{
			if((RF_flag & 0x05) == 0x05)
			{
				g_rf_buffer.recv_count = RFPacketReceived(g_rf_buffer.recv_data);
				if(g_rf_buffer.recv_count)
				{
					if(memcmp(g_rf_buffer.recv_data, offset_words[1][g_rf_chnl_nbr], 12) == 0)
					{//the freq channel is occupied!
						g_rf_chnl_nbr++;
						g_rf_chnl_nbr %= RF_MAX_channel;
						si4432_init();
						flag = 2;
						break;
					}
				}
				RF_flag_clearRx;
				//turn to rx mode
				RFIdle();
				RFReceive();
			}
		}
		if(flag == 1)
		{
			break;  //not recved freq occupied
		}
	}
}
/*****************************************************************************/

//hub, hopping, scan slave device, get ID, cycle scan
/*****************************************************************************/
static stru_protocol_inst_t s_prot_data;
void frequency_hopping(void)
{
	if(RFsystem.SYS_Mode != ALL_OPEN)
	{return;}

	if(!RFsystem.ch_scan_time)
	{
		g_rf_chnl_nbr++;
		g_rf_chnl_nbr %= RF_MAX_channel;

		WLS_enable();
		si4432_init();
		RFIdle();
		RFReceive();
		
		RFsystem.ch_scan_time = RF_SCAN_PERIOD;
		//read device id and local name
		s_prot_data.p_protypeH = GET_DEVICE_NAME_ID_FROM_SUB >> 8;
		s_prot_data.p_protypeL = GET_DEVICE_NAME_ID_FROM_SUB & 0xFF;
		s_prot_data.p_flag = 0x00;
		s_prot_data.p_length = 0;
		g_rf_buffer.send_count = s_prot_data.p_length + ADD_LENGTH;
		memcpy(g_rf_buffer.send_data, (void *)(&s_prot_data), g_rf_buffer.send_count);
		RFTransmit(g_rf_buffer.send_data, g_rf_buffer.send_count);
	}
}
/*****************************************************************************/

/*****************************************************************************/
void RF_channel_set(unsigned char ptr)
{
#if 0
	unsigned char i, j = 0;
	unsigned char buffer;
	
	buffer = RFsystem.device_count;
	
	for(i = 0; i < 4; i++)
	{
		if(buffer & 0x01)
		{
			j++;
		}
		if(j == ptr)
		{
			g_rf_chnl_nbr = i;
			si4432_init();
			RFIdle();
			RFReceive();
			break;
		}
		buffer >>= 1;
	}
#else
	//0x01,0x02, 0x03, 0x04
	//the rf channel number is 0, 1, 2, 3
	g_rf_chnl_nbr = ptr - 1;
	si4432_init();
	RFIdle();
	RFReceive();
#endif
}
/*****************************************************************************/
							
void uart_send_string(uint8_t *pstr, uint32_t len)
{
	uint32_t i;
	for(i = 0; i < len; i++)
	{
		app_uart_put(pstr[i]);
	}
}

extern ble_custom_t m_custom;

void g_send_data(uint8_t *data, uint32_t len, uint8_t ch)
{
	//dbg
	static uint8_t pkg_data[20];
	uint16_t i, unit_len;
	enum_noti_char_t notify_char;
	switch(ch)
	{
		case BLUETOOTH_CTL_PTL:
		case BLUETOOTH_NOTI_EVT:
		case BLUETOOTH_NOTI_ANGLE:
		case BLUETOOTH_NOTI_LRL_SETTINGS:				
		{
			if(ch == BLUETOOTH_CTL_PTL)
			{
				notify_char = NOTIFICATION_CTRL_PROT;
			}
			else if(ch == BLUETOOTH_NOTI_EVT)
			{
				notify_char = NOTIFICATION_NOTI_CHAN;
			}
			else if(ch == BLUETOOTH_NOTI_ANGLE)
			{
				notify_char = NOTIFICATION_ANGLE_CHAN;
			}
			else if(ch == BLUETOOTH_NOTI_LRL_SETTINGS)
			{
				notify_char = NOTIFICATION_LRL_SETTINGS_CHAN;
			}
			//unit_len = len;
			if(len > 20)
			{
				for(i = 0; i < len; i = i + 20)
				{
					unit_len = (len > (i+20))?(20):(len-i);
					memcpy(pkg_data, data + i, unit_len);
					ble_custom_string_send(&m_custom, pkg_data, unit_len, notify_char);
				}
			}
			else
			{ble_custom_string_send(&m_custom, data, len, notify_char);}
		}
		break;
		case USART:
			uart_send_string(data, len);
		break;
		case WIRELESS:
			memcpy(g_rf_buffer.send_data, data, len);
			g_rf_buffer.send_count = len;
			RFTransmit(g_rf_buffer.send_data, g_rf_buffer.send_count);
		break;
		default:
		break;
	}
}

//return the ascii string length
uint8_t myint2ascii(int data, char *pstr)
{
	//max data is 2147483647;
	uint8_t is_negative = 0;
	int8_t i, j;
	char sz_tmp[15] = {0};
	char *p = pstr;
	if(data < 0)
	{
		is_negative = 1;
		data = -data;
	}
	i = 0;
	if(data == 0)
	{
		*p = '0';
		return 1;
	}
	while(data > 0)
	{
		sz_tmp[i] = data % 10 + '0';
		i++;
		data /= 10;
	}
	if(is_negative)
	{
		*p = '-';
		p++;
	}
	for(j = i - 1; j >= 0; j--)
	{
		*p = sz_tmp[j];
		p++; 
	}
	//e.g. -1 to "-1", i=1, return 2
	return i + is_negative;
}

int my_atoi(char *ptr, int len)
{
	int sum =0;
	while(len)
	{
		sum = sum*10;
		sum +=*ptr -'0';
		len--;
		ptr++;
	}
	return sum; 
}

static uint32_t getstrbetween(uint8_t *des,uint8_t *src,uint32_t maxlength,char head,char tail)
{
	uint32_t i ;
	uint32_t j ;
	uint32_t length ;
	for(i=0;i<maxlength;i++)
	{
		if(*(src+i)==head) break ;
	}
	if(i==maxlength) return 0 ;
	for(j=i+1;j<maxlength;j++)
	{
		if(*(src+j)==tail) break ;
	}
	if(j==maxlength) return 0 ;
	length = j-i-1 ;
	memcpy(des,src+i+1,length) ;
	memset(des+length,0,i+1) ;

	return length ;
}
ret_code_t         ret;
void uart_cmd_event(stru_usart_data_t *ptr)
{
	uint32_t str_len;
	uint8_t str_buf[50];
	uint8_t notibuf[50] = {0};
	char sz_buf[20] = {0};
	uint8_t len, pos, k;
	
	if(!g_LRL_is_power_on && g_ble_device_type == SUB)
	{
		g_LRL_is_power_on = 1;
		nrf_gpio_cfg_input(HPP_PLOCK, NRF_GPIO_PIN_NOPULL);
	}
	if(memcmp(ptr->data+1, "mode?", 5) == 0)
	{
		switch(RFsystem.SYS_Mode)
		{
			case BT_OPEN:
				uart_send_string("BT_OPEN", 7);
			break;
			case RF_OPEN:
				uart_send_string("RF_OPEN", 7);
			break;
			case ALL_OPEN:
				uart_send_string("ALL_OPEN", 8);
			break;
			default:
			break;
		}
	}
	else if(memcmp(ptr->data+1, "mode+", 5) == 0)
	{//"mode+" cmd...
		if(memcmp(ptr->data+6, "VER:", 4) == 0)
		{
			str_len = getstrbetween(str_buf,ptr->data,ptr->len,'{','}');
			memcpy(common_infomation.main_mcu_fw_version,&str_buf[9],str_len-9);
			common_infomation.main_mcu_fw_version[str_len-9] = 0;
			ret = update_user_flash(common_infomation.main_mcu_fw_version, sizeof(common_infomation.main_mcu_fw_version), DEVICE_INFO_FILE_ID, DEVICE_RECORD_KEY_MAIN_MCU_FW_VERSION);
			uart_send_string("ok", 2);
		}
		else if(memcmp(ptr->data+6, "lockTool", 8) == 0)
		{
			common_infomation.manufacturer_byte_11_flag |= DEVICE_STATUS_CODE_LOCKED;
			update_user_flash(&common_infomation.manufacturer_byte_11_flag, 1, DEVICE_INFO_FILE_ID, DEVICE_RECORD_KEY_MANUFACTURER_FLAG);
			uart_send_string("ok", 2);
		}
		else if(memcmp(ptr->data+6, "unlockTool", 10) == 0)
		{
			common_infomation.manufacturer_byte_11_flag &= ~DEVICE_STATUS_CODE_LOCKED;
			update_user_flash(&common_infomation.manufacturer_byte_11_flag, 1, DEVICE_INFO_FILE_ID, DEVICE_RECORD_KEY_MANUFACTURER_FLAG);
			uart_send_string("ok", 2);
		}
		else if(memcmp(ptr->data+6, "disableTool", 11) == 0)
		{
			common_infomation.manufacturer_byte_11_flag |= DEVICE_STATUS_CODE_DISABLED;
			update_user_flash(&common_infomation.manufacturer_byte_11_flag, 1, DEVICE_INFO_FILE_ID, DEVICE_RECORD_KEY_MANUFACTURER_FLAG);
			uart_send_string("ok", 2);
		}
		else if(memcmp(ptr->data+6, "enableTool", 10) == 0)
		{
			common_infomation.manufacturer_byte_11_flag &= ~DEVICE_STATUS_CODE_DISABLED;
			update_user_flash(&common_infomation.manufacturer_byte_11_flag, 1, DEVICE_INFO_FILE_ID, DEVICE_RECORD_KEY_MANUFACTURER_FLAG);
			uart_send_string("ok", 2);
		}
		else if(memcmp(ptr->data+6, "closeBLE", 8) == 0)
		{
			if(g_ble_device_type == SUB)
			{
				if(RFsystem.SYS_Mode == NOTHING || RFsystem.SYS_Mode == RF_OPEN)
				{
					RFsystem.sysmode_changed = 0;
				}
				else if(RFsystem.SYS_Mode == ALL_OPEN)
				{//do not need to init rf!!
					g_disconnect_ble();
					nrf_delay_ms(15);
					RFsystem.SYS_Mode = RF_OPEN;
					RFsystem.sysmode_changed |= SYSMODE_CHANGED_FLAG_STOP_ADV;
				}
				else if(RFsystem.SYS_Mode == BT_OPEN)
				{
					RFsystem.SYS_Mode = NOTHING;
					if(common_infomation.manufacturer_byte_11_flag & DEVICE_STATUS_CODE_UNCOMMISSIONED)
					{
						g_disconnect_ble();
						nrf_delay_ms(15);
						sd_ble_gap_adv_stop();
						g_is_advertising = 0;
						//RFsystem.sysmode_changed |= SYSMODE_CHANGED_FLAG_STOP_ADV;
					}
					else
					{
						RFsystem.sysmode_changed = 0;	
						sd_ble_gap_adv_stop();
						nrf_delay_ms(20);
						g_init_and_start_slow_adv();
					}	
				}
				g_is_adv_interval_changed = 0 ;
			}
			
			if(g_ble_device_type == LDM)
			{
				common_infomation.manufacturer_byte_11_flag &= ~DEVICE_STATUS_CODE_BUTTON_PUSHED;
				sd_ble_gap_adv_stop();
				nrf_delay_ms(20);
				g_init_and_start_normal_adv();	
			}			
			uart_send_string((uint8_t *)"ok", 2) ;
		}
		else if(memcmp(ptr->data+6, "openBLE", 7) == 0)
		{
			if(g_ble_device_type == SUB)
			{
				if(RFsystem.SYS_Mode == ALL_OPEN || RFsystem.SYS_Mode == BT_OPEN)
				{
					RFsystem.sysmode_changed = 0;
					if(RFsystem.SYS_Mode == BT_OPEN && !g_is_advertising)
					{
						sd_ble_gap_adv_stop();
						nrf_delay_ms(20);
						g_init_and_start_fast_adv();
					}
				}
				else
				{
					if(RFsystem.SYS_Mode == RF_OPEN)
					{
						//in LRL, close RF, then open BLE
						WLS_disable();
					}
					RFsystem.SYS_Mode = BT_OPEN;
					sd_ble_gap_adv_stop();
					nrf_delay_ms(20);
					g_init_and_start_fast_adv();
					RFsystem.sysmode_changed = SYSMODE_CHANGED_FLAG_NORMAL;
				}
			}
			if(g_ble_device_type == LDM)
			{
				common_infomation.manufacturer_byte_11_flag |= DEVICE_STATUS_CODE_BUTTON_PUSHED;
				sd_ble_gap_adv_stop();
				nrf_delay_ms(20);
				g_init_and_start_fast_adv();	
			}
			uart_send_string((uint8_t *)"ok", 2);
		}
		else if(memcmp(ptr->data+6, "closeRF", 7) == 0)
		{
			if(RFsystem.SYS_Mode == NOTHING || RFsystem.SYS_Mode == BT_OPEN)
			{
				RFsystem.sysmode_changed = 0;
			}
			else
			{
				WLS_disable();
				if(RFsystem.SYS_Mode == ALL_OPEN)
				{//do not need to init ble!!
					RFsystem.SYS_Mode = BT_OPEN;
				}
				else if(RFsystem.SYS_Mode == RF_OPEN)
				{RFsystem.SYS_Mode = NOTHING;}
				RFsystem.sysmode_changed = SYSMODE_CHANGED_FLAG_NORMAL;
			}
			uart_send_string("ok", 2);
		}
		else if(memcmp(ptr->data+6, "openRF", 6) == 0)
		{
			if(RFsystem.SYS_Mode == ALL_OPEN || RFsystem.SYS_Mode == RF_OPEN)
			{
				//RFsystem.sysmode_changed = 0;
			}
			else
			{
				if((RFsystem.SYS_Mode == BT_OPEN) || g_is_advertising)
				{
					//in LRL, close BLE, then open RF
					//stopping adv will be in the sysmode_check_timeout_handle
					g_disconnect_ble();
					nrf_delay_ms(15);
//					sd_ble_gap_adv_stop();
//					g_is_advertising = 0;
					RFsystem.sysmode_changed |= SYSMODE_CHANGED_FLAG_STOP_ADV;
				}
				RFsystem.SYS_Mode = RF_OPEN;
				//need to init rf and stop adv
				RFsystem.sysmode_changed |= SYSMODE_CHANGED_FLAG_INIT_RF;
			}
			g_is_adv_interval_changed = 0 ;
			uart_send_string("ok", 2);
		}
		else if(memcmp(ptr->data+6, "openPushButton", 14) == 0)
		{
			common_infomation.manufacturer_byte_11_flag |= DEVICE_STATUS_CODE_BUTTON_PUSHED;
			if(ble_adv_mode == BLE_ADV_MODE_FAST)
			{
				sd_ble_gap_adv_stop();
				nrf_delay_ms(20);
				g_init_and_start_fast_adv();
			}
			uart_send_string("ok", 2);
		}
		else if(memcmp(ptr->data+6, "closePushButton", 14) == 0)
		{
			common_infomation.manufacturer_byte_11_flag &= ~DEVICE_STATUS_CODE_BUTTON_PUSHED;
			if(ble_adv_mode == BLE_ADV_MODE_FAST)
			{
				sd_ble_gap_adv_stop();
				nrf_delay_ms(20);
				g_init_and_start_fast_adv();
			}
			uart_send_string("ok", 2);
		}		
		else if(memcmp(ptr->data+6, "openProvision", 13) == 0)
		{
			common_infomation.manufacturer_byte_11_flag&= ~DEVICE_STATUS_CODE_UNCOMMISSIONED;
			update_user_flash(&common_infomation.manufacturer_byte_11_flag, 1, DEVICE_INFO_FILE_ID, DEVICE_RECORD_KEY_MANUFACTURER_FLAG);
			uart_send_string("ok", 2);
		}
		else if(memcmp(ptr->data+6, "closeProvision", 14) == 0)
		{
			common_infomation.manufacturer_byte_11_flag |= DEVICE_STATUS_CODE_UNCOMMISSIONED;
			update_user_flash(&common_infomation.manufacturer_byte_11_flag, 1, DEVICE_INFO_FILE_ID, DEVICE_RECORD_KEY_MANUFACTURER_FLAG);
			uart_send_string("ok", 2);
			if(g_ble_device_type == HUB)
			{
					sd_ble_gap_adv_stop();
					nrf_delay_ms(20);
					g_init_and_start_fast_adv();
			}
		}
		else if(memcmp(ptr->data+6, "MainCellLevel", 13) == 0)
		{
			pos = 6 + strlen("MainCellLevel+") ;
			memcpy(sz_buf, &ptr->data[pos], ptr->len - pos - 1);
			g_batt_level = my_atoi(sz_buf, ptr->len - pos - 1) ;
			m_bas.is_notification_supported = 1 ;
			ble_bas_battery_level_update(&m_bas, g_batt_level) ;
		}
		else if(memcmp(ptr->data+6, "GetCoincellLevel", 16) == 0)
		{
			memcpy((char *)notibuf, "{mode+CoincellLevel+", strlen("{mode+CoincellLevel+"));
			sprintf((char *)&notibuf[sizeof("{mode+CoincellLevel+")-1], "%d}", g_coincell_level);
			g_send_data(notibuf, sizeof(notibuf), USART);
			memset(notibuf, 0 ,50);
		}
		else if(memcmp(ptr->data+6, "GetSyncTime", 11) == 0)
		{
			memcpy((char *)notibuf, "{mode+SyncTime+", strlen("{mode+SyncTime+"));
			sprintf((char *)&notibuf[sizeof("{mode+SyncTime+")-1], "%4d%02d%02d%02d%02d%02d}", \
				g_device_time.year, g_device_time.month, g_device_time.day, g_device_time.hour, g_device_time.minute, g_device_time.second);
			g_send_data(notibuf, sizeof(notibuf), USART);
			memset(notibuf, 0 ,50);
		}
		else if(memcmp(ptr->data+6, "MainboardTemp", strlen("MainboardTemp")) == 0)
		{
			memcpy((char *)notibuf, "{mode+SyncTime+", strlen("{mode+SyncTime+"));
			sprintf((char *)&notibuf[strlen("{mode+SyncTime+")], "%4d%02d%02d%02d%02d%02d}", \
				g_device_time.year, g_device_time.month, g_device_time.day, g_device_time.hour, g_device_time.minute, g_device_time.second);
			g_send_data(notibuf, sizeof(notibuf), USART);
			memset(notibuf, 0 ,50);
		}		
		else if(memcmp(ptr->data+6, "notiData", strlen("notiData")) == 0)
		{//{mode+notiData+<DT...>,<>,_}
			//uint8 status;
			int rc ;
			char* strp_tmp ;
			int len_tmp = 0 ;
			uint8_t commom_notibuf[50];
			uint8_t angle_notibuf[50];			
			len_tmp = ptr->len - strlen("{mode+notiData+");
			if(len_tmp > 50) 
			{
				uart_send_string("error", 5);
			}
			else
			{				
				rc = sscanf((char*)ptr->data, "{mode+notiData+%s", commom_notibuf) ;
				if(rc != -1 && rc != 0)
				{
					strp_tmp = strchr((char*)commom_notibuf, ',') ;
					len_tmp = len_tmp - strlen(strp_tmp);
					if(len_tmp > 1 && *(strp_tmp+1) != '_')
					{
						g_send_data(commom_notibuf, len_tmp, BLUETOOTH_NOTI_EVT);
					}

						
					memcpy(notibuf, strp_tmp+1, strlen(strp_tmp)-1);
					len_tmp = strlen(strp_tmp)-1;
					strp_tmp = strchr((char*)notibuf, ',') ;
					len_tmp = len_tmp - strlen(strp_tmp);
					if(len_tmp >1 && notibuf[0] != '_')
					{
						memcpy(angle_notibuf, notibuf, len_tmp);
						g_send_data(angle_notibuf, len_tmp, BLUETOOTH_NOTI_ANGLE);
					}
					
					if((strlen(strp_tmp)-1) >= 1 && *(strp_tmp+1) != '_')
					{
						memcpy(sz_buf, strp_tmp+1, strlen(strp_tmp)-2);
						g_batt_level = my_atoi(sz_buf, strlen(strp_tmp)-2) ;
						m_bas.is_notification_supported = 1 ;
						ble_bas_battery_level_update(&m_bas, g_batt_level) ;
					}	
				}
				else
				{
					uart_send_string("error", 5);
				}
			}			
		}		
		else if(memcmp(ptr->data+6, "notiMeas", strlen("notiMeas")) == 0)
		{//{mode+notiMeas+<DT...>}
			pos = 6 + strlen("notiMeas+");
			for(k = ptr->len - 1; k > pos; k--)
			{
				if(ptr->data[k] == '>')
				{
					break;
				}
			}
			len = k - pos + 1;
			memcpy(notibuf, ptr->data + pos, len);
			g_send_data(notibuf, len, BLUETOOTH_NOTI_EVT);
		}
		else if(memcmp(ptr->data+6, "notiAngle", strlen("notiAngle")) == 0)
		{//{mode+notiAngle+<...>}
			pos = 6 + strlen("notiAngle+");
			for(k = ptr->len - 1; k > pos; k--)
			{
				if(ptr->data[k] == '>')
				{
					break;
				}
			}
			len = k - pos + 1;
			memcpy(notibuf, ptr->data + pos, len);
			g_send_data(notibuf, len, BLUETOOTH_NOTI_ANGLE);
			memset(notibuf, 0 ,50);
		}
		else if(memcmp(ptr->data+6, "notiEvent", strlen("notiEvent")) == 0)
		{//"{mode+notiEvent+ 14B }"
			//uint8_t event_type;
			//uint16_t event_idx;
			//uint8_t evt_date_time[14];  //e.g "20170630032233"
			//uint16_t evt_duration_ms;
			//            type,idx,valid,nbr,time,duration
			//put string to ":0,5,1,6,20170630032233,500:", then notify to APP
			//ptr->data+6+strlen("notiEvent+")
			uint8_t len = 6 + strlen("notiEvent+");
			uint8_t pos, k;
			int tmp;
			notibuf[0] = ':';
			notibuf[1] = ptr->data[len] + '0';  //event type
			notibuf[2] = ',';
			len++;
			tmp = ptr->data[len] | ((int)ptr->data[len+1] << 8);  //event index
			pos = 3;
			k = myint2ascii(tmp, sz_buf);
			memcpy(notibuf + pos, sz_buf, k);
			pos += k;
			notibuf[pos++] = ',';
			notibuf[pos++] = '1';  //valid
			notibuf[pos++] = ',';
			tmp = ptr->data[len] | ((int)ptr->data[len+1] << 8) + 1;  //nbr=idx+1
			k = myint2ascii(tmp, sz_buf);
			memcpy(notibuf + pos, sz_buf, k);
			pos += k;
			len += 2;
			notibuf[pos++] = ',';
			memcpy(notibuf + pos, ptr->data + len, 14);  //time
			pos += 14;
			len += 14;
			notibuf[pos++] = ',';
			tmp = ptr->data[len] | ((int)ptr->data[len+1] << 8);  //duration
			k = myint2ascii(tmp, sz_buf);
			memcpy(notibuf + pos, sz_buf, k);
			pos += k;
			notibuf[pos++] = ':';
			//
			g_send_data(notibuf, pos, BLUETOOTH_NOTI_EVT);
			memset(notibuf, 0 ,50);
		}
		else if(memcmp(ptr->data+6, "getTemperature?", strlen("getTemperature?")) == 0)
		{
			int32_t rt_temperature = 0;
			sd_temp_get(&rt_temperature);
			rt_temperature = rt_temperature / 4;
			memset(notibuf, 0 ,50);
			
			sprintf((char *)notibuf, "{mode+getTemperature=%04d}", rt_temperature);
			g_send_data(notibuf, strlen((char *)notibuf), USART);
			memset(notibuf, 0 ,50);
		}
		else
		{
			uart_send_string((uint8_t* )"error", 5);
		}
	}/*"mode+"*/
	else
	{//other data will send out by ble or rf
		if(RFsystem.SYS_Mode == RF_OPEN)
		{
			g_send_data(ptr->data, ptr->len, WIRELESS);
			//test, re_count=0. !!!!!!
			g_rf_buffer.Re_count = 0;//4;
		}
		else
		{
			if(g_ble_device_type == LDM)
			{
				g_send_data(ptr->data, ptr->len, BLUETOOTH_CTL_PTL);
			}
			else
			{
				uint16_t op_code;
				ble_ctlpro_frame_t *ctlpro_inst = 0 ;	//control protocol frame instance.	
				size_t len_data ;	
				ctlpro_inst = (ble_ctlpro_frame_t* )(ptr->data);
				len_data = ctlpro_inst->datlen ;
				if(len_data >= 4)
				{
					op_code = GET_OPCODE() ;
					switch(op_code)
					{
						case OP_CODE_GET_PROTOCOL_VERSION :
						case OP_CODE_GET_SUB_DEVICE_FIRMWARE_VERSION:
						case OP_CODE_SET_DEVICE_ID:
						case OP_CODE_TEST_DEVICE_LED:
						case OP_CODE_READ_DEVICE_STATUS:
						case OP_CODE_WRITE_TOOL_PAIRED:
						case OP_CODE_READ_TOOL_PAIRED:
						case OP_CODE_WRITE_TOOL_ENABLED:
						case OP_CODE_READ_TOOL_ENABLED:
						case OP_CODE_TOOL_IDENTIFY:
						case OP_CODE_GET_LDM_STATUS:
						case OP_CODE_TURN_LASER_ON:
						case OP_CODE_REGULAR_MEASURE:
						case OP_CODE_ENTER_EXIT_CONTINUOUS_MEASURE:
						case OP_CODE_CHANGE_MEASURING_START_POINT:
						case OP_CODE_CHANGE_MEASURING_UNIT_ON_LCD:
						case BCPC_SET_LDM_BASE_CONST:
						case BCPC_GET_LDM_BASE_CONST:
						case BCPC_SET_LDM_ANG_PARAM1:
						case BCPC_GET_LDM_ANG_PARAM1:
						case BCPC_SET_LDM_ANG_PARAM2:
						case BCPC_GET_LDM_ANG_PARAM2:
						case BCPC_SET_LDM_FACTORY_CALIB:
						case BCPC_RESET_LDM_EEPROM:
							g_send_data(ptr->data, ptr->len, BLUETOOTH_CTL_PTL);
							break;
						default:
							break;	
					}
				}
			}
		}
	}
}


extern const nrf_drv_spi_t m_spi_master_0;

//for si4432
unsigned char SpiRfReadRegister(unsigned char address)
{
	unsigned char tx[2], rx[2];

	tx[0] = address;
	tx[1] = 0x55; //don't care
	
	nrf_gpio_pin_clear(SPIM0_SS_PIN);
	nrf_drv_spi_transfer(&m_spi_master_0, tx, sizeof(tx), rx, sizeof(rx));
	nrf_gpio_pin_set(SPIM0_SS_PIN);
	return rx[1];
}

void SpiWriteAddressData(unsigned char address, unsigned char data)
{
	unsigned char tx[2], rx[2];

	tx[0] = 0x80 | address;
	tx[1] = data;

	nrf_gpio_pin_clear(SPIM0_SS_PIN);
	nrf_drv_spi_transfer(&m_spi_master_0, tx, sizeof(tx), rx, sizeof(rx));
	nrf_gpio_pin_set(SPIM0_SS_PIN);
}



/////////////////////////////
uint32_t GetStrBetween(uint8_t *des,uint8_t *src,uint32_t maxlength,char head,char tail)
{
	uint32_t i;
	uint32_t j;
	uint32_t length;
	for(i=0;i<maxlength;i++)
	{
		if(*(src+i)==head) break;
	}
	if(i==maxlength) return 0;
	for(j=i+1;j<maxlength;j++)
	{
		if(*(src+j)==tail) break;
	}
	if(j==maxlength) return 0;
	length = j-i-1;
	memcpy(des,src+i+1,length);
	memset(des+length,0,i+1);

	return length;
}
//-------------------
uint8_t 	B_datapop(uint8_t *ptr,uint8_t *str,uint8_t len)
{
	uint8_t buffer;	

	while(len--)
	{
		buffer = ((*str & 0xf0)>>4);
		if(buffer <= 9)
		{*ptr = buffer + '0';}
		else if((buffer >= 0x0a)&&(buffer <= 0x0f))
		{*ptr = buffer -0x0a + 'A';}
		else 
		{return 0xff;}
		*ptr++;

		buffer = (*str & 0x0f);
		if(buffer <= 9)
		{*ptr = buffer + '0';}
		else if((buffer >= 0x0a)&&(buffer <= 0x0f))
		{*ptr = buffer - 0x0a + 'A';}
		else 
		{return 0xff;}
		*ptr++;

		*str++;					    
	}
	return len*2;	
}
//-------------------
uint8_t 	B_dataPush(uint8_t *ptr,uint8_t *str,uint8_t len)
{
	uint8_t buffer;	

	len /= 2;
	while(len--)
	{
		buffer = 0;
		if((*ptr >= '0')&&(*ptr <= '9'))
		{buffer |= ((*ptr-'0')<<4);}
		else if((*ptr >= 'a')&&(*ptr <= 'f'))
		{buffer |= ((*ptr-'a'+0x0A)<<4);}
		else if((*ptr >= 'A')&&(*ptr <= 'F'))
		{buffer |= ((*ptr-'A'+0x0A)<<4);}
		else 
		{return 0xff;}
		*ptr++;

		if((*ptr >= '0')&&(*ptr <= '9'))
		{buffer |= (*ptr-'0');}
		else if((*ptr >= 'a')&&(*ptr <= 'f'))
		{buffer |= (*ptr-'a'+0x0A);}
		else if((*ptr >= 'A')&&(*ptr <= 'F'))
		{buffer |= (*ptr-'A'+0x0A);}
		else 
		{return 0xff;}
		*ptr++;

		*str = buffer;
		*str++;
	}
	return SUCCESS;
}
//-------------------------XOr
uint8_t XOr_Get ( uint8_t *Ptr , uint8_t Length )
{
	uint8_t XOr = 0;

	while ( Length-- )
	{
	  	XOr ^= *Ptr;
		*Ptr++;
	}
	return ( XOr );
}

/*****************************************************************************/
uint32_t GetStrInclude(uint8_t *des,uint8_t *src,uint32_t maxlength,char head,char tail)
{
	uint32_t i;
	uint32_t j;
	uint32_t length;
	for(i=0;i<maxlength;i++)
	{
		if(*(src+i)==head) break;
	}
	if(i==maxlength) return 0;
	for(j=i+1;j<maxlength;j++)
	{
		if(*(src+j)==tail) break;
	}
	if(j==maxlength) return 0;
	length = j-i+1;
	memcpy(des,src+i,length);
	memset(des+length,0,1);
	return length;
}
/*****************************************************************************/

void led_gpio_init(void)
{
	if(g_ble_device_type == SUB||g_ble_device_type == HUB)
	{
		nrf_gpio_cfg_output(WLS_PWR_EN);
		nrf_gpio_cfg_output(SPIM0_SS_PIN);
		nrf_gpio_cfg_output(LED_BT);
		nrf_gpio_cfg_output(LED_WLS);
		nrf_gpio_cfg_output(BLE_APP_DETECT);
		nrf_gpio_cfg_input(SI4432_NIRQ, NRF_GPIO_PIN_NOPULL);
		nrf_gpio_cfg_input(HPP_PLOCK, NRF_GPIO_PIN_NOPULL);
		//
		nrf_gpio_pin_clear(WLS_PWR_EN);
		nrf_gpio_pin_clear(LED_BT);
		nrf_gpio_pin_clear(LED_WLS);
		nrf_gpio_pin_set(SPIM0_SS_PIN);
		nrf_gpio_pin_set(BLE_APP_DETECT);  /*unconnected, High*/
	}
	else
	{
		nrf_gpio_cfg_output(LED_BT);
		nrf_gpio_cfg_output(BLE_APP_DETECT);
		nrf_gpio_pin_clear(LED_BT);
		nrf_gpio_pin_set(BLE_APP_DETECT);  /*unconnected, High*/
	}
}

/******************************************************************************/
void WLS_enable(void)
{
	nrf_gpio_pin_set(WLS_PWR_EN);
}
/******************************************************************************/

/******************************************************************************/
void WLS_disable(void)
{
	nrf_gpio_pin_clear(WLS_PWR_EN);
}
/******************************************************************************/

/******************************************************************************/
void bt_led_on(void)
{
	nrf_gpio_pin_set(LED_BT);
}
/******************************************************************************/

/******************************************************************************/
void bt_led_off(void)
{
	nrf_gpio_pin_clear(LED_BT);
}
/******************************************************************************/

/******************************************************************************/
void wls_led_on(void)
{
	nrf_gpio_pin_set(LED_WLS);
}
/******************************************************************************/

/******************************************************************************/
void wls_led_off(void)
{
	nrf_gpio_pin_clear(LED_WLS);
}
