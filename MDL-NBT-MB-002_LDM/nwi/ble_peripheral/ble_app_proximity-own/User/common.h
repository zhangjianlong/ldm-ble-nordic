#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>

#define MAX_UART_DATA_LEN  256  //100

#define RF_flag_setRx    (RF_flag |= 0x01)
#define RF_flag_clearRx  (RF_flag &= 0xFE)
#define RF_flag_setTx    (RF_flag |= 0x02)
#define RF_flag_clearTx  (RF_flag &= 0xFD)

#define RF_MAX_channel  4
                            //20 means 20*100=2000ms=2s
							//30 means 30*100=3000ms=3s
#define RF_SCAN_PERIOD  20  /*the unit is 100ms, in the delay_count_timeout_handle()*/

extern unsigned char RF_flag;
//
//typedef enum 
//{
//  ERROR = 0, 
//  SUCESS = !ERROR
//} ErrorStatus;
#define SUCCESS  0x00
#define FAILURE  0x01

#define SYSMODE_CHANGED_FLAG_NORMAL    0x01  //0b0000 0001
#define SYSMODE_CHANGED_FLAG_INIT_RF   0x10  //0b0001 0000
#define SYSMODE_CHANGED_FLAG_STOP_ADV  0x20  //0b0010 0000
//#define SYSMODE_CHANGED_FLAG_NORMAL  0x

#define DEVICE_STATUS_CODE_DEFAULT         	0x01
#define DEVICE_STATUS_CODE_DISABLED        	0x02
#define DEVICE_STATUS_CODE_LOCKED          	0x04
#define DEVICE_STATUS_CODE_LEND            	0x08
#define DEVICE_STATUS_CODE_CLIENT_CONNECTED	0x20
#define DEVICE_STATUS_CODE_BUTTON_PUSHED   	0x40
#define DEVICE_STATUS_CODE_UNCOMMISSIONED 	0x80


#define BLE_INFO_FILE_ID                  0x0001
#define BLE_INFO_RECORD_KEY_START         0x0001 /**< The beginning of the range of record keys reserved for BLE infomation. */
#define BLE_RECORD_KEY_DEVICE_ID	       BLE_INFO_RECORD_KEY_START	//use the snv position BLE_NVID_SN_CFG_START.
#define BLE_RECORD_KEY_DEVICE_NAME	       (BLE_INFO_RECORD_KEY_START + 1)
#define BLE_RECORD_KEY_DEVICE_PTI	       (BLE_INFO_RECORD_KEY_START + 2)
#define BLE_RECORD_KEY_DEVICE_CIDC	       (BLE_INFO_RECORD_KEY_START + 3)	//company id code.
#define BLE_RECORD_KEY_DEVICE_MODEL       (BLE_INFO_RECORD_KEY_START + 4)
#define BLE_RECORD_KEY_DEVICE_SN	       (BLE_INFO_RECORD_KEY_START + 5)

#define DEVICE_INFO_FILE_ID               0x0002
#define DEVICE_INFO_RECORD_KEY_START      0x0001 /**< The beginning of the range of record keys reserved for DEVICE infomation. */
#define DEVICE_RECORD_KEY_MANUFACTURER_FLAG  	DEVICE_INFO_RECORD_KEY_START
#define DEVICE_RECORD_KEY_MAIN_MCU_FW_VERSION  	(DEVICE_INFO_RECORD_KEY_START +1)
#define DEVICE_RECORD_KEY_DEVICE_ID_PAIR	     (DEVICE_INFO_RECORD_KEY_START +2)
enum{
	BLUETOOTH_CTL_PTL = 0,
	BLUETOOTH_NOTI_EVT,
	BLUETOOTH_NOTI_ANGLE,
	BLUETOOTH_NOTI_LRL_SETTINGS,
	USART,
	WIRELESS,
};

typedef enum
{ 
	off = 0,
	on,
	flash,
	once_flash,
}LaserMode;

typedef enum{
	NOTHING = 0,
	BT_OPEN,
	RF_OPEN,
	ALL_OPEN,
}RFWokeMode;

typedef struct{
     RFWokeMode SYS_Mode;
	LaserMode  runRF_LED;
	LaserMode  event_LED;
	unsigned char sysmode_changed;  //1: changed! 0: unchanged.
	unsigned char ch_scan_time;
	unsigned char device_count;
	unsigned char device[4][20];
}SYSTEset;

typedef struct{
	unsigned char Rev_flag;
	unsigned char recv_data[256];
	uint16_t recv_count;
	unsigned char send_data[256];
	uint16_t send_count;
	unsigned char Rev_delay;
	unsigned char Re_Wait_delay;  //wait for ack
	unsigned char Re_count;  //re-send count
	unsigned char buffer_data[256];
}stru_rf_data_t;


typedef enum{
	DB_NWI,
	DB_PLOTT,
	DB_STANLEY,
	DB_DEWALT,
	DB_UNKNOWN,
}device_brand_t ;

typedef enum 
{
	NWITOOL = 0,
	LDM,
	HUB,			//hub device, in detector, open ble and rf
	SUB			//sub device,in the hpp, do not open ble adv and rf at first.
}g_ble_device_type_t;


#pragma anon_unions
typedef struct{
	unsigned char  protocol_version[5] ;
	unsigned char  BLE_spec_version[5] ;
	unsigned char  ldm_swver[21] ;	//ldm software version.
	union
	{
		unsigned char manufacturer_byte_11_flag;
		uint32_t manufact_flag;
	};
	union
	{
		unsigned char manufact_name[32] ;	//a string, remember we need a '\0' at the end
		uint32_t manufact_data;
	};	
	union
	{
		unsigned char main_mcu_fw_version[32]; //a string, remember we need a '\0' at the end
		uint32_t fw_version;
	};
	union
	{
		unsigned char deviceid[6];
		uint32_t id_data;
	};
	union
	{
		unsigned char device_name[8] ;
		uint32_t name_data;	
	};
	union
	{
		unsigned char device_pti[3] ;
		uint32_t pti_data;
	};
	union
	{
		char  company_identifier_code[2] ;
		uint32_t company_code_data;
	};
	union
	{
		char model[21] ;//a string, remember we need a '\0' at the end.
		uint32_t model_data ;
	};
	union
	{
		char sn[14] ;
		uint32_t sn_data ;
	};
	union
	{
		unsigned char deviceid_pair[6];
		uint32_t id_data_pair;
	};	
}common_infomation_t ;
#pragma no_anon_unions

typedef enum
{
	JANUARY = 1,
	FEBRUARY,
	MARCH,
	APRIL,
	MAY,
	JUNE,
	JULY,
	AUGUST,
	SEPTEMBER,
	OCTOBER,
	NOVEMBER,
	DECEMBER
}month_calendar_t ;

typedef struct
{
	uint16_t year;
	month_calendar_t  month;
	uint8_t  day;
	uint8_t  hour;
	uint8_t  minute;
	uint8_t  second;
}g_device_time_t ;

#define FLASH_PAGE_SIZE    (NRF_FICR->CODEPAGESIZE)
//use the last page of application area flash!!
#define FLASH_PAGE_NUMBER  (NRF_FICR->CODESIZE - 9)

extern uint8_t g_status_code;


extern stru_rf_data_t g_rf_buffer;
extern SYSTEset RFsystem;


typedef struct{
	uint8_t idx;
	uint8_t len;
	uint8_t data[MAX_UART_DATA_LEN];
	uint8_t rx_idle_count;
	uint8_t rx_idle_count_en;
}stru_usart_data_t;

//
#pragma pack(1)
typedef struct{
	uint8_t p_protypeL;
	uint8_t p_protypeH;
	uint8_t p_flag;
	uint8_t p_length;
	uint8_t p_data[255];
}stru_protocol_inst_t;
#pragma pack()

extern stru_usart_data_t g_uart_data;

extern stru_rf_data_t g_rf_buffer;
extern SYSTEset RFsystem;

extern unsigned char g_rf_chnl_nbr;
extern uint8_t g_is_advertising;
extern uint8_t g_is_rf_connected;  //for RF HUB, ALL_OPEN mode

void g_disconnect_ble(void);
void advertising_start(void);

void uart_send_string(uint8_t *pstr, uint32_t len);
void g_send_data(uint8_t *data, uint32_t len, uint8_t ch);
void uart_cmd_event(stru_usart_data_t *ptr);

unsigned char SpiRfReadRegister(unsigned char address);
void SpiWriteAddressData(unsigned char address, unsigned char data);

void wireless_receive_handler(void);
void wireless_repeat(void);
void RF_offset_event(void);
void frequency_hopping(void);
void RF_channel_set(unsigned char ptr);


uint32_t GetStrBetween(uint8_t *des,uint8_t *src,uint32_t maxlength,char head,char tail);
uint8_t B_datapop(uint8_t *ptr,uint8_t *str,uint8_t len);
uint8_t B_dataPush(uint8_t *ptr,uint8_t *str,uint8_t len);
uint8_t XOr_Get( uint8_t *Ptr , uint8_t Length );
uint32_t GetStrInclude(uint8_t *des,uint8_t *src,uint32_t maxlength,char head,char tail);

void led_gpio_init(void);
void WLS_enable(void);
void WLS_disable(void);
void bt_led_on(void);
void bt_led_off(void);
void wls_led_on(void);
void wls_led_off(void);

#endif  /*__COMMON_H__*/

