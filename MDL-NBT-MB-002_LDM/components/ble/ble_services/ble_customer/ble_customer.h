#ifndef BLE_CUSTOMER_H__
#define BLE_CUSTOMER_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

//
#define USER_MEM_EN  1
#if USER_MEM_EN
extern ble_user_mem_block_t m_mem_block;
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
	NOTIFICATION_CTRL_PROT,
	NOTIFICATION_NOTI_CHAN,
	NOTIFICATION_ANGLE_CHAN,
	NOTIFICATION_LRL_SETTINGS_CHAN,
}enum_noti_char_t;
	
typedef struct ble_custom_s ble_custom_t;

typedef void (*ble_custom_data_handler_t) (ble_custom_t *p_custom, uint8_t *p_data, uint16_t length);

typedef struct{
	ble_custom_data_handler_t data_handler;
}ble_custom_init_t;

struct ble_custom_s{
	uint8_t                   uuid_type;
	uint16_t                  service_uuid;
	uint16_t                  service_handle;
	ble_gatts_char_handles_t  ctrl_prot_handles;
	ble_gatts_char_handles_t  ble_spec_ver_handles;
	ble_gatts_char_handles_t  noti_chan_handles;
	ble_gatts_char_handles_t  noti_angle_handles;
	ble_gatts_char_handles_t  noti_LRL_settings_handles;	
	uint16_t                  conn_handle;
	bool                      is_ctrl_prot_notification_enabled;
	bool                      is_noti_chan_notification_enabled;
	bool                      is_noti_angle_notification_enabled;
	bool                      is_noti_LRL_settings_notification_enabled;	
	ble_custom_data_handler_t data_handler;
};







uint32_t ble_custom_init(ble_custom_t *p_custom, ble_custom_init_t *p_custom_init);
void ble_custom_on_ble_evt(ble_custom_t *p_custom, ble_evt_t *p_ble_evt);
uint32_t ble_custom_string_send(ble_custom_t *p_custom, uint8_t *p_string, uint16_t length, enum_noti_char_t noti_char);







#ifdef __cplusplus
}
#endif

#endif  //BLE_CUSTOMER_H__
