#ifndef __PROTOCOL_H_
#define __PROTOCOL_H_

#include <string.h>

//#include "osal.h"
//#include "hal_types.h"
//#include "serialAppUtil.h"
//#include "simpleBLEperipheral.h"
//#include "si4432.h"


typedef struct packet
{
	unsigned int count;
	unsigned char buffer[255];
}packet_t, *ppacket_t;

typedef struct{
	char opcode_h ;
	char opcode_l ;
	char flag ;
	char datlen ;
	char dat[1] ;
}ble_ctlpro_frame_t ;

typedef struct{
	//uint16_t p_protype ;
	char opcode_h ;
	char opcode_l ;
	char flag ;
	char datlen_h ;
	char datlen_l ;
	char dat[1] ;
}ble_ctlpro_frameex_t ;

#define GET_OPCODE()	(((uint16_t )(ctlpro_inst->opcode_l) <<8) | ctlpro_inst->opcode_h)
#define GET_EXOPCODE()	(((uint16_t )(ctlpro_instex->opcode_l) <<8) | ctlpro_instex->opcode_h)
#define GET_EXLENGTH()	(((uint16_t )(ctlpro_instex->datlen_l) <<8) | ctlpro_instex->datlen_h)
#define BUILD_EXLENGTH(length)	{ctlpro_instex->datlen_l = (length >> 8) & 0xff  ; ctlpro_instex->datlen_h = length & 0xff ;}



#define 	B_Err_NOERR					0x00
#define 	B_Err_INSTR					0x01		//Execution failed or not doable
#define 	B_Err_OP						0x02		//Op-code not recognized
#define 	B_Err_FLAG					0x04		//Flag bits not recognized
#define 	B_Err_DATA					0x08	  	//Data values invalid
#define 	B_Err_Xor					     0x10	  	//Xor error

//control protocol command flag.
#define CP_CFLAG_NOACKCMD		0X01 				//don't need secondary device to response.
#define CP_CFLAG_EXLENGTH		0X40 				//extended command.
//control protocol error code.
#define CP_ECODE_NOERR  			(0X00)
#define CP_ECODE_ERR  			(0X01)  				//Execution failed or not doable
#define CP_ECODE_OPCODE     		(0X02 | CP_ECODE_ERR)  	//Op-code not recognized
#define CP_ECODE_FLAG   			(0X04 | CP_ECODE_ERR)  	//Flag bits not recognized
#define CP_ECODE_DATA   			(0X08 | CP_ECODE_ERR)  	//Data values invalid
#define CP_ECODE_TOOLALARM  		(0X10 | CP_ECODE_ERR)  

/*old*/
#define SET_NRL800HT_MODE				0x1000
#define SET_NRL800HT_ANGLE				0x1001
#define SET_MOTOR_ANGLE					0x1002
#define SET_MOTOR_SPEED					0x1003
#define SET_MOTOR_DIRECTION 			0x1004
#define REBOOT_NRL800HT					0x1005
#define READ_STATUS_PARAMS				0x1007
#define READ_PRODUCT_ID					0x1009
#define SET_STATUS_PARAMS				0x1008
#define SET_RCVMDL_MODE					0x2000
#define READ_RCVMDL_STATUS				0x2001
#define SET_SUB_DRIVCE					0x3000
#define READ_HUB_STATUS					0x3001
#define READ_SUB_ID						0x3003
#define BOOTLOADER_READ_VERSIONS		0x4001
#define BOOTLOADER_START_DOWNLOAD		0x4002
#define BOOTLOADER_SEND_HEX_FILE		0x4003
#define BOOTLOADER_COMPLETE_DOWNLOAD	0x4004
#define BOOTLOADER_GOTO_APP				0x4005

//for control protocol
#define ADD_LENGTH                                  4  /*2B opcode + 1B flag + 1B length*/
#define DEFAULT_RESP_RESULT_CODE                    0x80
#define RESP_DATA_VALUES_INVALID                    0x08


//control protocol opcode.
#define CP_OPCODE_GETPROTOCOLVER		0X0001//
#define CP_OPCODE_GETSUBDEVICEFWVER	0X0002//
#define CP_OPCODE_SETDEVICEID			0X0003//
#define CP_OPCODE_ENTERDFU			0X0004//
#define CP_OPCODE_ENTERAPPDOWNLOAD		0X0011//
#define CP_OPCODE_DOWNLOAD			0X0012//
#define CP_OPCODE_EXITAPPDOWNLOAD		0X0013//
#define CP_OPCODE_APPRECOVERY			0X0014//
#define CP_OPCODE_WRITECOMMTOKENS		0X0015//
#define CP_OPCODE_WRITEUSERID			0X0016//
#define CP_OPCODE_READNONCE			0X0017//
#define CP_OPCODE_CONNECTCOMM			0X0018//
#define CP_OPCODE_REMOVECOMM			0X0019//
#define CP_OPCODE_REMOVECOMM			0X0019//
#define CP_OPCODE_RESETDEVICE			0X001A//
#define CP_OPCODE_FACTORYRESET		0X001B//
#define CP_OPCODE_DISABLEDEVICE		0X001C//
#define CP_OPCODE_ENABLEDEVICE		0X001D//
#define CP_OPCODE_TESTDEVICELED		0X001E//
#define CP_OPCODE_SYNCDEVICETIME		0X001F//
#define CP_OPCODE_READDEVICESTATUS		0X0020//

//Below are in factory defines, won't offer to customer.
#define BCPC_SET_DEVICENAME                     	0x8004
#define BCPC_SET_PTI                         	0x8005
#define BCPC_SET_CPNCODE                        	0x8006	//BLE advertisement payload manufacture data- company code.
#define BCPC_SET_MODEL                        	0x8007	//BLE device information service- model number.	
#define BCPC_SET_SN                        		0x8008	//BLE device information service- serial number.
#define BCPC_RESET_BLE_INFO                       0x80FF	//RESET BLE device information.
//synchronize the current time
#define SYNCHRONIZE_TIME                       	0x001F	//synchronize the current time.

/*LRL*/
#define GET_LRL_STATUS                            0x0301
#define SET_WORKING_MODE                          0x0302
#define SET_SCANNING_RANGE                        0x0303
#define SET_ROTATING_SPEED                        0x0304
#define SET_ROTATING_DIRECTION                    0x0305
#define POWER_ON_OFF_LRL_DEVICE                   0x0306
#define READ_EVENT_NUMBER                         0x0310
#define READ_EVENT_HISTORY                        0x0311
#define DELETE_EVENT_HISTORY                      0x0312
#define CONFIRM_LAST_COMMITTED_EVENT              0x0313

/*HUB*/
#define GET_HUB_STATUS                              0x0401
#define GET_LRL_NAME_AND_DEVICE_ID                  0x0402
#define ESTABLISH_OR_DISCONNECT_HUB_CHANNEL_TO_LRL  0x0403
#define PAIR_LRL_TO_HUB                             0x0404
#define UNPAIR_LRL_FROM_HUB                         0x0405

#define GET_DEVICE_NAME_ID_FROM_SUB                 0x04C1


//common commands
#define OP_CODE_GET_PROTOCOL_VERSION             0x0001
#define OP_CODE_GET_SUB_DEVICE_FIRMWARE_VERSION  0x0002
#define OP_CODE_SET_DEVICE_ID                    0x0003
#define OP_CODE_TEST_DEVICE_LED                  0x001E
#define OP_CODE_READ_DEVICE_STATUS               0x0020
#define OP_CODE_WRITE_TOOL_PAIRED                0x0021
#define OP_CODE_READ_TOOL_PAIRED                 0x0022
#define OP_CODE_WRITE_TOOL_ENABLED               0x0023
#define OP_CODE_READ_TOOL_ENABLED                0x0024
#define OP_CODE_TOOL_IDENTIFY                    0x0025

//for ldm
#define OP_CODE_GET_LDM_STATUS                   0x0101
#define OP_CODE_TURN_LASER_ON                    0x0102
#define OP_CODE_REGULAR_MEASURE                  0x0103
#define OP_CODE_ENTER_EXIT_CONTINUOUS_MEASURE    0x0104
#define OP_CODE_CHANGE_MEASURING_START_POINT     0x0105
#define OP_CODE_CHANGE_MEASURING_UNIT_ON_LCD     0x0106

//for factory defines
#define BCPC_SET_LDM_BASE_CONST                  0x8010
#define BCPC_GET_LDM_BASE_CONST                  0x8011
#define BCPC_SET_LDM_ANG_PARAM1                  0x8012
#define BCPC_GET_LDM_ANG_PARAM1                  0x8013
#define BCPC_SET_LDM_ANG_PARAM2                  0x8014
#define BCPC_GET_LDM_ANG_PARAM2                  0x8015
#define BCPC_SET_LDM_FACTORY_CALIB               0x8016
#define BCPC_RESET_LDM_EEPROM                    0x8017



//void RF_channel_list(void);

//attHandleValueNoti_t * protocol_handler(attHandleValueNoti_t * ptr);

#endif

