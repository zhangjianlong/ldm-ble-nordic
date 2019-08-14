#include <string.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "system_nrf52.h"
//#include "app_uart.h"
#include "boards.h"
#include "si4432.h"
#include "common.h"

#define MAX_RF_RX_LEN  256  //2048  //max data buffer length

extern unsigned char irq_flag;
/******************************************************************************/
unsigned char RF_flag;		  
unsigned char RF_Rate_num = 1;

#if 0
unsigned char RF_BAND = 0x53;
unsigned char CAR_FREQ1 = 0x4B;
unsigned char CAR_FREQ2 = 0x00;
#else
unsigned char RF_BAND = 0x75;
unsigned char CAR_FREQ1 = 0xBB;
unsigned char CAR_FREQ2 = 0x80;
#endif

unsigned char g_rf_chnl_nbr = 0;  //rf channel number, 0, 1, 2, 3
unsigned char RF_Power_DB = 7;  //0=11db,1=14db,2=17db,3=20db
unsigned char  RF_Fre_Band = 1;
uint8_t g_is_rf_connected = 0;  //for RF HUB, ALL_OPEN mode

/******************************************************************************/
unsigned char baud[6][13] =
{
// IFBW, COSR, CRO2, CRO1, CRO0, CTG1, CTG0, TDR1, TDR0, MMC1, FDEV, AFC ,AFCLimiter 
	{0x1B, 0x83, 0xC0, 0x13, 0xA9, 0x00, 0x03, 0x09, 0xD5, 0x2D, 0x3A, 0x40,0x1E},  //0 DR: 1.2kbps,  DEV: +-36kHz,  BBBW: 77.1kHz
	{0x1B, 0x41, 0x60, 0x27, 0x52, 0x00, 0x07, 0x13, 0xA9, 0x2D, 0x3A, 0x40,0x1E},  //1 DR: 2.4kbps,  DEV: +-36kHz,  BBBW: 77.1kHz
	{0x1B, 0xA1, 0x20, 0x4E, 0xA5, 0x00, 0x17, 0x27, 0x52, 0x2D, 0x3A, 0x40,0x1E},	//2 DR: 4.8kbps,  DEV: +-36kHz,  BBBW: 77.1kHz
	{0x1C, 0xD0, 0x00, 0x9D, 0x49, 0x00, 0x56, 0x4e, 0xA5, 0x2D, 0x3A, 0x40,0x1E},	//3 DR: 9.6kbps,  DEV: +-36kHz,  BBBW: 85.1kHz
	{0x16, 0x68, 0x01, 0x3A, 0x93, 0x02, 0x5F, 0x9D, 0x49, 0x2D, 0x20, 0x40,0x1E},	//4 DR: 19.2kbps, DEV: +-20kHz,  BBBW: 64.1kHz
	{0x02, 0x68, 0x01, 0x3A, 0x93, 0x04, 0xBC, 0x09, 0xD5, 0x0D, 0x20, 0x40,0x1E},	//5 DR: 38.4kbps, DEV: +-20kHz,  BBBW: 83.2kHz
};
/******************************************************************************/

/******************************************************************************/
//void DelayMS(unsigned short msec)
//{ 
//    unsigned short i,j;
//    
//    for (i=0; i<msec; i++)
//        for (j=0; j<536; j++);
//}
/******************************************************************************/

/******************************************************************************/
void si4432_init(void)
{ 
	unsigned char temp8;
	int8_t read_count;

	//check si4432...
	read_count = 5;
	do
	{
		nrf_delay_ms(15);
		temp8 = SpiRfReadRegister(DeviceType);
		if((temp8 == 0x07) || (temp8 == 0x08))
		{
			break;
		}
		read_count--;
	}while(read_count > 0);
	if((temp8 != 0x07) && (temp8 != 0x08))
	{//error, ack is wrong!
//		while(1);
	}
	//read interrupt status registers
	SpiRfReadRegister( InterruptStatus1 );
	SpiRfReadRegister( InterruptStatus2 );	

	//SW reset 
	SpiWriteAddressData ( OperatingFunctionControl1, 0x80 );
	while(nrf_gpio_pin_read(SI4432_NIRQ) == 1);

	//read interrupt status registers to clear the interrupt flags and release NIRQ pin
	//SpiRfReadRegister( InterruptStatus1 );
	//SpiRfReadRegister( InterruptStatus2 );
	//temp8 = SpiRfReadRegister( DeviceVersion );
	//temp8 = temp8 ;

	//wait for chip ready interrupt from the radio
//	while ( RF_NIRQ_R );  
	//read interrupt status registers to clear the interrupt flags and release NIRQ pin
//	SpiRfReadRegister( InterruptStatus1 );
//	SpiRfReadRegister( InterruptStatus2 );

	//set frequency band and frequency
//	Set_frq(915000);
	SpiWriteAddressData ( FrequencyBandSelect, RF_BAND);   
	SpiWriteAddressData ( NominalCarrierFrequency1,CAR_FREQ1);
	SpiWriteAddressData ( NominalCarrierFrequency0,CAR_FREQ2);  
	SpiWriteAddressData ( FrequencyHoppingStepSize, 0x0A );

//	SpiWriteAddressData ( FrequencyOffset,0);
//	SpiWriteAddressData ( FrequencyChannelControl,RF_OFFSET);
	//set  RF_Rate
	RFSetRfParameters( RF_Rate_num );

	//set preamble length & detection threshold
    //SpiWriteAddressData ( PreambleLength, 0x0A );
	SpiWriteAddressData ( PreambleDetectionControl, 0x20 );

	//disable RX-TX headers,
	SpiWriteAddressData ( HeaderControl1, 0x00 );   
	SpiWriteAddressData ( HeaderControl2, 0x22 );    //0x02

	//set synchron word
	SpiWriteAddressData ( SyncWord3, 0x2D );   
	SpiWriteAddressData ( SyncWord2, 0xD4 );  

	//enable packet handler and CRC-16
	SpiWriteAddressData ( DataAccessControl, 0x8D ); 
	
	//set GFSK modulation and FIFO mode
	SpiWriteAddressData ( ModulationModeControl2,0x23 );   //0x23
	
	//set the AGC
	SpiWriteAddressData ( AGCOverride1, 0x60 );

	//set GPIO0 & GPIO1 to control the TRX switch    
    SpiWriteAddressData ( GPIO0Configuration, 0x1E );
    SpiWriteAddressData ( GPIO1Configuration, 0x15 ); 
	SpiWriteAddressData ( GPIO2Configuration, 0x12 ); 
	//set cap. bank to cancel the frequency offset from the center frequency
	SpiWriteAddressData ( CrystalOscillatorLoadCapacitance, 0xB3);	   //0xB3

	// Set TXPower
	SpiWriteAddressData ( TXPower, 0x18|RF_Power_DB );

	//Set Channel
	SpiWriteAddressData ( FrequencyHoppingChannelSelect, g_rf_chnl_nbr );
   	
	//finish init
	RF_flag |= 0x04;
}  
/******************************************************************************/

/******************************************************************************/
void RFSetRfParameters(unsigned char setting)
{
	//the modem parameters are read from the table
	SpiWriteAddressData	(IFFilterBandwidth, baud[setting][0] );
	SpiWriteAddressData	(ClockRecoveryOversamplingRatio,baud[setting][1] );
	SpiWriteAddressData ( ClockRecoveryOffset2, baud[setting][2] );   // Clock Recovery Offset 2
	SpiWriteAddressData ( ClockRecoveryOffset1, baud[setting][3] );   // Clock Recovery Offset 1
	SpiWriteAddressData ( ClockRecoveryOffset0, baud[setting][4] );   // Clock Recovery Offset 0
	SpiWriteAddressData ( ClockRecoveryTimingLoopGain1, baud[setting][5] );   // Clock Recovery Timing Loop Gain 1
	SpiWriteAddressData ( ClockRecoveryTimingLoopGain0, baud[setting][6] );   // Clock Recovery Timing Loop Gain 0
	SpiWriteAddressData ( TXDataRate1, baud[setting][7] );   // Block Enable Override 2
	SpiWriteAddressData ( TXDataRate0, baud[setting][8] );   // Block Enable Override 2
	SpiWriteAddressData ( ModulationModeControl1, baud[setting][9] );	 
	SpiWriteAddressData ( FrequencyDeviation, baud[setting][10] );   // Frequency Deviation Setting
	SpiWriteAddressData ( AFCLoopGearshiftOverride,baud[setting][11] );
	SpiWriteAddressData ( AFCTimingControl, 0x0A );
	SpiWriteAddressData ( AFCLimiter, baud[setting][12] );
}
/******************************************************************************/
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  + FUNCTION NAME:  Set_frq(unsigned int frq)  +
  + DESCRIPTION:    set the TX frequency     +
  + INPUT:			the frequency  
  + RETURN:         None
  + NOTES:          
  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void Set_frq(unsigned int frq)
{
	unsigned int  temp;
	unsigned char fb;
	unsigned char fc_h;
	unsigned char fc_l;
	switch(RF_Fre_Band)
	{
		case 0:
			fb = 0x40|((unsigned char)(frq/10000)+16);
			break;
		case 1:
			fb = 0x40|((unsigned char)(frq/10000)+20);
			break;
		case 2:
			fb = 0x60|((unsigned char)(frq/10000)+18);
			break;		
		default:
			fb = 0x40|((unsigned char)(frq/10000)+16);
			break;
	}
	temp =  (frq%10000)/10*64;
	fc_h = temp >> 8;
	fc_l = temp & 0x00ff;
	
	//set frequency
	SpiWriteAddressData ( FrequencyBandSelect, fb );
	SpiWriteAddressData ( NominalCarrierFrequency1,fc_h);
	SpiWriteAddressData ( NominalCarrierFrequency0,fc_l);
	SpiWriteAddressData ( FrequencyHoppingStepSize, 0x0A );
}
/******************************************************************************/


/******************************************************************************/
void RFIdle(void)
{
	SpiWriteAddressData( OperatingFunctionControl1, 0x01 );

	//diasble all ITs
	SpiWriteAddressData ( InterruptEnable1, 0x00 );
	SpiWriteAddressData( InterruptEnable2, 0x00 );
   //releaze all IT flags
	SpiRfReadRegister ( InterruptStatus1 );
	SpiRfReadRegister ( InterruptStatus2 );

}
/******************************************************************************/

/******************************************************************************/
void RFTransmit(unsigned char *packet, uint16_t length)
{
//	unsigned char ItStatus1,ItStatus2;
	unsigned int i;
	unsigned int timeout = 0;
	
	//clear recv flag!
//	RF_flag &= ~0x08;
	//
	static uint16_t dbg_cnt2 = 0;
	if((packet[1] == 0x05/*g_rf_buffer.recv_data[2]*/) \
	&& (packet[2] == 0x05/*g_rf_buffer.recv_data[3]*/) \
	&& (packet[3] == 0x05/*g_rf_buffer.recv_data[4]*/))
	{
		dbg_cnt2++;
	}
	
	//disable the receiver chain (but keep the XTAL running to have shorter TX on time!)
	SpiWriteAddressData ( OperatingFunctionControl1, 0x01 );
	//clear TX FIFO
	SpiWriteAddressData ( OperatingFunctionControl2, 0x01 );   
	SpiWriteAddressData ( OperatingFunctionControl2, 0x00 );
	//clear RX FIFO
	SpiWriteAddressData(OperatingFunctionControl2, 0x02);
	SpiWriteAddressData(OperatingFunctionControl2, 0x00);

	//set packet length
	SpiWriteAddressData ( TransmitPacketLength, length );	   
	for(i=0; i<length; i++)
	{
		SpiWriteAddressData ( FIFOAccess, packet[i] ); 
	}

	//enable the packet sent interrupt
	SpiWriteAddressData ( InterruptEnable1, 0x04 );
	SpiWriteAddressData ( InterruptEnable2, 0x00 );

	//releaze all IT flags
	SpiRfReadRegister( InterruptStatus1 );
	SpiRfReadRegister( InterruptStatus2 );

	//enable transmitter
	SpiWriteAddressData ( OperatingFunctionControl1, 0x09 );	
	
	//wait for the packet sent interrupt
	while((nrf_gpio_pin_read(SI4432_NIRQ) == 1) && (timeout < 6000))
	{timeout++;}

//	while((RF_flag & 0x05) == 0x05)
//	{RF_flag_clearRx;}

	SpiRfReadRegister( InterruptStatus1 );
	SpiRfReadRegister( InterruptStatus2 );
	
//	//
//	SpiWriteAddressData ( InterruptEnable1, 0x03 );
	//RFReceive();
}
/******************************************************************************/

/******************************************************************************/
void RFReceive(void)
{
	//unsigned char ItStatus1,ItStatus2;

	//clear recive FIFO
	SpiWriteAddressData ( OperatingFunctionControl2, 0x02 );  
	SpiWriteAddressData ( OperatingFunctionControl2, 0x00 );

	//enable the packet valid interrupt
	SpiWriteAddressData ( InterruptEnable1, 0x03 );	
	SpiWriteAddressData ( InterruptEnable2, 0x00 );

	//releaze all IT flags
	SpiRfReadRegister( InterruptStatus1 );
	SpiRfReadRegister( InterruptStatus2 );

	//enable receiver chain
	SpiWriteAddressData ( OperatingFunctionControl1, 0x05 );

	RF_flag |= 0x08;
}
/******************************************************************************/

/******************************************************************************/
unsigned char RFPacketReceived(unsigned char * packet)
{
	unsigned char ItStatus1;
	uint16_t length; 
	uint16_t i;
	//interrupt occured, read the IT status bits
	ItStatus1 = SpiRfReadRegister ( InterruptStatus1 );
	SpiRfReadRegister ( InterruptStatus2 );	
	if( (ItStatus1 & 0x01) == 0x01 )
	{
		//clear recive FIFO
		SpiWriteAddressData ( OperatingFunctionControl2, 0x02 );  
		SpiWriteAddressData ( OperatingFunctionControl2, 0x00 );
		return 0;
	}
	if( (ItStatus1 & 0x02) == 0x02 )
	{//packet received
		//read buffer
		length = SpiRfReadRegister ( ReceivedPacketLength );
		if(length > MAX_RF_RX_LEN)
		{
			length = MAX_RF_RX_LEN;
		}
		for(i = 0; i < length; i++)
		{
			*packet++ =  SpiRfReadRegister ( FIFOAccess );
		}
		//clear recive FIFO
		SpiWriteAddressData ( OperatingFunctionControl2, 0x02 );   
		SpiWriteAddressData ( OperatingFunctionControl2, 0x00 );
		SpiWriteAddressData ( OperatingFunctionControl1, 0x05 );
		return length;
	}
	return 0;
}
/******************************************************************************/
