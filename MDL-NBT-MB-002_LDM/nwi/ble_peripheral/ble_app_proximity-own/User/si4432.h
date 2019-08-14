#ifndef __SI4432_H_
#define __SI4432_H_

#include <stdint.h>
//#include "spi.h"

#define  DeviceType 							    	 0x00
#define  DeviceVersion								 0x01
#define  DeviceStatus 								 0x02
#define  InterruptStatus1 							 0x03
#define  InterruptStatus2 							 0x04
#define  InterruptEnable1 							 0x05          
#define  InterruptEnable2 							 0x06         
#define  OperatingFunctionControl1 				 0x07
#define  OperatingFunctionControl2 				 0x08	 
#define  CrystalOscillatorLoadCapacitance 		 0x09
#define  MicrocontrollerOutputClock 				 0x0A
#define  GPIO0Configuration 						 0x0B
#define  GPIO1Configuration 						 0x0C         
#define  GPIO2Configuration						     0x0D
#define  IOPortConfiguration						 0x0E
#define  ADCConfiguration							 0x0F
#define  ADCSensorAmplifierOffset					 0x10
#define  ADCValue									 0x11
#define  TemperatureSensorControl					 0x12
#define  TemperatureValueOffset					     0x13	 
#define  WakeUpTimerPeriod1 						 0x14          
#define  WakeUpTimerPeriod2 						 0x15         
#define  WakeUpTimerPeriod3 						 0x16         
#define  WakeUpTimerValue1							 0x17
#define  WakeUpTimerValue2							 0x18
#define  LowDutyCycleModeDuration 					 0x19       
#define  LowBatteryDetectorThreshold  				 0x1A
#define  BatteryVoltageLevel 						 0x1B                          
#define  IFFilterBandwidth  						 0x1C                           
#define  AFCLoopGearshiftOverride					 0x1D
#define  AFCTimingControl 							 0x1E                              
#define  ClockRecoveryGearshiftOverride 			 0x1F              
#define  ClockRecoveryOversamplingRatio 			 0x20              
#define  ClockRecoveryOffset2 						 0x21                       
#define  ClockRecoveryOffset1 						 0x22                       
#define  ClockRecoveryOffset0 						 0x23                     
#define  ClockRecoveryTimingLoopGain1 				 0x24              
#define  ClockRecoveryTimingLoopGain0 				 0x25             
#define  ReceivedSignalStrengthIndicator 			 0x26          
#define  RSSIThresholdForClearChannelIndicator 	     0x27   
#define  AntennaDiversityRegister1					 0x28
#define  AntennaDiversityRegister2					 0x29
#define  AFCLimiter								     0x2A
#define  DataAccessControl 						     0x30                          
#define  EZmacStatus 								 0x31                                  
#define  HeaderControl1 							 0x32                               
#define  HeaderControl2 							 0x33                              
#define  PreambleLength 							 0x34                               
#define  PreambleDetectionControl 				 0x35                    
#define  SyncWord3 								 0x36                                   
#define  SyncWord2 								 0x37                                   
#define  SyncWord1 								 0x38                               
#define  SyncWord0 								 0x39                                
#define  TransmitHeader3						 0x3A                       
#define  TransmitHeader2 						 0x3B                             
#define  TransmitHeader1 							 0x3C                              
#define  TransmitHeader0 							 0x3D                             
#define  TransmitPacketLength 						 0x3E                         
#define  CheckHeader3 								 0x3F                                
#define  CheckHeader2 								 0x40                              
#define  CheckHeader1 								 0x41                             
#define  CheckHeader0 								 0x42                            
#define  HeaderEnable3 							 0x43                               
#define  HeaderEnable2 							 0x44                                 
#define  HeaderEnable1 							 0x45                                
#define  HeaderEnable0 							 0x46                              
#define  ReceivedHeader3 							 0x47                          
#define  ReceivedHeader2 							 0x48                         
#define  ReceivedHeader1 							 0x49                           
#define  ReceivedHeader0 							 0x4A                             
#define  ReceivedPacketLength						 0x4B                       
#define  ChannelFilterCoefficientAddress 			 0x60                       
#define  CrystalOscillator_ControlTest 		         0x62               
#define  AGCOverride1					 			 0x69             
#define  TXPower 									 0x6D                                   
#define  TXDataRate1 								 0x6E                            
#define  TXDataRate0 								 0x6F                              
#define  ModulationModeControl1 					 0x70                   
#define  ModulationModeControl2 					 0x71                   
#define  FrequencyDeviation 						 0x72                            
#define  FrequencyOffset 							 0x73                            
#define  FrequencyChannelControl					 0x74
#define  FrequencyBandSelect 						 0x75                        
#define  NominalCarrierFrequency1	 				 0x76                    
#define  NominalCarrierFrequency0 					 0x77                    
#define  FrequencyHoppingChannelSelect 			     0x79               
#define  FrequencyHoppingStepSize 					 0x7A                     
#define  TXFIFOControl1 							 0x7C                        
#define  TXFIFOControl2 							 0x7D    
#define  RXFIFOControl 							 0x7E                               
#define  FIFOAccess								 0x7F

extern unsigned char RF_flag;

//void DelayMS(unsigned short msec);
void RFSetRfParameters(unsigned char setting);
void si4432_init(void);
void Set_frq(unsigned int frq);
void RFIdle(void);
void RFTransmit(unsigned char *packet, uint16_t length);
void RFReceive(void);
unsigned char RFPacketReceived(unsigned char * packet );


#endif

