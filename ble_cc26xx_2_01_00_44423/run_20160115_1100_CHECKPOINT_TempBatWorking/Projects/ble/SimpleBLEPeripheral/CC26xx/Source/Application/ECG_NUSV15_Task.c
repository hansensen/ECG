#include <string.h>
#include "OSAL.h"
#include "hal_mcu.h"
#include "_hal_sleep.h"

#include "bcomdef.h"
#include "simpleGATTprofile.h"
#include "hal_led.h"
#include "ECG_NUSV15_Task.h"
#include "gatt.h"
#include "Board.h"
#include "OnBoard.h"
//#include <ti/drivers/SPI.h>
//#include <ti/drivers/pin/PINCC26XX.h>
#include "simpleBLEPeripheral.h"
#include "hci.h"

void ECG_NUSV15_InitialiseMem(uint8 *au8_configArray, PIN_Handle *hSbpPins) {
  
  //au8_configArray= { 0xA5, /*Default of 0b10 @ bit 15 and bit 14*/\
//	            0xFF, /*Default of 0b10 @ bit 15 and bit 14*/\
	//                          0xE9, /*Byte 1: 0b11 @ bit 15 and bit 14 - Gain 1000, 256 Sampling Rate, High Bandwidth, DRL Enable*/
//	            0xE6, //odd, it is 300 gain/*0xfe0xByte 1: 0b11 @ bit 15 and bit 14 - Gain 1000, 512 Sampling Rate, High Bandwidth, DRL Enable*/
//	            0x60, /* 0x64 Byte 2: 0b11 @ bit 15 and bit 14 - Lead Off Detection Enable, Impedance Testing Resistive and Capacitive*/\
//						  0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint16 u16_delay = 0;
  uint8 u8_temp = 0xFF;
  
//  uint8_t au8_configArray[10] = { 0xA5, /*Default of 0b10 @ bit 15 and bit 14*/\
//            0xFF, /*Default of 0b10 @ bit 15 and bit 14*/\
//            0xC8,//0xEE, /*Byte 1: 0b11 @ bit 15 and bit 14 - Gain 1000, 256 Sampling Rate, High Bandwidth, DRL Enable*/
//            0x04, /*Byte 2: 0b11 @ bit 15 and bit 14 - Lead Off Detection Enable, Impedance Testing Resistive and Capacitive*/\
//            0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  
  au8_configArray[0] = 0xA5;
  au8_configArray[1] = 0xFF;
  au8_configArray[2] = 0xC8;//0xC8 will be 256Hz; 0xC0 will be 512Hz
  au8_configArray[3] = 0x64;
  au8_configArray[4] = 0x00;
  au8_configArray[5] = 0x00;
  au8_configArray[6] = 0x00;
  au8_configArray[7] = 0x00;
  au8_configArray[8] = 0x00;
  au8_configArray[9] = 0x00;
  
  u8_temp = PIN_setOutputValue(*hSbpPins, SS_ECG , CS_ENABLED); //HJ
//  SS_ECG = CS_ENABLED;
  for (u16_delay = 0; u16_delay < NUSV15_DELAY; u16_delay ++);
  write10Bytes_NUSV15( &(au8_configArray[0]) );  
  u8_temp = PIN_setOutputValue(*hSbpPins, SS_ECG , CS_DISABLED);
//  SS_ECG = CS_DISABLED;
}

extern void ecg_writeMode(SPI_Handle *SbpSpiHandle) {
  SPI_close(*SbpSpiHandle);
  SPI_Params_init(&SPI_WriteParams);
  SPI_WriteParams.bitRate  = 2000000;
  SPI_WriteParams.frameFormat = SPI_POL0_PHA1;
  /*
    SPI_POL0_PHA0   = 0,
    SPI_POL0_PHA1   = 1,
    SPI_POL1_PHA0   = 2,
    SPI_POL1_PHA1   = 3,
  */
  *SbpSpiHandle = SPI_open(CC2650_SPI0, &SPI_WriteParams);
}

extern void ecg_readMode(SPI_Handle *SbpSpiHandle) {
  SPI_close(*SbpSpiHandle);
  SPI_Params_init(&SPI_ReadParams);
  SPI_WriteParams.bitRate  = 130000;
  SPI_ReadParams.frameFormat = SPI_POL0_PHA0;
  *SbpSpiHandle = SPI_open(CC2650_SPI0, &SPI_ReadParams);
}

extern void ecg_resetAfe(PIN_Handle *hSbpPins) {
  //
  //RSTFEA
  PIN_setOutputValue(*hSbpPins, RST_AFE , RSTFEA_ENABLED);
  HCI_EXT_DelaySleepCmd(1000);
  //for (uint16 delay=0; delay<1000; delay++) {}
  //halSleep(15); // Wait 1m sec
  PIN_setOutputValue(*hSbpPins, RST_AFE , RSTFEA_DISABLED);
  HCI_EXT_DelaySleepCmd(1);
  //for (uint16 delay=0; delay<1000; delay++) {}
  //OSAL_SET_CPU_INTO_SLEEP(1); // Wait 1ms sec
  
  //RSTECG - Always pulled high
  //PIN_setOutputValue(*hSbpPins, RST_ECG , RSTECGB_DISABLED);
  PIN_setOutputValue(*hSbpPins, RST_ECG , RSTECGB_ENABLED);
  HCI_EXT_DelaySleepCmd(1000);
  //for (uint16 delay=0; delay<1000; delay++) {}
  //OSAL_SET_CPU_INTO_SLEEP(15); // Wait 1 sec
  //PIN_setOutputValue(*hSbpPins, RST_ECG , RSTECGB_DISABLED);
  HCI_EXT_DelaySleepCmd(1);
  //for (uint16 delay=0; delay<1000; delay++) {}
  //OSAL_SET_CPU_INTO_SLEEP(1); // Wait 1ms sec
  
  /* Original Code
    //RSTFEA
  NUSV15_RSTFEA = RSTFEA_ENABLED;
  OSAL_SET_CPU_INTO_SLEEP(15); // Wait 1 sec
  NUSV15_RSTFEA = RSTFEA_DISABLED;
  OSAL_SET_CPU_INTO_SLEEP(1); // Wait 1ms sec
  
  //RSTECG - Always pulled high
  NUSV15_RSTECGB = RSTECGB_DISABLED;
  NUSV15_RSTECGB = RSTECGB_ENABLED;
  OSAL_SET_CPU_INTO_SLEEP(15); // Wait 1 sec
  NUSV15_RSTECGB = RSTECGB_DISABLED;
  OSAL_SET_CPU_INTO_SLEEP(1); // Wait 1ms sec
  */
}

extern void ECG_NUSV15_Task_ProcessEvent( uint8 *pu8_NUSV15Val ) //DavidW
{
	//VOID task_id; // OSAL required parameter that isn't used in this function
  static uint8 charVal[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
  static uint8 u8_toggle = 0;
  static uint8 u8_toggleRr = 0;
  static uint8 u8_counter = 0;
  static uint8 u8_index12bits[] = {0,1,3,4,6};
  uint16 u16_manipulation1;
  uint16 u16_manipulation2;
  uint8 u8_isRealData;

//Initialisation SPI and ECGV15
  
  //ecg_SPIInit();
  //ECG_NUSV15_Init();

//#define DIVBYHALF
#ifdef DIVBYHALF
u8_myToggle = u8_myToggle ^ 1;
if ( u8_myToggle == 1) {
#endif //DIVBYHALF
    SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &u8_isRealData );
    
    for (u8_counter = 0; u8_counter < 5; u8_counter++) {
      pu8_NUSV15Val[u8_counter * 2] = (pu8_NUSV15Val[u8_counter * 2] + 8 ) & 0x0f;
    }
    
    switch (u8_isRealData) {
      case 0: //For verification
        u8_toggle++;
        if (u8_toggle >= 2<<0) {
          u8_toggle = 0;
          for (u8_counter = 0; u8_counter < 19; u8_counter++) {
            charVal[u8_counter]++;
          }
          SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, &charVal );
        }
        break;
        
      case 1: //1x 24-bit ECG[250Hz] RR[250Hz], 3x ACC[125Hz]
        //[0-1] Sequence Number
        //[1-3] Lead Status
        //[4-6] Reps
        //[7-9] Lead 2
        //[10-12] XYZ accelerometer
        //[13-15] Resp
        //[16-18] Lead 2
        //[20] 0x20 - End of Marker
        
        //[0-1] ECG CH-0
        //[2-3] ECG CH-1
        //[4-5] IMP Resistance
        //[6-7] ZERO
        //[8-9] IMP Reactance
        
        memcpy(&(charVal[1+(u8_toggle*9)]), &(pu8_NUSV15Val[0]), 9);
        u8_toggle++;
        if (u8_toggle >= 2<<0) {
          u8_toggle = 0;
          u8_toggleRr = 0;
          //Accel_ReadData(&(charVal[10])); // MMA8452Q Read 3D-XYZ
          SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 )*20, &charVal );
          charVal[0]++;
        }
        break;
        
      case 2: //1x 16-bit ECG[250Hz] RR[250Hz], 3x ACC[62.5Hz]
        memcpy(&(charVal[1+(u8_toggle*2)]), &(pu8_NUSV15Val[0]), 2); //ECG
        memcpy(&(charVal[9+(u8_toggle*2)]), &(pu8_NUSV15Val[2]), 2); //RR
        
        u8_toggle++;
        if (u8_toggle >= 4<<0) {
          u8_toggle = 0;
          u8_toggleRr = 0;
          //Accel_ReadData(&(charVal[17])); // MMA8452Q Read 3D-XYZ
          SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 )*20, &charVal );
          charVal[0]++;
        }
        break;
        
      case 3: //1x 16-bit ECG[250Hz] RR[83.33Hz], 3x ACC [41.67Hz]
        memcpy(&(charVal[1+(u8_toggle*2)]), &(pu8_NUSV15Val[0]), 2); //ECG
        u8_toggle++;
        
        if (u8_toggle == 0x03 || u8_toggle == 0x06) { //RR
          memcpy(&(charVal[13+(u8_toggleRr*2)]), &(pu8_NUSV15Val[4]), 2);
          u8_toggleRr++;
        }
        
        if (u8_toggle >= 6) {
          u8_toggle = 0;
          u8_toggleRr = 0;
          //Accel_ReadData(&(charVal[17])); // MMA8452Q Read 3D-XYZ
          SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 )*20, &charVal );
          charVal[0]++;
        }
        
        break;
      
      case 4: //1x 16-bit ECG[250Hz] 
        memcpy(&(charVal[1+(u8_toggle*2)]), &(pu8_NUSV15Val[0]), 2); //ECG
        u8_toggle++;
        
        if (u8_toggle >= 9) {
          u8_toggle = 0;
          u8_toggleRr = 0;
          SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 )*19, &charVal );
          charVal[0]++;
        }
        break;
        
      case 5: //16-bit ECG[250Hz] RR[250Hz]
        memcpy(&(charVal[1+(u8_toggle*2)]), &(pu8_NUSV15Val[0]), 2); //ECG
        memcpy(&(charVal[9+(u8_toggle*2)]), &(pu8_NUSV15Val[4]), 2); //RR
        u8_toggle++;
        
        if (u8_toggle >= 4) {
          u8_toggle = 0;
          u8_toggleRr = 0;
          SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 )*17, &charVal );
          charVal[0]++;
        }
        break;
        
      case 6: //2x 16-bit ECG[250Hz] RR[250Hz], 3x ACC[125Hz]
        memcpy(&(charVal[1+(u8_toggle*2)]), &(pu8_NUSV15Val[0]), 2); //ECG1
        memcpy(&(charVal[5+(u8_toggle*2)]), &(pu8_NUSV15Val[6]), 2); //ECG2
        memcpy(&(charVal[9+(u8_toggle*2)]), &(pu8_NUSV15Val[2]), 2); //RR1
        memcpy(&(charVal[13+(u8_toggle*2)]), &(pu8_NUSV15Val[4]), 2); //RR2
        u8_toggle++;
        
        if (u8_toggle >= 2) {
          u8_toggle = 0;
          u8_toggleRr = 0;
          //Accel_ReadData(&(charVal[17])); // MMA8452Q Read 3D-XYZ
          {
          uint8_t accValue[3];
          LSM6DS3_Read_Acc(&(accValue[0]));
          charVal[17] = accValue[0];
          charVal[18] = accValue[1];
          charVal[19] = accValue[2];
          }
          SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 )*20, &charVal );
          charVal[0]++;
        }
        break;
        
      case 7: //2x 16-bit ECG[250Hz] 1x RR[83.33Hz], 3x ACC [83.33Hz]
        memcpy(&(charVal[1+(u8_toggle*2)]), &(pu8_NUSV15Val[0]), 2); //ECG
        memcpy(&(charVal[7+(u8_toggle*2)]), &(pu8_NUSV15Val[6]), 2); //ECG
        u8_toggle++;
        
        if (u8_toggle >= 3) {
          u8_toggle = 0;
          u8_toggleRr = 0;
          memcpy(&(charVal[13]), &(pu8_NUSV15Val[4]), 2); //RR
          //Accel_ReadData(&(charVal[15])); // MMA8452Q Read 3D-XYZ
          SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 )*20, &charVal );
          charVal[0]++;
        }
        break;
        
      case 8: //2x 12-bit ECG[250Hz] 1x 16-bit RR[50.0Hz]

        if (u8_toggle == 0) { 
          memset(&(charVal[1]), 0, 19);
        } 
        //index - 1, 2.5, 4, 5.5, 7 
        //index - 9,10.5,12,13.5,15
        
        if (u8_toggle & 0x01) { //odd
          memcpy(&u16_manipulation1, &(pu8_NUSV15Val[0]), 2); //ECG1
          //u16_manipulation1 = 0xABCD;
          u16_manipulation2 = (u16_manipulation1 >> 0) & 0x00FF;
          u16_manipulation1 = (u16_manipulation1 >> 8) & 0x000F;
          charVal[1+(u8_index12bits[u8_toggle]) + 0]  |= (uint8)u16_manipulation1;
          charVal[1+(u8_index12bits[u8_toggle]) + 1] = (uint8)u16_manipulation2;
          
          memcpy(&u16_manipulation1, &(pu8_NUSV15Val[6]), 2); //ECG2
          //u16_manipulation1 = 0xCDEF;
          u16_manipulation2 = (u16_manipulation1 >> 0) & 0x00FF;
          u16_manipulation1 = (u16_manipulation1 >> 8) & 0x000F;
          charVal[9+(u8_index12bits[u8_toggle]) + 0] |= (uint8)u16_manipulation1;
          charVal[9+(u8_index12bits[u8_toggle]) + 1] = (uint8)u16_manipulation2;
        } else { //even
          memcpy(&u16_manipulation1, &(pu8_NUSV15Val[0]), 2); //ECG1
          //u16_manipulation1 = 0x1234;
          u16_manipulation2 = (u16_manipulation1 >> 4) & 0x00FF;
          u16_manipulation1 = (u16_manipulation1 << 4) & 0x00F0;
          charVal[1+(u8_index12bits[u8_toggle]) + 0] = (uint8)u16_manipulation2;
          charVal[1+(u8_index12bits[u8_toggle]) + 1] = (uint8)u16_manipulation1;
          
          memcpy(&u16_manipulation1, &(pu8_NUSV15Val[6]), 2); //ECG2
          //u16_manipulation1 = 0x5678;
          u16_manipulation2 = (u16_manipulation1 >> 4) & 0x00FF;
          u16_manipulation1 = (u16_manipulation1 << 4) & 0x00F0;
          charVal[9+(u8_index12bits[u8_toggle]) + 0] = (uint8)u16_manipulation2;
          charVal[9+(u8_index12bits[u8_toggle]) + 1] = (uint8)u16_manipulation1;
        }
        
        u8_toggle++;
        
        if (u8_toggle >= 5) {
          u8_toggle = 0;
          u8_toggleRr = 0;
          memcpy(&(charVal[17]), &(pu8_NUSV15Val[4]), 2); //RR
          SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 )*19, &charVal );
          charVal[0]++;
        }
        break;
        
      default:
        break;
    }
#ifdef DIVBYHALF
}
#endif //DIVBYHALF
    return;
}
