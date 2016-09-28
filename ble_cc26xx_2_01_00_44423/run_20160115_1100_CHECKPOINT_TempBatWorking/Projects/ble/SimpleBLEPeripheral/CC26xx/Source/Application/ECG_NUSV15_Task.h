#include <ti/drivers/pin/PINCC26XX.h>
//#include "hal_sleep.h"
#include <ti/drivers/SPI.h>

#ifndef ECG_NUSV15__TASK_H_
#define BUTTON_TASK_H_

#define NUSV15_RSTFEA P0_1
#define NUSV15_CS     P0_4
#define NUSV15_RSTECGB   P1_4
#define RSTECGB_ENABLED   0
#define RSTECGB_DISABLED  1
#define RSTFEA_ENABLED   1
#define RSTFEA_DISABLED  0
#define CS_ENABLED   0
#define CS_DISABLED  1

#define NUSV15_ID_VAL     0x73
#define NUSV15_DELAY      0x01

#define NUSV15_125SPS     0x00
#define NUSV15_250SPS     0x01
#define NUSV15_500SPS     0x02
#define NUSV15_CONTINUOUS  0x00

//extern uint8_t accValue[6];

static SPI_Params SPI_WriteParams = {
    SPI_MODE_BLOCKING,  /* transferMode */
    SPI_WAIT_FOREVER,   /* transferTimeout */
    NULL,               /* transferCallbackFxn */
    SPI_MASTER,         /* mode */
    1500000,            /* bitRate */
    8,                  /* dataSize */
    SPI_POL1_PHA1,      /* frameFormat */ //HJ
    NULL                /* custom */
};

static SPI_Params SPI_ReadParams = {
    SPI_MODE_BLOCKING,  /* transferMode */
    SPI_WAIT_FOREVER,   /* transferTimeout */
    NULL,               /* transferCallbackFxn */
    SPI_MASTER,         /* mode */
    1500000,            /* bitRate */
    8,                  /* dataSize */
    SPI_POL0_PHA0,      /* frameFormat */ //HJ
    NULL                /* custom */
};

extern inline void ECG_NUSV15_Init(void);
extern void ECG_NUSV15_Init( void );
extern void ECG_NUSV15_Task_ProcessEvent( uint8 *pu8_NUSV15Val );
extern void ECG_NUSV15_InitialiseMem(uint8 *au8_configArray, PIN_Handle *hSbpPins);

extern void ecg_SPIInit(void);
extern void ecg_writeMode(SPI_Handle *SbpSpiHandle);
extern void ecg_readMode(SPI_Handle *SbpSpiHandle);
extern void ecg_resetAfe(PIN_Handle *hSbpPins);
#endif