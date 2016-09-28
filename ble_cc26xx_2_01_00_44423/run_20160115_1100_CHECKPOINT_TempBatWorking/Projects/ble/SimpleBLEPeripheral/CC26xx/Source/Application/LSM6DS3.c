#include "LSM6DS3.h"
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

