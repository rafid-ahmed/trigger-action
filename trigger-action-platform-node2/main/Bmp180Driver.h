/*
Copyright (c) 2019 Teemu Kärkkäinen

Driver for the BMP180 sensor for use in trigger-action programming.
*/

#pragma once

#include <driver/i2c.h>

#include "TAParser.h"
#include "trigger_action.h"
#include "trigger_driver.h"

// Guard against missing include by user.
// Causes linking to fail if the header is not included.
#define Bmp180dr_NewDriver  bmp180dr_new_driver


/***********************************************************************/
/* Values */
/***********************************************************************/
#define BMP180DR_CONDITION_LIST_SIZE    (1024)

#define BMP180DR_CONDITION_OP_ERROR (0xFF)
#define BMP180DR_CONDITION_OP_GT    (0x01)
#define BMP180DR_CONDITION_OP_LT    (0x02)
#define BMP180DR_CONDITION_OP_RG    (0x05)
/***********************************************************************/


/***********************************************************************/
/* Types */
/***********************************************************************/
typedef struct bmp_180_driver Bmp180Driver;
/***********************************************************************/


/***********************************************************************/
/* Functions */
/***********************************************************************/

/**
 * Creates a new driver.
 * 
 * @param onTriggerSignal is a callback that the driver will invoke when a trigger condition becomes true. Must be thread safe.
 * @param i2cPort is the I2C to which the BMP180 is connected
 * @return Newly allocated driver.
 */
TriggerDriver* Bmp180dr_NewDriver( TriggerSignalCallback onTriggerSignal, i2c_port_t i2cPort );

/***********************************************************************/
