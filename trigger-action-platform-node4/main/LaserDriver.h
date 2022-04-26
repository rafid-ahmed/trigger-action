/*
Copyright (c) 2019 Teemu Kärkkäinen

ESP32 library for trigger-action programming
*/

#pragma once

#include <freertos/FreeRTOS.h>
#include <driver/ledc.h>
#include <stdint.h>

#include "action_driver.h"

/***********************************************************************/
/* Functions */
/***********************************************************************/

/**
 * Creates a new driver.
 *
 * @param gpio       GPIO port to use for the laser's sig pin.
 * @return Newly allocated driver.
 */
ActionDriver* Laser_NewDriver( uint8_t gpio );

/***********************************************************************/
