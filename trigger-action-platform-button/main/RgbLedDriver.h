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
 * @param timer         Timer to use for driving the LEDs.
 * @param redChannel    Channel to use for the red LED.
 * @param redGpio       GPIO port to use for the red LED.
 * @param greenChannel  Channel to use for the green LED.
 * @param greenGpio     GPIO port to use for the green LED.
 * @param blueChannel   Channel to use for the blue LED.
 * @param blueGpio      GPIO port to use for the blue LED.
 * @return Newly allocated driver.
 */
ActionDriver* RgbLed_NewDriver( ledc_timer_t timer,
        ledc_channel_t redChannel, uint8_t redGpio,
        ledc_channel_t greenChannel, uint8_t greenGpio,
        ledc_channel_t blueChannel, uint8_t blueGpio );

/***********************************************************************/