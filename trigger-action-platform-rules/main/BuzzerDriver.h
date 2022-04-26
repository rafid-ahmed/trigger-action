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
/* API */
/***********************************************************************/

/**
 * Creates a new driver.
 * 
 * @param timer         Timer to use for driving the buzzer.
 * @param channel       Channel to use for the buzzer.
 * @param gpio          GPIO port to use for the buzzer.
 * @return Newly allocated driver.
 */
extern ActionDriver* Buzzer_NewDriver( ledc_timer_t timer, ledc_channel_t channel, uint8_t gpio );

/***********************************************************************/
