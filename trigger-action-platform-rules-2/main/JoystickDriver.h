/*
Copyright (c) 2019 Teemu Kärkkäinen

Joystick driver for use in trigger-action programming.
*/

#pragma once

#include "trigger_action.h"
#include "trigger_driver.h"
#include "TAParser.h"

#include <driver/adc.h>


/***********************************************************************/
/* Trigger Driver API */
/***********************************************************************/
TriggerDriver* Joystick_NewDriver( TriggerSignalCallback onTriggerSignal, uint8_t gpio, adc1_channel_t xADC, adc1_channel_t yADC);
/***********************************************************************/
