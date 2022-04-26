/*
Copyright (c) 2019 Teemu Kärkkäinen

Button driver for use in trigger-action programming.
*/

#pragma once

#include "trigger_action.h"
#include "trigger_driver.h"
#include "TAParser.h"


/***********************************************************************/
/* Trigger Driver API */
/***********************************************************************/
TriggerDriver* Button_NewDriver( TriggerSignalCallback onTriggerSignal, uint8_t gpio );
/***********************************************************************/
