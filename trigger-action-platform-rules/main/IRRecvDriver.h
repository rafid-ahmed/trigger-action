/*
Copyright (c) 2019 Luna Fuchsloch

IR reciever driver for use in trigger-action programming.
*/

#pragma once

#include "trigger_action.h"
#include "trigger_driver.h"
#include "TAParser.h"


/***********************************************************************/
/* Trigger Driver API */
/***********************************************************************/
TriggerDriver* IRRecv_NewDriver( TriggerSignalCallback onTriggerSignal, uint8_t gpio);
/***********************************************************************/
