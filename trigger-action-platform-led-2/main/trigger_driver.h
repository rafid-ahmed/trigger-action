/*
Copyright (c) 2019 Teemu Kärkkäinen

ESP32 library for trigger-action programming
*/

/**
 * This header file defines the API for trigger drivers.
 */

#pragma once

#include "trigger_action.h"

/***********************************************************************/
/* Types */
/***********************************************************************/

// Abstract Trigger Driver
// params: addTrigger, RemoveTrigger, taskLoop, free, state
typedef struct TriggerDriver TriggerDriver;

/**
 * Adds a trigger condition to the driver.
 * 
 * @param state     The custom state object attached to the driver.
 * @param condition     The trigger condition to add for evaluation.
 */
typedef void ( *Driver_AddTrigger )( void *state, trigger_definition_s const *condition );

/**
 * Removes a previously added trigger condition from the driver.
 * d
 * @param state     The custom state object attached to the driver.
 * @param condition     The trigger condition to remove.
 */
typedef void ( *Driver_RemoveTrigger )( void *state, trigger_definition_s const *condition );

/**
 * Main function of the driver.
 * 
 * Will be called from an exclusive task and must never return.
 * The driver should continuously evaluate its list of conditions and call into the platform
 * when conditions evaluate to true (using a TriggerSignalCallback passed to the function that
 * created the TriggerDriver instance).
 * 
 * @param state         The custom state object attached to the driver.
 * @param pulseMillis   The "pulse period", i.e., the time period between evaluating the conditions.
 */
typedef void ( *Driver_TaskLoop )( void *state, uint32_t pulseMillis );

/**
 * Frees the driver. Currently will never be called and does not need to be implemented (can be NULL).
 * 
 * @param driver    The driver to be freed.
 */
typedef void ( *Driver_Free )( TriggerDriver *driver );

struct TriggerDriver {
    Driver_AddTrigger addTrigger;           //Button_Addtrigger
    Driver_RemoveTrigger removeTrigger;     //Button_RemoveTrigger
    Driver_TaskLoop taskLoop;               //Button_TaskLoop
    Driver_Free free;                       //Button_Free
    void *state;                            //button_driver_s*
};

/***********************************************************************/
