/*
Copyright (c) 2019 Teemu Kärkkäinen

ESP32 library for trigger-action programming
*/

/**
 * This header file defines the API for action drivers.
 */

#pragma once

#include "trigger_action.h"

/***********************************************************************/
/* Types */
/***********************************************************************/

typedef struct ActionDriver ActionDriver;

/**
 * Add an action definition to the driver.
 * 
 * @param state     The custom state object attached to the driver.
 * @param action    Action definition to add.
 */
typedef void ( *ActionDriver_AddAction )( void *state, action_definition_s *action );

/**
 * Remove a previously added action definition.
 * 
 * @param state     The custom state object attached to the driver.
 * @param action    Action definition to add.
 */
typedef void ( *ActionDriver_RemoveAction )( void *state, action_definition_s *action );

/**
 * Take the action specified by the identifier. Do nothing if the action has not been defined previously.
 * 
 * @param state     The custom state object attached to the driver.
 * @param actionId  Identifier of the action to take.
 */
typedef void ( *ActionDriver_TakeAction )( void *state, uint32_t actionId );

/**
 * Main function of the driver.
 * 
 * Will be called from an exclusive task and must never return.
 * 
 * @param state         The custom state object attached to the driver.
 * @param pulseMillis   The "pulse period", i.e., the time period between evaluating the conditions.
 */
typedef void ( *ActionDriver_TaskLoop )( void *state, uint32_t pulseMillis );

/**
 * Frees the memory used by the driverdriver.
 * 
 * @param driver    The driver instance whose memory to free.
 */
typedef void ( *ActionDriver_Free )( ActionDriver *driver );

struct ActionDriver {
    ActionDriver_AddAction addAction;
    ActionDriver_RemoveAction removeAction;
    ActionDriver_TakeAction takeAction;
    ActionDriver_TaskLoop taskLoop;
    ActionDriver_Free free;
    void *state;
};

/***********************************************************************/
