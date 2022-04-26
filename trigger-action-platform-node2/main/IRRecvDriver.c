/*
Copyright (c) 2019 Luna Fuchsloch

IR recv driver for use in trigger-action programming.
*/

/***********************************************************************/
/* Includes */
/***********************************************************************/
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <freertos/semphr.h>
#include <esp_log.h>

#include "IRRecvDriver.h"
#include "sdnv.h"
/***********************************************************************/


/***********************************************************************/
/* Definitions */
/***********************************************************************/
static const char* TAG = "IRRecvDriver";

#define IRRECV_CONDITION_LIST_SIZE   (64)

typedef uint8_t irrecv_event_type;
#define IRRECV_EVENT_ON   (0)
#define IRRECV_EVENT_OFF     (1)

typedef struct {
    trigger_definition_s *trigger;
    irrecv_event_type eventType;
} irrecv_trigger_s;

typedef struct {
    TriggerSignalCallback onTriggerSignal;
    uint8_t gpio;
    gpio_config_t *gpioConfig;

    irrecv_trigger_s* conditions[ IRRECV_CONDITION_LIST_SIZE ];
    uint32_t conditionCount;
    SemaphoreHandle_t conditionListMutex;
} irrecv_driver_s;

TriggerDriver* IRRecv_NewDriver( TriggerSignalCallback onTriggerSignal, uint8_t gpio );
void IRRecv_AddTrigger( void *state, trigger_definition_s const *condition );
void IRRecv_RemoveTrigger( void *state, trigger_definition_s const *condition );
void IRRecv_TaskLoop( void *state, uint32_t pulseMillis );
void IRRecv_Free( TriggerDriver *driver );
/***********************************************************************/


/***********************************************************************/
/* Private */
/***********************************************************************/

/***********************************************************************/


/***********************************************************************/
/* API implementation */
/***********************************************************************/
TriggerDriver* IRRecv_NewDriver( TriggerSignalCallback onTriggerSignal, uint8_t gpio ) {
    TriggerDriver *actionDriver = calloc( 1, sizeof( TriggerDriver ) );
    actionDriver->addTrigger = IRRecv_AddTrigger;
    actionDriver->removeTrigger = IRRecv_RemoveTrigger;
    actionDriver->taskLoop = IRRecv_TaskLoop;
    actionDriver->free = IRRecv_Free;

    irrecv_driver_s *irrecvDriver = calloc( 1, sizeof( irrecv_driver_s ) );
    irrecvDriver->onTriggerSignal = onTriggerSignal;
    irrecvDriver->gpio = gpio;
    irrecvDriver->gpioConfig = calloc( 1, sizeof( gpio_config_t ) );
    irrecvDriver->conditionListMutex = xSemaphoreCreateMutex();

    actionDriver->state = irrecvDriver;

    // Initialize GPIO
    irrecvDriver->gpioConfig->intr_type = GPIO_PIN_INTR_DISABLE;
    irrecvDriver->gpioConfig->mode = GPIO_MODE_INPUT;
    irrecvDriver->gpioConfig->pin_bit_mask = ( 1ULL << gpio );
    irrecvDriver->gpioConfig->pull_down_en = 0;
    irrecvDriver->gpioConfig->pull_up_en = 0;
    esp_err_t result = gpio_config( irrecvDriver->gpioConfig );
    if ( result != ESP_OK ) ESP_LOGI( TAG, "Failed to configure GPIO (%d)", result );

    return actionDriver;
}

void IRRecv_AddTrigger( void *state, trigger_definition_s const *condition ) {
    // Preconditions
    if ( condition->trigger_type != IRRECV_TRIGGER_TYPE || condition->params_length < 1 ) {
        return;
    }

    irrecv_driver_s *irrecvDriver = state;
    xSemaphoreTake( irrecvDriver->conditionListMutex, portMAX_DELAY );
    {
        // Get and check the driver
        if ( irrecvDriver->conditionCount >= IRRECV_CONDITION_LIST_SIZE ) {
            ESP_LOGI( TAG, "Condition list is full. Ignoring new trigger." );
        }

        // Parse parameters
        uint8_t eventType = *( condition->params );

        ESP_LOGI( TAG, "New trigger: eventType=%u", eventType );

		// Add the condition
		ESP_LOGI( TAG, "Adding new trigger: eventType=%u", eventType );
		irrecv_trigger_s *trigger = calloc( 1, sizeof( irrecv_trigger_s ) );
		trigger->eventType = eventType;
		trigger->trigger = ( trigger_definition_s* )condition;
		irrecvDriver->conditions[ irrecvDriver->conditionCount++ ] = trigger;
    }
    xSemaphoreGive( irrecvDriver->conditionListMutex );
}

void IRRecv_RemoveTrigger( void *state, trigger_definition_s const *trigger ) {
    irrecv_driver_s *irrecvDriver = state;
    xSemaphoreTake( irrecvDriver->conditionListMutex, portMAX_DELAY );
    {
        for ( int i = 0; i < irrecvDriver->conditionCount; i++ ) {
            irrecv_trigger_s *condition = irrecvDriver->conditions[ i ];
            if ( condition->trigger->trigger_id == trigger->trigger_id ) {
                irrecvDriver->conditionCount -= 1;
                irrecvDriver->conditions[ i ] = irrecvDriver->conditions[ irrecvDriver->conditionCount ];
                i--; // The moved item may be the same id
            }
        }
    }
    xSemaphoreGive( irrecvDriver->conditionListMutex );
}

void IRRecv_TaskLoop( void *state, uint32_t pulseMillis ) {
    // TODO: Should debounce more intelligently, but a 50ms delay seems to be good enough
    irrecv_driver_s *irrecvDriver = state;

    uint8_t previousLevel = gpio_get_level( irrecvDriver->gpio );
    while ( true ) {
        uint8_t level = gpio_get_level( irrecvDriver->gpio );
        if ( level != previousLevel ) {
            irrecv_event_type eventType = ( level == 0 ) ? IRRECV_EVENT_ON : IRRECV_EVENT_OFF;
            ESP_LOGI( TAG, "Event: %u", eventType );
            xSemaphoreTake( irrecvDriver->conditionListMutex, portMAX_DELAY );
            {
                for ( int i = 0; i < irrecvDriver->conditionCount; i++ ) {
                    irrecv_trigger_s *condition = irrecvDriver->conditions[ i ];
                    if ( condition->eventType == eventType ) {
                        ESP_LOGI( TAG, "Triggering signal %u", condition->trigger->trigger_id );
                        irrecvDriver->onTriggerSignal( condition->trigger->trigger_id );
                    }
                }
            }
            xSemaphoreGive( irrecvDriver->conditionListMutex );
        }
        previousLevel = level;
        vTaskDelay( 10 / portTICK_PERIOD_MS );
    }
}

void IRRecv_Free( TriggerDriver *driver ) {
    irrecv_driver_s *irrecvDriver = driver->state;
    free( irrecvDriver->gpioConfig );
    free( irrecvDriver );
    free( driver );
}
/***********************************************************************/
