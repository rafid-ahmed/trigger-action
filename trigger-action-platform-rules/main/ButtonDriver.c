/*
Copyright (c) 2019 Teemu Kärkkäinen

Button driver for use in trigger-action programming.
*/

/***********************************************************************/
/* Includes */
/***********************************************************************/
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <freertos/semphr.h>
#include <esp_log.h>

#include "ButtonDriver.h"
#include "sdnv.h"
/***********************************************************************/


/***********************************************************************/
/* Definitions */
/***********************************************************************/
static const char* TAG = "ButtonDriver";

#define BUTTON_CONDITION_LIST_SIZE   (64)

typedef uint8_t button_event_type;
#define BUTTON_EVENT_DOWN   (0)
#define BUTTON_EVENT_UP     (1)

// button trigger state
typedef struct {
    trigger_definition_s *trigger;      // trigger definition state
    button_event_type eventType;        // button event (0 - down, 1 - up)
} button_trigger_s;

// button driver state
typedef struct {
    TriggerSignalCallback onTriggerSignal;                          // trigger signal callback (defined in main.c)
    uint8_t gpio;                                                   // GPIO of the button
    uint32_t buttonId;                                              // 
    gpio_config_t *gpioConfig;                                      // GPIO config info

    button_trigger_s* conditions[ BUTTON_CONDITION_LIST_SIZE ];     // list of button trigger states/conditions
    uint32_t conditionCount;                                        // number of button trigger states/conditions
    SemaphoreHandle_t conditionListMutex;                           // mutex for button trigger states/conditions list
} button_driver_s;

TriggerDriver* Button_NewDriver( TriggerSignalCallback onTriggerSignal, uint8_t gpio );
void Button_AddTrigger( void *state, trigger_definition_s const *condition );
void Button_RemoveTrigger( void *state, trigger_definition_s const *condition );
void Button_TaskLoop( void *state, uint32_t pulseMillis );
void Button_Free( TriggerDriver *driver );
/***********************************************************************/


/***********************************************************************/
/* Private */
/***********************************************************************/

/***********************************************************************/


/***********************************************************************/
/* API implementation */
/***********************************************************************/
TriggerDriver* Button_NewDriver( TriggerSignalCallback onTriggerSignal, uint8_t gpio ) {
    TriggerDriver *triggerDriver = calloc( 1, sizeof( TriggerDriver ) );
    triggerDriver->addTrigger = Button_AddTrigger;
    triggerDriver->removeTrigger = Button_RemoveTrigger;
    triggerDriver->taskLoop = Button_TaskLoop;
    triggerDriver->free = Button_Free;

    button_driver_s *buttonDriver = calloc( 1, sizeof( button_driver_s ) );
    buttonDriver->onTriggerSignal = onTriggerSignal;
    buttonDriver->gpio = gpio;
    buttonDriver->gpioConfig = calloc( 1, sizeof( gpio_config_t ) );
    buttonDriver->conditionListMutex = xSemaphoreCreateMutex();

    triggerDriver->state = buttonDriver;

    // Initialize GPIO
    buttonDriver->gpioConfig->intr_type = GPIO_PIN_INTR_DISABLE;
    buttonDriver->gpioConfig->mode = GPIO_MODE_INPUT;
    buttonDriver->gpioConfig->pin_bit_mask = ( 1ULL << gpio );
    buttonDriver->gpioConfig->pull_down_en = 0;
    buttonDriver->gpioConfig->pull_up_en = 0;
    esp_err_t result = gpio_config( buttonDriver->gpioConfig );
    if ( result != ESP_OK ) 
        ESP_LOGI( TAG, "Failed to configure GPIO (%d)", result );

    return triggerDriver;
}

void Button_AddTrigger( void *state, trigger_definition_s const *condition ) {
    // Preconditions
    if ( condition->trigger_type != BUTTON_TRIGGER_TYPE || condition->params_length != 1 ) {
        return;
    }

    button_driver_s *buttonDriver = state;
    xSemaphoreTake( buttonDriver->conditionListMutex, portMAX_DELAY );
    {
        // Get and check the driver
        if ( buttonDriver->conditionCount >= BUTTON_CONDITION_LIST_SIZE ) {
            ESP_LOGI( TAG, "Condition list is full. Ignoring new trigger." );
        }

        // Parse parameters
        uint8_t eventType = *( condition->params );     // 1 byte of data for button (0 - down, 1 - up)

        #if (CONFIG_TA_DEBUG_GENERAL_INFO)
        ESP_LOGI( TAG, "New trigger: eventType=%u", eventType );
        #endif

        // Only match trigger definitions for this button id, unless the button id for this driver is 0 (match-all)
//        if ( buttonDriver->buttonId == 0 || buttonId.value.u64 == buttonDriver->buttonId ) {
            // Add the condition
//            ESP_LOGI( TAG, "Adding new trigger: id=%llu, eventType=%u", buttonId.value.u64, eventType );
            button_trigger_s *trigger = calloc( 1, sizeof( button_trigger_s ) );
//            trigger->buttonId = buttonId.value.u64;
            trigger->eventType = eventType;
            trigger->trigger = ( trigger_definition_s* )condition;
            buttonDriver->conditions[ buttonDriver->conditionCount++ ] = trigger;
//        }
    }
    xSemaphoreGive( buttonDriver->conditionListMutex );
}

void Button_RemoveTrigger( void *state, trigger_definition_s const *trigger ) {
    button_driver_s *buttonDriver = state;
    xSemaphoreTake( buttonDriver->conditionListMutex, portMAX_DELAY );
    {
        for ( int i = 0; i < buttonDriver->conditionCount; i++ ) {
            button_trigger_s *condition = buttonDriver->conditions[ i ];
            if ( condition->trigger->trigger_id == trigger->trigger_id ) {
                buttonDriver->conditionCount -= 1;
                buttonDriver->conditions[ i ] = buttonDriver->conditions[ buttonDriver->conditionCount ];
                i--; // The moved item may be the same id
            }
        }
    }
    xSemaphoreGive( buttonDriver->conditionListMutex );
}

void Button_TaskLoop( void *state, uint32_t pulseMillis ) {
    // TODO: Should debounce more intelligently, but a 50ms delay seems to be good enough
    button_driver_s *buttonDriver = state;

    uint8_t previousLevel = gpio_get_level( buttonDriver->gpio );
    while ( true ) {
        uint8_t level = gpio_get_level( buttonDriver->gpio );
        if ( level != previousLevel ) {
            button_event_type eventType = ( level == 0 ) ? BUTTON_EVENT_DOWN : BUTTON_EVENT_UP;
            #if (CONFIG_TA_DEBUG_GENERAL_INFO)
            ESP_LOGI( TAG, "Event: %u", eventType );
            #endif
            xSemaphoreTake( buttonDriver->conditionListMutex, portMAX_DELAY );
            {
                for ( int i = 0; i < buttonDriver->conditionCount; i++ ) {
                    button_trigger_s *condition = buttonDriver->conditions[ i ];
                    if ( condition->eventType == eventType ) {

                        #if (CONFIG_TA_DEBUG_GENERAL_INFO)
                        ESP_LOGI( TAG, "Triggering signal %u", condition->trigger->trigger_id );
                        #endif
                        buttonDriver->onTriggerSignal( condition->trigger->trigger_id );
                    }
                }
            }
            xSemaphoreGive( buttonDriver->conditionListMutex );
        }
        previousLevel = level;
        vTaskDelay( 50 / portTICK_PERIOD_MS );
    }
}

void Button_Free( TriggerDriver *driver ) {
    button_driver_s *buttonDriver = driver->state;
    free( buttonDriver->gpioConfig );
    free( buttonDriver );
    free( driver );
}
/***********************************************************************/
