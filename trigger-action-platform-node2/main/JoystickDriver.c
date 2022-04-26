/*
Copyright (c) 2019 Teemu Kärkkäinen

Joystick driver for use in trigger-action programming.
*/

/***********************************************************************/
/* Includes */
/***********************************************************************/
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include <freertos/semphr.h>
#include <esp_log.h>

#include "JoystickDriver.h"
/***********************************************************************/


/***********************************************************************/
/* Definitions */
/***********************************************************************/
static const char* TAG = "JoystickDriver";

#define JOYSTICK_CONDITION_LIST_SIZE   (64)

typedef uint8_t joystick_trigger_axis_t;		//TODO move to header
#define JOYSTICK_AXIS_BUTTON	(0)
#define JOYSTICK_AXIS_X			(1)
#define JOYSTICK_AXIS_Y			(2)

typedef uint8_t joystick_trigger_op_t;			//TODO move to header or into trigger_action.h as generic operands over uint8_t
#define JOYSTICK_OP_EQ			(0)
#define JOYSTICK_OP_LT			(1)
#define JOYSTICK_OP_GT			(2)
#define JOYSTICK_OP_LEQ			(3)
#define JOYSTICK_OP_GEQ			(4)
//with range, both ends are inclusive
#define JOYSTICK_OP_RANGE		(5)
#define JOYSTICK_OP_ERROR		(6)

typedef struct {
    trigger_definition_s	*trigger;
    joystick_trigger_axis_t axis;
	joystick_trigger_op_t	op;
	uint8_t					value;		//when JOYSTICK_AXIS_BUTTON, then 0=off/up, else on/down
										// this is inverse to the physical reading, where 0=down and 1 = up
	uint8_t					range;		//only used with JOYSTICK_OP_RANGE as upper range delimiter
} joystick_trigger_s;

typedef struct {
    TriggerSignalCallback onTriggerSignal;
    uint8_t			buttonGPIO;
    gpio_config_t	*gpioConfig;
	adc1_channel_t	xADC1chan;
	adc1_channel_t	yADC1chan;


    joystick_trigger_s*	conditions[ JOYSTICK_CONDITION_LIST_SIZE ];
    uint32_t			conditionCount;
    SemaphoreHandle_t	conditionListMutex;
} joystick_driver_s;

TriggerDriver* Joystick_NewDriver( TriggerSignalCallback onTriggerSignal, uint8_t gpio , adc1_channel_t xADC, adc1_channel_t yADC);
void Joystick_AddTrigger( void *state, trigger_definition_s const *condition );
void Joystick_RemoveTrigger( void *state, trigger_definition_s const *condition );
void Joystick_TaskLoop( void *state, uint32_t pulseMillis );
void Joystick_Free( TriggerDriver *driver );
/***********************************************************************/


/***********************************************************************/
/* Private */
/***********************************************************************/

/***********************************************************************/


/***********************************************************************/
/* API implementation */
/***********************************************************************/
TriggerDriver* Joystick_NewDriver( TriggerSignalCallback onTriggerSignal, uint8_t gpio,
								   adc1_channel_t xADC1chan, adc1_channel_t yADC1chan ) {
    TriggerDriver *triggerDriver = calloc( 1, sizeof( TriggerDriver ) );
    triggerDriver->addTrigger = Joystick_AddTrigger;
    triggerDriver->removeTrigger = Joystick_RemoveTrigger;
    triggerDriver->taskLoop = Joystick_TaskLoop;
    triggerDriver->free = Joystick_Free;

    joystick_driver_s *joystickDriver = calloc( 1, sizeof( joystick_driver_s ) );
    joystickDriver->onTriggerSignal = onTriggerSignal;
	joystickDriver->buttonGPIO = gpio;
    joystickDriver->gpioConfig = calloc( 1, sizeof( gpio_config_t ) );
	joystickDriver->xADC1chan = xADC1chan;
	joystickDriver->yADC1chan = yADC1chan;
    joystickDriver->conditionListMutex = xSemaphoreCreateMutex();

    triggerDriver->state = joystickDriver;

    // Initialize GPIO
    joystickDriver->gpioConfig->intr_type = GPIO_PIN_INTR_DISABLE;
    joystickDriver->gpioConfig->mode = GPIO_MODE_INPUT;
    joystickDriver->gpioConfig->pin_bit_mask = ( 1ULL << gpio );
    joystickDriver->gpioConfig->pull_down_en = 0;
    joystickDriver->gpioConfig->pull_up_en = 0;
    esp_err_t result = gpio_config( joystickDriver->gpioConfig );
    if ( result != ESP_OK ) ESP_LOGI( TAG, "Failed to configure GPIO (%d)", result );

	// Initialise ADC
	adc1_config_width(ADC_WIDTH_BIT_9);
	adc1_config_channel_atten(joystickDriver->xADC1chan, ADC_ATTEN_DB_11);
    if ( result != ESP_OK ) ESP_LOGI( TAG, "Failed to configure ADC channel for x-axis (%d)", result );
	adc1_config_channel_atten(joystickDriver->yADC1chan, ADC_ATTEN_DB_11);
    if ( result != ESP_OK ) ESP_LOGI( TAG, "Failed to configure ADC channel for y-axis (%d)", result );


    return triggerDriver;
}

static void parseTrigger( joystick_trigger_s *trig) {
	if (trig->trigger->params_length < 2) {
		trig->op = JOYSTICK_OP_ERROR;
		return;
	}

	trig->axis = trig->trigger->params[0];
	trig->op = trig->trigger->params[1];

	switch (trig->op) {
		case JOYSTICK_OP_EQ:
		case JOYSTICK_OP_LT:
		case JOYSTICK_OP_GT:
		case JOYSTICK_OP_LEQ:
		case JOYSTICK_OP_GEQ:
			if (trig->trigger->params_length < 3) {
				trig->op = JOYSTICK_OP_ERROR;
				return;
			}
			trig->value = trig->trigger->params[2];
			break;
		case JOYSTICK_OP_RANGE:
			if (trig->trigger->params_length < 4) {
				trig->op = JOYSTICK_OP_ERROR;
				return;
			}
			trig->value = trig->trigger->params[2];
			trig->range = trig->trigger->params[3];
			break;
		default:
			trig->op = JOYSTICK_OP_ERROR;
			return;
	}
}

void Joystick_AddTrigger( void *state, trigger_definition_s const *condition ) {
    // Preconditions
    if ( condition->trigger_type != JOYSTICK_TRIGGER_TYPE || condition->params_length < 2 ) {
        return;
    }

    joystick_driver_s *joystickDriver = state;
    xSemaphoreTake( joystickDriver->conditionListMutex, portMAX_DELAY );
    {
        // Get and check the driver
        if ( joystickDriver->conditionCount >= JOYSTICK_CONDITION_LIST_SIZE ) {
            ESP_LOGI( TAG, "Condition list is full. Ignoring new trigger." );
        }


		joystick_trigger_s *trig = calloc(1, sizeof(joystick_trigger_s));
		trig->trigger = ( trigger_definition_s* )condition;
		parseTrigger(trig);
		if (trig->op == JOYSTICK_OP_ERROR) {
			ESP_LOGI(TAG, "ERROR: could not parse trigger");
			free(trig);
			return;
		}

        ESP_LOGI( TAG, "New trigger: axis=%u, op=%u, val=%u, range=%u", trig->axis, trig->op, trig->value, trig->range );

		// Add the condition
        ESP_LOGI( TAG, "Adding new trigger: axis=%u, op=%u", trig->axis, trig->op );
		joystickDriver->conditions[ joystickDriver->conditionCount++ ] = trig;
    }
    xSemaphoreGive( joystickDriver->conditionListMutex );
}

void Joystick_RemoveTrigger( void *state, trigger_definition_s const *trigger ) {
    joystick_driver_s *joystickDriver = state;
    xSemaphoreTake( joystickDriver->conditionListMutex, portMAX_DELAY );
    {
        for ( int i = 0; i < joystickDriver->conditionCount; i++ ) {
            joystick_trigger_s *condition = joystickDriver->conditions[ i ];
            if ( condition->trigger->trigger_id == trigger->trigger_id ) {
                joystickDriver->conditionCount -= 1;
                joystickDriver->conditions[ i ] = joystickDriver->conditions[ joystickDriver->conditionCount ];
                i--; // The moved item may be the same id
            }
        }
    }
    xSemaphoreGive( joystickDriver->conditionListMutex );
}

static bool evalCond(joystick_trigger_s *trig, uint8_t val) {
	switch(trig->op) {
		case JOYSTICK_OP_EQ: return val == trig->value;
		case JOYSTICK_OP_LT: return val < trig->value;
		case JOYSTICK_OP_GT: return val > trig->value;
		case JOYSTICK_OP_LEQ: return val <= trig->value;
		case JOYSTICK_OP_GEQ: return val >= trig->value;
		case JOYSTICK_OP_RANGE: return val >= trig->value && val <= trig->range;
		default: ESP_LOGI(TAG, "Unknown condition operator (%d)", trig->op);
	}
	return false;
}

void Joystick_TaskLoop( void *state, uint32_t pulseMillis ) {
    // TODO: Should debounce more intelligently, but a 50ms delay seems to be good enough
    joystick_driver_s *joystickDriver = state;

    uint8_t prev_button = gpio_get_level( joystickDriver->buttonGPIO );
	uint8_t prev_xADC = adc1_get_raw (joystickDriver->xADC1chan) / 21;	//reduce the input into 25 steps
	uint8_t prev_yADC = adc1_get_raw (joystickDriver->yADC1chan) / 21;
    while ( true ) {
		uint8_t button = gpio_get_level( joystickDriver->buttonGPIO );
		uint8_t xADC = adc1_get_raw (joystickDriver->xADC1chan) / 21;
		uint8_t yADC = adc1_get_raw (joystickDriver->yADC1chan) / 21;
		if (prev_button != button) {
			uint8_t buttonState = (button == 0) ? 0 : 1; //reading of 0 = down, reading of 1 = up
			ESP_LOGI(TAG, "Event: Button %s", (button == 0) ? "down" : "up");
            xSemaphoreTake( joystickDriver->conditionListMutex, portMAX_DELAY );
			{
                for ( int i = 0; i < joystickDriver->conditionCount; i++ ) {
                    joystick_trigger_s *condition = joystickDriver->conditions[ i ];
                    if ( condition->axis == JOYSTICK_AXIS_BUTTON && (condition->value == 0 ? 0 : 1) != buttonState) { //button condition is ivnerse to button reading
                        ESP_LOGI( TAG, "Triggering signal %u", condition->trigger->trigger_id );
                        joystickDriver->onTriggerSignal( condition->trigger->trigger_id );
                    }
                }
			}
			xSemaphoreGive( joystickDriver->conditionListMutex );
		}
		if (prev_xADC != xADC) {
			ESP_LOGI(TAG, "Event: x-axis %u", xADC);
            xSemaphoreTake( joystickDriver->conditionListMutex, portMAX_DELAY );
			{
                for ( int i = 0; i < joystickDriver->conditionCount; i++ ) {
                    joystick_trigger_s *condition = joystickDriver->conditions[ i ];
                    if ( condition->axis == JOYSTICK_AXIS_X && evalCond(condition, xADC)) {
                        ESP_LOGI( TAG, "Triggering signal %u", condition->trigger->trigger_id );
                        joystickDriver->onTriggerSignal( condition->trigger->trigger_id );
                    }
                }
			}
			xSemaphoreGive( joystickDriver->conditionListMutex );
		}
		if (prev_yADC != yADC) {
			ESP_LOGI(TAG, "Event: y-axis %u", yADC);
            xSemaphoreTake( joystickDriver->conditionListMutex, portMAX_DELAY );
			{
                for ( int i = 0; i < joystickDriver->conditionCount; i++ ) {
                    joystick_trigger_s *condition = joystickDriver->conditions[ i ];
                    if ( condition->axis == JOYSTICK_AXIS_Y && evalCond(condition, yADC)) {
                        ESP_LOGI( TAG, "Triggering signal %u", condition->trigger->trigger_id );
                        joystickDriver->onTriggerSignal( condition->trigger->trigger_id );
                    }
                }
			}
			xSemaphoreGive( joystickDriver->conditionListMutex );
		}
		prev_button = button;
		prev_xADC = xADC;
		prev_yADC = yADC;
        vTaskDelay( 50 / portTICK_PERIOD_MS );
    }
}

void Joystick_Free( TriggerDriver *driver ) {
    joystick_driver_s *joystickDriver = driver->state;
    free( joystickDriver->gpioConfig );
    free( joystickDriver );
    free( driver );
}
/***********************************************************************/
