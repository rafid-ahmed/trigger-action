#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "LaserDriver.h"


/***********************************************************************/
/* Constants */
/***********************************************************************/
#define LASER_ACTION_LIST_SIZE    (64)

static const char* TAG = "LaserDriver";
/***********************************************************************/



/***********************************************************************/
/* Types */
/***********************************************************************/
typedef uint8_t laser_event_type;
#define LASER_EVENT_OFF		(0)
#define LASER_EVENT_ON		(1)

typedef struct {
    uint32_t actionId;
	laser_event_type eventType;
} ActionListItem;

typedef struct {
    uint8_t gpio;
    ActionListItem* actionList[ LASER_ACTION_LIST_SIZE ];
    uint16_t actionListPos;
} LaserDriver;
/***********************************************************************/

/***********************************************************************/
/* Functions */
/***********************************************************************/
// Driver API
void Laser_AddAction( void *state, action_definition_s *action );
void Laser_RemoveAction( void *state, action_definition_s *action );
void Laser_TakeAction( void *state, uint32_t actionId );
void Laser_Free( ActionDriver *driver );

/***********************************************************************/

/***********************************************************************/
/* Driver API implementation */
/***********************************************************************/
ActionDriver* Laser_NewDriver( uint8_t gpio ) {
    ActionDriver *driver = calloc( 1, sizeof( ActionDriver ) );
    LaserDriver *laserDriver = calloc( 1, sizeof( LaserDriver ) );

    driver->state = laserDriver;
    driver->addAction = Laser_AddAction;
    driver->removeAction = Laser_RemoveAction;
    driver->takeAction = Laser_TakeAction;
    driver->taskLoop = NULL;
    driver->free = Laser_Free;

    laserDriver->gpio = gpio;
    laserDriver->actionListPos = 0;

    // Initialize GPIO
	gpio_pad_select_gpio(gpio);
	gpio_set_direction(gpio, GPIO_MODE_OUTPUT);

    ESP_LOGI( TAG, "Initialized Laser." );

    return driver;
}

void Laser_AddAction( void *state, action_definition_s *action ) {
    if ( action->params_length != 1 ) {
        ESP_LOGI( TAG, "Incorrect parameter length (%u).", action->params_length );
        return;
    }

    // Get and check driver state
    LaserDriver *laserDriver = state;
    if ( laserDriver->actionListPos >= LASER_ACTION_LIST_SIZE ) {
        ESP_LOGI( TAG, "Action list is full. Ignoring." );
        return;
    }

    // Skip existing actions
    for ( int i = 0; i < laserDriver->actionListPos; i++ ) {
        ActionListItem *item = laserDriver->actionList[ i ];
        if ( item->actionId == action->action_id ) return; // Already exists
    }

    // Add new action
    ActionListItem *item = malloc( sizeof( ActionListItem ) );
    item->actionId = action->action_id;
	item->eventType = action->params[ 0 ];
    laserDriver->actionList[ laserDriver->actionListPos++ ] = item;
}

void Laser_RemoveAction( void *state, action_definition_s *action ) {
    LaserDriver *laserDriver = state;

    // Find the action
    for ( int i = 0; i < laserDriver->actionListPos; i++ ) {
        ActionListItem *item = laserDriver->actionList[ i ];
        if ( item->actionId == action->action_id ) {
            free( item );
            laserDriver->actionListPos -= 1;
            laserDriver->actionList[ i ] = laserDriver->actionList[ laserDriver->actionListPos ];
        }
    }
}

void Laser_TakeAction( void *state, uint32_t actionId ) {
    LaserDriver *laserDriver = state;

    // Find the action
    for ( int i = 0; i < laserDriver->actionListPos; i++ ) {
        ActionListItem *item = laserDriver->actionList[ i ];
        if ( item->actionId == actionId ) {
			if (item->eventType == LASER_EVENT_ON) {
				gpio_set_level(laserDriver->gpio, 0);
				ESP_LOGI( TAG, "Laser on." );
			} else if (item->eventType == LASER_EVENT_OFF) {
				gpio_set_level(laserDriver->gpio, 1);
				ESP_LOGI( TAG, "Laser off." );
			}
        }
    }
}

void Laser_Free( ActionDriver *driver ) {
    free( driver->state );
    free( driver );
}

/***********************************************************************/
