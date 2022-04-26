#include <freertos/FreeRTOS.h>
#include <driver/ledc.h>
#include <esp_log.h>

#include "RgbLedDriver.h"


/***********************************************************************/
/* Constants */
/***********************************************************************/
#define RGBLED_COLOR_RED (0)
#define RGBLED_COLOR_GREEN (1)
#define RGBLED_COLOR_BLUE (2)

#define RGBLED_DUTY_MAX (4095)

#define RGBLED_ACTION_LIST_SIZE    (64)

static const char* TAG = "RgbLedDriver";
/***********************************************************************/



/***********************************************************************/
/* Types */
/***********************************************************************/
typedef struct {
    uint32_t actionId;
    uint8_t red, green, blue;
} ActionListItem;

typedef struct {
    ledc_timer_config_t ledTimer;
    ledc_channel_config_t ledChannels[ 3 ];
    uint8_t currentRed, currentGreen, currentBlue;
    ActionListItem* actionList[ RGBLED_ACTION_LIST_SIZE ];
    uint16_t actionListPos;
} RgbLedDriver;
/***********************************************************************/

/***********************************************************************/
/* Functions */
/***********************************************************************/
// Driver API
void RgbLed_AddAction( void *state, action_definition_s *action );
void RgbLed_RemoveAction( void *state, action_definition_s *action );
void RgbLed_TakeAction( void *state, uint32_t actionId );
void RgbLed_Free( ActionDriver *driver );

// Private
static uint32_t color_to_duty( uint16_t color );
static void rgb_led_set_color( RgbLedDriver *ledDriver, uint8_t red, uint8_t green, uint8_t blue );
/***********************************************************************/


/***********************************************************************/
/* Private */
/***********************************************************************/
static uint32_t color_to_duty( uint16_t color ) {
    return RGBLED_DUTY_MAX - color * 16;
}

static void rgb_led_set_color( RgbLedDriver *ledDriver, uint8_t red, uint8_t green, uint8_t blue ) {
    if ( ledDriver->currentRed == red && ledDriver->currentGreen == green && ledDriver->currentBlue == blue ) return;

    ledDriver->currentRed = red;
    ledDriver->currentGreen = green;
    ledDriver->currentBlue = blue;

    ledc_set_duty( ledDriver->ledChannels[ RGBLED_COLOR_RED ].speed_mode,
                    ledDriver->ledChannels[ RGBLED_COLOR_RED ].channel, color_to_duty( red ) );
    ledc_update_duty( ledDriver->ledChannels[ RGBLED_COLOR_RED ].speed_mode,
                    ledDriver->ledChannels[ RGBLED_COLOR_RED ].channel );

    ledc_set_duty( ledDriver->ledChannels[ RGBLED_COLOR_GREEN ].speed_mode,
                    ledDriver->ledChannels[ RGBLED_COLOR_GREEN ].channel, color_to_duty( green ) );
    ledc_update_duty( ledDriver->ledChannels[ RGBLED_COLOR_GREEN ].speed_mode,
                    ledDriver->ledChannels[ RGBLED_COLOR_GREEN ].channel );

    ledc_set_duty( ledDriver->ledChannels[ RGBLED_COLOR_BLUE ].speed_mode,
                    ledDriver->ledChannels[ RGBLED_COLOR_BLUE ].channel, color_to_duty( blue ) );
    ledc_update_duty( ledDriver->ledChannels[ RGBLED_COLOR_BLUE ].speed_mode,
                    ledDriver->ledChannels[ RGBLED_COLOR_BLUE ].channel );
}
/***********************************************************************/


/***********************************************************************/
/* Driver API implementation */
/***********************************************************************/
ActionDriver* RgbLed_NewDriver( ledc_timer_t timer, ledc_channel_t redChannel, uint8_t redGpio, ledc_channel_t greenChannel, uint8_t greenGpio, ledc_channel_t blueChannel, uint8_t blueGpio ) {
    
    ActionDriver *driver = calloc( 1, sizeof( ActionDriver ) );
    RgbLedDriver *ledDriver = calloc( 1, sizeof( RgbLedDriver ) );

    driver->state = ledDriver;
    driver->addAction = RgbLed_AddAction;
    driver->removeAction = RgbLed_RemoveAction;
    driver->takeAction = RgbLed_TakeAction;
    driver->taskLoop = NULL;
    driver->free = RgbLed_Free;
    
    ledDriver->actionListPos = 0;

    // Setup timer
    ledDriver->ledTimer.duty_resolution = LEDC_TIMER_12_BIT;
    ledDriver->ledTimer.freq_hz = 1000;
    ledDriver->ledTimer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledDriver->ledTimer.timer_num = timer;

    // Setup channels
    ledDriver->ledChannels[ RGBLED_COLOR_RED ].channel    = redChannel;
    ledDriver->ledChannels[ RGBLED_COLOR_RED ].duty       = RGBLED_DUTY_MAX;
    ledDriver->ledChannels[ RGBLED_COLOR_RED ].gpio_num   = redGpio;
    ledDriver->ledChannels[ RGBLED_COLOR_RED ].speed_mode = LEDC_HIGH_SPEED_MODE;
    ledDriver->ledChannels[ RGBLED_COLOR_RED ].timer_sel  = timer;

    ledDriver->ledChannels[ RGBLED_COLOR_GREEN ].channel    = greenChannel;
    ledDriver->ledChannels[ RGBLED_COLOR_GREEN ].duty       = RGBLED_DUTY_MAX;
    ledDriver->ledChannels[ RGBLED_COLOR_GREEN ].gpio_num   = greenGpio;
    ledDriver->ledChannels[ RGBLED_COLOR_GREEN ].speed_mode = LEDC_HIGH_SPEED_MODE;
    ledDriver->ledChannels[ RGBLED_COLOR_GREEN ].timer_sel  = timer;

    ledDriver->ledChannels[ RGBLED_COLOR_BLUE ].channel    = blueChannel;
    ledDriver->ledChannels[ RGBLED_COLOR_BLUE ].duty       = RGBLED_DUTY_MAX;
    ledDriver->ledChannels[ RGBLED_COLOR_BLUE ].gpio_num   = blueGpio;
    ledDriver->ledChannels[ RGBLED_COLOR_BLUE ].speed_mode = LEDC_HIGH_SPEED_MODE;
    ledDriver->ledChannels[ RGBLED_COLOR_BLUE ].timer_sel  = timer;

    // Initialize
    esp_err_t result;

    ESP_LOGI( TAG, "Initializing timer." );
    result = ledc_timer_config( &( ledDriver->ledTimer ) );
    if ( result != ESP_OK ) ESP_LOGI( TAG, "Failed to configure timer (%d)", result );

    ESP_LOGI( TAG, "Initializing red." );
    result = ledc_channel_config( &( ledDriver->ledChannels[ RGBLED_COLOR_RED ] ) );
    if ( result != ESP_OK ) ESP_LOGI( TAG, "Failed to configure red channel (%d)", result );

    ESP_LOGI( TAG, "Initializing green." );
    result = ledc_channel_config( &( ledDriver->ledChannels[ RGBLED_COLOR_GREEN ] ) );
    if ( result != ESP_OK ) ESP_LOGI( TAG, "Failed to configure green channel (%d)", result );

    ESP_LOGI( TAG, "Initializing blue." );
    result = ledc_channel_config( &( ledDriver->ledChannels[ RGBLED_COLOR_BLUE ] ) );
    if ( result != ESP_OK ) ESP_LOGI( TAG, "Failed to configure blue channel (%d)", result );

    ESP_LOGI( TAG, "Initialized LED." );

    // Set the initial color
    ledDriver->currentRed = 1; // 1 since otherwise the rgb_led_set_color will optimize the call away
    ledDriver->currentGreen = 1;
    ledDriver->currentBlue = 1;
    rgb_led_set_color( ledDriver, 0, 0, 0 );

    return driver;
}

void RgbLed_AddAction( void *state, action_definition_s *action ) {
    if ( action->params_length != 3 ) {
        ESP_LOGI( TAG, "Incorrect parameter length (%u).", action->params_length );
        return;
    }

    // Get and check driver state
    RgbLedDriver *ledDriver = state;
    if ( ledDriver->actionListPos >= RGBLED_ACTION_LIST_SIZE ) {
        ESP_LOGI( TAG, "Action list is full. Ignoring." );
        return;
    }

    // Skip existing actions
    for ( int i = 0; i < ledDriver->actionListPos; i++ ) {
        ActionListItem *item = ledDriver->actionList[ i ];
        if ( item->actionId == action->action_id ) return; // Already exists
    }

    // Add new action
    ActionListItem *item = malloc( sizeof( ActionListItem ) );
    item->actionId = action->action_id;
    item->red = action->params[ 0 ];
    item->green = action->params[ 1 ];
    item->blue = action->params[ 2 ];
    ledDriver->actionList[ ledDriver->actionListPos++ ] = item;

    #if (CONFIG_TA_DEBUG_GENERAL_INFO)
    ESP_LOGI( TAG, "Added a new action (%u)", action->action_id );
    #endif
}

void RgbLed_RemoveAction( void *state, action_definition_s *action ) {
    RgbLedDriver *ledDriver = state;

    // Find the action
    for ( int i = 0; i < ledDriver->actionListPos; i++ ) {
        ActionListItem *item = ledDriver->actionList[ i ];
        if ( item->actionId == action->action_id ) {
            free( item );
            ledDriver->actionListPos -= 1;
            ledDriver->actionList[ i ] = ledDriver->actionList[ ledDriver->actionListPos ];
        }
    }
}

void RgbLed_TakeAction( void *state, uint32_t actionId ) {
    RgbLedDriver *ledDriver = state;

    // Find the action
    for ( int i = 0; i < ledDriver->actionListPos; i++ ) {
        ActionListItem *item = ledDriver->actionList[ i ];
        if ( item->actionId == actionId ) {
            #if (CONFIG_TA_DEBUG_GENERAL_INFO)
            ESP_LOGI( TAG, "Taking action. R:%u G:%u B:%u", item->red, item->green, item->blue);
            #endif
            rgb_led_set_color( ledDriver, item->red, item->green, item->blue );
        }
    }
}

void RgbLed_Free( ActionDriver *driver ) {
    free( driver->state );
    free( driver );
}

/***********************************************************************/
