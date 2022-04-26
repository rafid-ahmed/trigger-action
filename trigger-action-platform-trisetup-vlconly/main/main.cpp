extern "C" 
{
    #include <freertos/FreeRTOS.h>
    #include <freertos/task.h>
    #include <esp_system.h>
    #include <nvs_flash.h>
    #include <driver/ledc.h>
    #include <string.h>

    #include <esp_log.h>
    #include <esp_timer.h>

    #include "TAParser.h"
    #include "RgbLedDriver.h"
    #include "trigger_driver.h"
    #include "Bmp180Driver.h"
    #include "ButtonDriver.h"
    #include "ProtocolGen.h"
    #include "BuzzerDriver.h"
    #include "LaserDriver.h"
    #include "JoystickDriver.h"
    #include "IRRecvDriver.h"
}

#include "TriggerActionNetwork.h"

/***********************************************************************/
/* Constants */
/***********************************************************************/
static const char* TAG = "beacontest";

#define I2C_DATA_PIN    (32)
#define I2C_CLOCK_PIN   (33)
#define BUZZER_PIN      (13)

#define PULSE_MILLIS    (1000)
/***********************************************************************/


/***********************************************************************/
/* Types */
/***********************************************************************/
typedef struct {
    TriggerDriver *driver;
    uint32_t pulseMillis;
} trigger_driver_runnable_s;

typedef struct {
    ActionDriver *driver;
    uint32_t pulseMillis;
} action_driver_runnable_s;

// abstract action driver with info
// params: action_type, instance_id, *driver
typedef struct {
    uint32_t action_type;
    uint32_t instance_id;
    ActionDriver *driver;
} action_driver_s;

// abstract trigger driver with info
// params: trigger_type, instance_id, *driver
typedef struct {
    uint32_t trigger_type;
    uint32_t instance_id;
    TriggerDriver *driver;
} trigger_driver_s;

// trigger definition list item
// params: timestamp, trigger_definition_s
typedef struct {
    int64_t timestamp;
    trigger_definition_s *trigger;
} trigger_list_item_s;

// action definition list item
// params: timestamp, action_definition_s
typedef struct {
    int64_t timestamp;
    action_definition_s *action;
} action_list_item_s;

// rule definition list item
// params: timestamp, rule_s
typedef struct {
    int64_t timestamp;
    rule_s *rule;
} rule_list_item_s;
/***********************************************************************/


/***********************************************************************/
/* Function prototypes */
/***********************************************************************/

void parseTriggerActionPayload( char *media, uint8_t priority, uint8_t *payload, uint8_t payloadLength, bool deprovision ) ;

/***********************************************************************/


/***********************************************************************/
/* Vars */
/***********************************************************************/
// static BleParser *bleParser;
static TAParser *parser;

static action_driver_s* action_drivers[ 10 ];
static uint8_t action_driver_count = 0;

static trigger_driver_s* trigger_drivers[ 10 ];
static uint8_t trigger_driver_count = 0;

// trigger definition list
static trigger_list_item_s* trigger_definitions[ 100 ];
// number of trigger definitions inside trigger definition list
static uint8_t trigger_defition_count = 0;

// action definition list
static action_list_item_s* action_definitions[ 100 ];
// number of action definitions inside action definition list
static uint8_t action_definition_count = 0;

// rule definition list
static rule_list_item_s* rules[ 100 ];
// number of rule definitions inside rule definition list
static uint8_t rule_count = 0;

/***********************************************************************/
/* Signal routing */
/***********************************************************************/

// #pragma region Signal routing

static void route_signal( uint32_t trigger_id ) {
    // Find a rules with the trigger
    for ( int rule_i = 0; rule_i < rule_count; rule_i++ ) {
        rule_s *rule = rules[ rule_i ]->rule;
        if ( rule->trigger_id == trigger_id ) {
            uint32_t actionId = rule->action_id;
            // Find the action definition
            for ( int action_i = 0; action_i < action_definition_count; action_i++ ) {
                action_definition_s *action = action_definitions[ action_i ]->action;
                if ( action->action_id == actionId ) {
                    // Find the action driver
                    for ( int driver_i = 0; driver_i < action_driver_count; driver_i++ ) {
                        if ( action_drivers[ driver_i ]->action_type == action->action_type ) {
                            #if (CONFIG_TA_DEBUG_GENERAL_INFO)
                            ESP_LOGI( TAG, "Triggering a driver." );
                            #endif
                            action_drivers[ driver_i ]->driver->takeAction( action_drivers[ driver_i ]->driver->state, actionId );
                        }
                    }
                }
            }
        }
    }
}

void onLocalSignal( uint32_t trigger_id ) {
    #if (CONFIG_TA_DEBUG_GENERAL_INFO)
    ESP_LOGI( TAG, "Local signal: %u", trigger_id );
    #endif
    route_signal( trigger_id );
    uint32_t bufferLength;
    uint8_t *buffer = ProtoGen_NewSignal( trigger_id, &bufferLength );
    networkWrite( buffer, bufferLength, PRIORITY_HIGH, NON_PERSISTENT);
    free( buffer );
}

// #pragma endregion

/***********************************************************************/
/* Parser callbacks */
/***********************************************************************/

// #pragma region Parser callbacks

void onError( ParseError error ) {
    ESP_LOGI( TAG, "onError(%u)", error );
}

void onTriggerDefinition( trigger_definition_s *trigger, bool deprovision ) {
//    ESP_LOGI( TAG, "onTriggerDefinition: id = %u", trigger->trigger_id );

    // Search for an existing trigger definition
    for ( int i = 0; i < trigger_defition_count; i++ ) {
        if ( trigger_definitions[ i ]->trigger->trigger_id == trigger->trigger_id 
          && trigger_definitions[ i ]->trigger->trigger_type == trigger->trigger_type
          && trigger_definitions[ i ]->trigger->instance_id == trigger->instance_id
          && !deprovision ) {
            // Same ID exists, update timestamp and ignore.
            /// TODO: new tigger with same ID should override the previous?
            if ( trigger_definitions[ i ]->trigger->trigger_id == trigger->trigger_id ) {
                trigger_definitions[ i ]->timestamp = esp_timer_get_time();
            } else {
                trigger_definitions[ i ]->trigger->trigger_id = trigger->trigger_id;
            }
            free_trigger_definition( trigger );
            return;
        }
    }

    // Add the trigger condition to the drivers
    for ( int i = 0; i < trigger_driver_count; i++ ) {
        trigger_driver_s *trigger_driver = trigger_drivers[ i ];
        if ( ( trigger_driver->trigger_type == trigger->trigger_type )
                && ( ( trigger->instance_id == 0 ) || ( trigger_driver->instance_id == trigger->instance_id ) ) ) {

            if ( deprovision ) {
                #if (CONFIG_TA_DEBUG_GENERAL_INFO)
                ESP_LOGW( TAG, "Removing trigger, id = %u", trigger->trigger_id );
                #endif
                trigger_driver->driver->removeTrigger( trigger_driver->driver->state, trigger );
            } else {
                // Create a new trigger definition
                #if (CONFIG_TA_DEBUG_GENERAL_INFO)
                ESP_LOGW( TAG, "Adding a new trigger, id = %u", trigger->trigger_id );
                #endif
                trigger_list_item_s *newTrigger = (trigger_list_item_s *)malloc(sizeof( trigger_list_item_s ) );
                newTrigger->trigger = trigger;
                newTrigger->timestamp = esp_timer_get_time();
                trigger_definitions[ trigger_defition_count ] = newTrigger;
                trigger_defition_count += 1;
                
                trigger_driver->driver->addTrigger( trigger_driver->driver->state, trigger );
            }
        }
    }
}

void onActionDefinition( action_definition_s *action, bool deprovision ) {
//    ESP_LOGI( TAG, "onActionDefinition: id = %u", action->action_id );
    // Check for an existing action definition
    for ( int i = 0; i < action_definition_count; i++ ) {
        if ( action_definitions[ i ]->action->action_id == action->action_id 
          && action_definitions[ i ]->action->action_type == action->action_type 
          && action_definitions[ i ]->action->instance_id == action->instance_id
          && !deprovision ) {
            free_action_definition( action );
            action_definitions[ i ]->timestamp = esp_timer_get_time();
            return;
        }
    }

    // Add the new action to the drivers
    for ( int i = 0; i < action_driver_count; i++ ) {
        action_driver_s *actionDriver = action_drivers[ i ];
        if ( ( actionDriver->action_type == action->action_type )
                && ( ( action->instance_id == 0 ) || ( actionDriver->instance_id == action->instance_id) ) ) {

            if ( deprovision ) {
                #if (CONFIG_TA_DEBUG_GENERAL_INFO)
                ESP_LOGW( TAG, "Removing action, id = %u", action->action_id );
                #endif
                actionDriver->driver->removeAction( actionDriver->driver->state, action );
            } else {
                // Create a new action definition
                #if (CONFIG_TA_DEBUG_GENERAL_INFO)
                ESP_LOGW( TAG, "Adding a new action, id = %u", action->action_id );
                #endif
                action_list_item_s *newAction = (action_list_item_s *)malloc( sizeof( action_list_item_s ) );
                newAction->action = action;
                newAction->timestamp = esp_timer_get_time();
                action_definitions[ action_definition_count++ ] = newAction;
                
                actionDriver->driver->addAction( actionDriver->driver->state, action );
            }
        }
    }
}

void onRuleDefinition( rule_s *rule ) {
//    ESP_LOGI( TAG, "onRuleDefinition: id = %u", rule->rule_id );

    // Check for an existing rule
    for ( int i = 0; i < rule_count; i++ ) {
        if ( rules[ i ]->rule->rule_id == rule->rule_id ) {
            free_rule( rule );
            rules[ i ]->timestamp = esp_timer_get_time();
            return;
        }
    }

    // Create a new rule
    #if (CONFIG_TA_DEBUG_GENERAL_INFO)
    ESP_LOGW( TAG, "Adding a new rule, id = %u, %u -> %u", rule->rule_id, rule->trigger_id, rule->action_id );
    #endif
    rule_list_item_s *newRule = (rule_list_item_s *)malloc( sizeof( rule_list_item_s ) );
    newRule->rule = rule;
    newRule->timestamp = esp_timer_get_time();
    rules[ rule_count++ ] = newRule;
}

void onTriggerSignal( uint32_t trigger_id ) {
    // ESP_LOGI( TAG, "onTriggerSignal: id = %u", trigger_id );
    route_signal( trigger_id );
}

// #pragma endregion

/***********************************************************************/
/* Private functions */
/***********************************************************************/

// #pragma region Private functions

static void init_i2c() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_DATA_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_CLOCK_PIN;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config( I2C_NUM_0, &conf );
    i2c_driver_install( I2C_NUM_0, conf.mode, 0, 0, 0);
}

void parseTriggerActionPayload( char *media, uint8_t priority, uint8_t *payload, uint8_t payloadLength, bool deprovision ) {
    uint8_t pos = 0;
//    while ( pos < payloadLength ) {
        #if (CONFIG_TA_DEBUG_GENERAL_INFO)
        // ESP_LOGI(TAG, "Priority %d packet received from %s", priority, media);
        #endif
        pos += TAParser_Parse( parser, ( payloadLength - pos ), ( payload + pos ), deprovision );
        #if (CONFIG_TA_DEBUG_GENERAL_INFO)
        if ( pos != payloadLength ) 
            ESP_LOGI( TAG, "Parser has %u bytes left over.", ( payloadLength - pos ) );
        #endif
//    }
}

// #pragma endregion

/***********************************************************************/
/* Util functions */
/***********************************************************************/

// #pragma region Util

static void run_trigger_driver( void *params ) {
    trigger_driver_runnable_s *runnable = (trigger_driver_runnable_s *)params;
    runnable->driver->taskLoop( runnable->driver->state, runnable->pulseMillis );
}

static void start_trigger_driver( TriggerDriver *driver, uint32_t pulseMillis, const char * const taskName ) {
    if ( driver->taskLoop == NULL ) return;
    trigger_driver_runnable_s *runnable = (trigger_driver_runnable_s *)malloc( sizeof( trigger_driver_runnable_s ) );
    runnable->driver = driver;
    runnable->pulseMillis = pulseMillis;
    xTaskCreate( run_trigger_driver, taskName, 2048, runnable, 5, NULL );
}

static void run_action_driver( void *params ) {
    action_driver_runnable_s *runnable = (action_driver_runnable_s *)params;
    runnable->driver->taskLoop( runnable->driver->state, runnable->pulseMillis );
}

static void start_action_driver( ActionDriver *driver, uint32_t pulseMillis, const char * const taskName ) {
    if ( driver->taskLoop == NULL ) return;
    action_driver_runnable_s *runnable = (action_driver_runnable_s *)calloc( 1, sizeof( action_driver_runnable_s ) );
    runnable->driver = driver;
    runnable->pulseMillis = pulseMillis;
    xTaskCreate( run_action_driver, taskName, 2048, runnable, 5, NULL );
}

// #pragma endregion

/***********************************************************************/
/* Main */
/***********************************************************************/
extern "C" void app_main( void ) {
    ESP_LOGI( TAG, "started" );

    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_LOGI( TAG, "Flash initialized" );

    networkInit( parseTriggerActionPayload );

// Setup I2C -------------------------------------------------------------------------------
#ifdef CONFIG_I2C_ENABLED
    init_i2c();
    ESP_LOGI( TAG, "I2C initialized" );
#endif

// Set up parsers ------------------------------------------------------------------------------------
    parser = TAParser_NewParser( onError, onTriggerDefinition, onActionDefinition, onRuleDefinition, onTriggerSignal );
    ESP_LOGI( TAG, "Parsers created" );

// RGB Driver ------------------------------------------------------------------------------------
#ifdef CONFIG_DRIVER_RGB_LED
    // Setup RGB LED driver
    ActionDriver *rgbLedDriver = RgbLed_NewDriver( LEDC_TIMER_0, LEDC_CHANNEL_0, CONFIG_DRIVER_RGB_LED_GPIO_RED,
																 LEDC_CHANNEL_1, CONFIG_DRIVER_RGB_LED_GPIO_GREEN,
																 LEDC_CHANNEL_2, CONFIG_DRIVER_RGB_LED_GPIO_BLUE);
    action_drivers[ action_driver_count ] = (action_driver_s *)malloc( sizeof( action_driver_s ) );
    action_drivers[ action_driver_count ]->action_type = RGB_LED_ACTION_TYPE;
    action_drivers[ action_driver_count ]->instance_id = CONFIG_DRIVER_RGB_LED_INSTANCE_ID;
    action_drivers[ action_driver_count++ ]->driver = rgbLedDriver;

    #ifdef CONFIG_DRIVER_RGB_LED_ADVERTISE_LOCATION
        uint8_t *rgbLocationAd;
        uint32_t rgbLocationAdLength;
        rgbLocationAd = ProtoGen_NewActionLocationDescriptor( RGB_LED_ACTION_TYPE, CONFIG_DRIVER_RGB_LED_INSTANCE_ID,
                CONFIG_DRIVER_RGB_LED_ADVERTISE_LOCATION_X * 0.01, CONFIG_DRIVER_RGB_LED_ADVERTISE_LOCATION_Y * 0.01, CONFIG_DRIVER_RGB_LED_ADVERTISE_LOCATION_Z * 0.01,
                &rgbLocationAdLength );
        // networkWrite( rgbLocationAd, rgbLocationAdLength, PRIORITY_LOW, PERSISTENT);
        ESP_LOGI( TAG, "Set up location advertisement for RGB LED: (%i, %i, %i)",
                CONFIG_DRIVER_RGB_LED_ADVERTISE_LOCATION_X, CONFIG_DRIVER_RGB_LED_ADVERTISE_LOCATION_Y, CONFIG_DRIVER_RGB_LED_ADVERTISE_LOCATION_Z );
    #endif

    ESP_LOGI( TAG, "RGB LED driver created" );
#endif

// Laser Driver ------------------------------------------------------------------------------------
#ifdef CONFIG_DRIVER_LASER
	ActionDriver *laserDriver = Laser_NewDriver ( CONFIG_DRIVER_LASER_GPIO );
	action_drivers[ action_driver_count ] = malloc( sizeof( action_driver_s ) );
	action_drivers[ action_driver_count ]-> action_type = LASER_ACTION_TYPE;
    action_drivers[ action_driver_count ]->instance_id = CONFIG_DRIVER_LASER_INSTANCE_ID;
	action_drivers[ action_driver_count++ ]->driver = laserDriver;

    #ifdef CONFIG_DRIVER_LASER_ADVERTISE_LOCATION
        uint8_t *laserLocationAd;
        uint32_t laserLocationAdLength;
        laserLocationAd = ProtoGen_NewActionLocationDescriptor( LASER_ACTION_TYPE, CONFIG_DRIVER_LASER_INSTANCE_ID,
                CONFIG_DRIVER_LASER_ADVERTISE_LOCATION_X * 0.01, CONFIG_DRIVER_LASER_ADVERTISE_LOCATION_Y * 0.01, CONFIG_DRIVER_LASER_ADVERTISE_LOCATION_Z * 0.01,
                &laserLocationAdLength );
        // networkWrite( laserLocationAd, laserLocationAdLength, PRIORITY_LOW, PERSISTENT);
        ESP_LOGI( TAG, "Set up location advertisement for laser: (%i, %i, %i)",
                CONFIG_DRIVER_LASER_ADVERTISE_LOCATION_X, CONFIG_DRIVER_LASER_ADVERTISE_LOCATION_Y, CONFIG_DRIVER_LASER_ADVERTISE_LOCATION_Z );
    #endif

    ESP_LOGI( TAG, "Laser driver created" );
#endif

// BMP180 Driver ------------------------------------------------------------------------------------
#ifdef CONFIG_DRIVER_BMP180
    // Same driver for temperature and trigger types
    TriggerDriver *bmp180driver = Bmp180dr_NewDriver( onLocalSignal, I2C_NUM_0 );
    #ifdef CONFIG_DRIVER_BMP180_TEMPERATURE
        trigger_drivers[ trigger_driver_count ] = malloc( sizeof( trigger_driver_s ) );
        trigger_drivers[ trigger_driver_count ]->trigger_type = TEMPERATURE_TRIGGER_TYPE;
        trigger_drivers[ trigger_driver_count ]->instance_id = CONFIG_DRIVER_BMP180_TEMPERATURE_INSTANCE_ID;
        trigger_drivers[ trigger_driver_count++ ]->driver = bmp180driver;

        #ifdef CONFIG_DRIVER_BMP180_TEMPERATURE_ADVERTISE_LOCATION
            uint8_t *bmp180TemperatureLocationAd;
            uint32_t lbmp180TemperatureLocationAdLength;
            bmp180TemperatureLocationAd = ProtoGen_NewTriggerLocationDescriptor( TEMPERATURE_TRIGGER_TYPE, CONFIG_DRIVER_BMP180_TEMPERATURE_INSTANCE_ID,
                CONFIG_DRIVER_BMP180_TEMPERATURE_ADVERTISE_LOCATION_X * 0.01, CONFIG_DRIVER_BMP180_TEMPERATURE_ADVERTISE_LOCATION_Y * 0.01, CONFIG_DRIVER_BMP180_TEMPERATURE_ADVERTISE_LOCATION_Z * 0.01,
                &lbmp180TemperatureLocationAdLength );
            networkWrite( bmp180TemperatureLocationAd, lbmp180TemperatureLocationAdLength, PRIORITY_LOW, PERSISTENT);
            ESP_LOGI( TAG, "Set up location advertisement for BMP180 temperature: (%i, %i, %i)",
                CONFIG_DRIVER_BMP180_TEMPERATURE_ADVERTISE_LOCATION_X, CONFIG_DRIVER_BMP180_TEMPERATURE_ADVERTISE_LOCATION_Y, CONFIG_DRIVER_BMP180_TEMPERATURE_ADVERTISE_LOCATION_Z );
        #endif

        ESP_LOGI( TAG, "BMP180 temperature driver created" );
    #endif
    #ifdef CONFIG_DRIVER_BMP180_PRESSURE
        trigger_drivers[ trigger_driver_count ] = malloc( sizeof( trigger_driver_s ) );
        trigger_drivers[ trigger_driver_count ]->trigger_type = PRESSURE_TRIGGER_TYPE;
        trigger_drivers[ trigger_driver_count ]->instance_id = CONFIG_DRIVER_BMP180_PRESSURE_INSTANCE_ID;
        trigger_drivers[ trigger_driver_count++ ]->driver = bmp180driver;

        #ifdef CONFIG_DRIVER_BMP180_PRESSURE_ADVERTISE_LOCATION
            uint8_t *bmp180PressureLocationAd;
            uint32_t bmp180PressureLocationAdLength;
            bmp180PressureLocationAd = ProtoGen_NewTriggerLocationDescriptor( PRESSURE_TRIGGER_TYPE, CONFIG_DRIVER_BMP180_PRESSURE_INSTANCE_ID,
                CONFIG_DRIVER_BMP180_PRESSURE_ADVERTISE_LOCATION_X * 0.01, CONFIG_DRIVER_BMP180_PRESSURE_ADVERTISE_LOCATION_Y * 0.01, CONFIG_DRIVER_BMP180_PRESSURE_ADVERTISE_LOCATION_Z * 0.01,
                &bmp180PressureLocationAdLength );
            networkWrite( bmp180PressureLocationAd, bmp180PressureLocationAdLength, PRIORITY_LOW, PERSISTENT);
            ESP_LOGI( TAG, "Set up location advertisement for BMP180 pressure: (%i, %i, %i)",
                CONFIG_DRIVER_BMP180_PRESSURE_ADVERTISE_LOCATION_X, CONFIG_DRIVER_BMP180_PRESSURE_ADVERTISE_LOCATION_Y, CONFIG_DRIVER_BMP180_PRESSURE_ADVERTISE_LOCATION_Z );
        #endif

        ESP_LOGI( TAG, "BMP180 pressure driver created" );
    #endif
    #if defined(CONFIG_DRIVER_BMP180_TEMPERATURE) || defined(CONFIG_DRIVER_BMP180_PRESSURE)
        start_trigger_driver( bmp180driver, PULSE_MILLIS, "bmp180" );
    #endif
#endif

// Button Driver ------------------------------------------------------------------------------------
#ifdef CONFIG_DRIVER_BUTTON
#ifdef CONFIG_DRIVER_BUTTON_1_ENABLED
    TriggerDriver *buttonDriver1 = Button_NewDriver( onLocalSignal, CONFIG_DRIVER_BUTTON_1_GPIO );
    trigger_drivers[ trigger_driver_count ] = (trigger_driver_s *)calloc( 1, sizeof( trigger_driver_s ) );
    trigger_drivers[ trigger_driver_count ]->trigger_type = BUTTON_TRIGGER_TYPE;
    trigger_drivers[ trigger_driver_count ]->instance_id = CONFIG_DRIVER_BUTTON_1_ID;
    trigger_drivers[ trigger_driver_count++ ]->driver = buttonDriver1;

    #ifdef CONFIG_DRIVER_BUTTON_1_ADVERTISE_LOCATION
        uint8_t *button1LocationAd;
        uint32_t button1LocationAdLength;
        button1LocationAd = ProtoGen_NewTriggerLocationDescriptor( BUTTON_TRIGGER_TYPE, CONFIG_DRIVER_BUTTON_1_ID,
            CONFIG_DRIVER_BUTTON_1_ADVERTISE_LOCATION_X * 0.01, CONFIG_DRIVER_BUTTON_1_ADVERTISE_LOCATION_Y * 0.01, CONFIG_DRIVER_BUTTON_1_ADVERTISE_LOCATION_Z * 0.01,
            &button1LocationAdLength );
        networkWrite( button1LocationAd, button1LocationAdLength, PRIORITY_LOW, PERSISTENT);
        ESP_LOGI( TAG, "Set up location advertisement for button #1: (%i, %i, %i)",
                CONFIG_DRIVER_BUTTON_1_ADVERTISE_LOCATION_X, CONFIG_DRIVER_BUTTON_1_ADVERTISE_LOCATION_Y, CONFIG_DRIVER_BUTTON_1_ADVERTISE_LOCATION_Z );
    #endif

    ESP_LOGI( TAG, "Button #1 driver created" );
    start_trigger_driver( buttonDriver1, PULSE_MILLIS, "button1" );
#endif
#ifdef CONFIG_DRIVER_BUTTON_2_ENABLED
    TriggerDriver *buttonDriver2 = Button_NewDriver( onLocalSignal, CONFIG_DRIVER_BUTTON_2_GPIO );
    trigger_drivers[ trigger_driver_count ] = calloc( 1, sizeof( trigger_driver_s ) );
    trigger_drivers[ trigger_driver_count ]->trigger_type = BUTTON_TRIGGER_TYPE;
    trigger_drivers[ trigger_driver_count ]->instance_id = CONFIG_DRIVER_BUTTON_2_ID;
    trigger_drivers[ trigger_driver_count++ ]->driver = buttonDriver2;

    #ifdef CONFIG_DRIVER_BUTTON_2_ADVERTISE_LOCATION
        uint8_t *button2LocationAd;
        uint32_t button2LocationAdLength;
        button2LocationAd = ProtoGen_NewTriggerLocationDescriptor( BUTTON_TRIGGER_TYPE, CONFIG_DRIVER_BUTTON_2_ID,
            CONFIG_DRIVER_BUTTON_2_ADVERTISE_LOCATION_X * 0.01, CONFIG_DRIVER_BUTTON_2_ADVERTISE_LOCATION_Y * 0.01, CONFIG_DRIVER_BUTTON_2_ADVERTISE_LOCATION_Z * 0.01,
            &button2LocationAdLength );
        networkWrite( button2LocationAd, button2LocationAdLength, PRIORITY_LOW, PERSISTENT);
        ESP_LOGI( TAG, "Set up location advertisement for button #2: (%i, %i, %i)",
                CONFIG_DRIVER_BUTTON_2_ADVERTISE_LOCATION_X, CONFIG_DRIVER_BUTTON_2_ADVERTISE_LOCATION_Y, CONFIG_DRIVER_BUTTON_2_ADVERTISE_LOCATION_Z );
    #endif

    ESP_LOGI( TAG, "Button #2 driver created" );
    start_trigger_driver( buttonDriver2, PULSE_MILLIS, "button2" );
#endif
#endif

#ifdef CONFIG_DRIVER_JOYSTICK
	TriggerDriver *joystickDriver = Joystick_NewDriver(onLocalSignal, 13, 4, 5);
	trigger_drivers[ trigger_driver_count ] = calloc(1, sizeof(trigger_driver_s));
	trigger_drivers[ trigger_driver_count ]->trigger_type = JOYSTICK_TRIGGER_TYPE;
	trigger_drivers[ trigger_driver_count++ ]->driver = joystickDriver;
    ESP_LOGI( TAG, "Joystick driver created" );
    start_trigger_driver( joystickDriver, PULSE_MILLIS, "joystick" );
#endif


// IRRecv Driver
#ifdef CONFIG_DRIVER_IRRECV
	TriggerDriver *irrecvDriver = IRRecv_NewDriver(onLocalSignal, CONFIG_DRIVER_IRRECV_GPIO);
	trigger_drivers[ trigger_driver_count ] = calloc(1, sizeof(trigger_driver_s));
	trigger_drivers[ trigger_driver_count ]->trigger_type = IRRECV_TRIGGER_TYPE;
	trigger_drivers[ trigger_driver_count++ ]->driver = irrecvDriver;
	ESP_LOGI(TAG, "IR recv driver created");
	start_trigger_driver( irrecvDriver, PULSE_MILLIS, "irrecv" );

#endif

// Buzzer Driver ------------------------------------------------------------------------------------
#ifdef CONFIG_DRIVER_BUZZER
    ActionDriver *buzzerDriver = Buzzer_NewDriver( LEDC_TIMER_1, LEDC_CHANNEL_3, CONFIG_DRIVER_BUZZER_GPIO );
    start_action_driver( buzzerDriver, PULSE_MILLIS, "buzzer" );
    ESP_LOGI( TAG, "Buzzer driver created" );

#ifdef CONFIG_DRIVER_BUZZER_FREQ_ENABLE
    action_drivers[ action_driver_count ] = calloc( 1, sizeof( action_driver_s ) );
    action_drivers[ action_driver_count ]->action_type = BUZZER_ACTION_TYPE;
    action_drivers[ action_driver_count ]->instance_id = CONFIG_DRIVER_BUZZER_FREQ_INSTANCE_ID;
    action_drivers[ action_driver_count ]->driver = buzzerDriver;
    action_driver_count += 1;

    #ifdef CONFIG_DRIVER_BUZZER_FREQ_ADVERTISE_LOCATION
        uint8_t *buzzerFreqLocationAd;
        uint32_t buzzerFreqLocationAdLength;
        buzzerFreqLocationAd = ProtoGen_NewActionLocationDescriptor( BUZZER_ACTION_TYPE, CONFIG_DRIVER_BUZZER_FREQ_INSTANCE_ID,
            CONFIG_DRIVER_BUZZER_FREQ_ADVERTISE_LOCATION_X * 0.01, CONFIG_DRIVER_BUZZER_FREQ_ADVERTISE_LOCATION_Y * 0.01, CONFIG_DRIVER_BUZZER_FREQ_ADVERTISE_LOCATION_Z * 0.01,
            &buzzerFreqLocationAdLength );
        networkWrite( buzzerFreqLocationAd, buzzerFreqLocationAdLength, PRIORITY_LOW, PERSISTENT);
        ESP_LOGI( TAG, "Set up location advertisement for frequency buzzer: (%i, %i, %i)",
                CONFIG_DRIVER_BUZZER_FREQ_ADVERTISE_LOCATION_X, CONFIG_DRIVER_BUZZER_FREQ_ADVERTISE_LOCATION_Y, CONFIG_DRIVER_BUZZER_FREQ_ADVERTISE_LOCATION_Z );
    #endif

    ESP_LOGI( TAG, "Added buzzer frequency action." );
#endif
#ifdef CONFIG_DRIVER_BUZZER_NOTE_ENABLE
    action_drivers[ action_driver_count ] = calloc( 1, sizeof( action_driver_s ) );
    action_drivers[ action_driver_count ]->action_type = NOTE_PLAYER_ACTION_TYPE;
    action_drivers[ action_driver_count ]->instance_id = CONFIG_DRIVER_BUZZER_NOTE_INSTANCE_ID;
    action_drivers[ action_driver_count ]->driver = buzzerDriver;
    action_driver_count += 1;

    #ifdef CONFIG_DRIVER_BUZZER_NOTE_ADVERTISE_LOCATION
        uint8_t *buzzerNoteLocationAd;
        uint32_t buzzerNoteLocationAdLength;
        buzzerNoteLocationAd = ProtoGen_NewActionLocationDescriptor( NOTE_PLAYER_ACTION_TYPE, CONFIG_DRIVER_BUZZER_NOTE_INSTANCE_ID,
            CONFIG_DRIVER_BUZZER_NOTE_ADVERTISE_LOCATION_X * 0.01, CONFIG_DRIVER_BUZZER_NOTE_ADVERTISE_LOCATION_Y * 0.01, CONFIG_DRIVER_BUZZER_NOTE_ADVERTISE_LOCATION_Z * 0.01,
            &buzzerNoteLocationAdLength );
        networkWrite( buzzerNoteLocationAd, buzzerNoteLocationAdLength, PRIORITY_LOW, PERSISTENT);
        ESP_LOGI( TAG, "Set up location advertisement for note player: (%i, %i, %i)",
                CONFIG_DRIVER_BUZZER_NOTE_ADVERTISE_LOCATION_X, CONFIG_DRIVER_BUZZER_NOTE_ADVERTISE_LOCATION_Y, CONFIG_DRIVER_BUZZER_NOTE_ADVERTISE_LOCATION_Z );
    #endif

    ESP_LOGI( TAG, "Added buzzer note player action." );
#endif
#ifdef CONFIG_DRIVER_BUZZER_TUNE_ENABLE
    action_drivers[ action_driver_count ] = calloc( 1, sizeof( action_driver_s ) );
    action_drivers[ action_driver_count ]->action_type = TUNE_PLAYER_ACTION_TYPE;
    action_drivers[ action_driver_count ]->instance_id = CONFIG_DRIVER_BUZZER_TUNE_INSTANCE_ID;
    action_drivers[ action_driver_count ]->driver = buzzerDriver;
    action_driver_count += 1;

    #ifdef CONFIG_DRIVER_BUZZER_TUNE_ADVERTISE_LOCATION
        uint8_t *buzzerTuneLocationAd;
        uint32_t buzzerTuneLocationAdLength;
        buzzerTuneLocationAd = ProtoGen_NewActionLocationDescriptor( TUNE_PLAYER_ACTION_TYPE, CONFIG_DRIVER_BUZZER_TUNE_INSTANCE_ID,
            CONFIG_DRIVER_BUZZER_TUNE_ADVERTISE_LOCATION_X * 0.01, CONFIG_DRIVER_BUZZER_TUNE_ADVERTISE_LOCATION_Y * 0.01, CONFIG_DRIVER_BUZZER_TUNE_ADVERTISE_LOCATION_Z * 0.01,
            &buzzerTuneLocationAdLength );
        networkWrite( buzzerTuneLocationAd, buzzerTuneLocationAdLength, PRIORITY_LOW, PERSISTENT);
        ESP_LOGI( TAG, "Set up location advertisement for tune player: (%i, %i, %i)",
                CONFIG_DRIVER_BUZZER_TUNE_ADVERTISE_LOCATION_X, CONFIG_DRIVER_BUZZER_TUNE_ADVERTISE_LOCATION_Y, CONFIG_DRIVER_BUZZER_TUNE_ADVERTISE_LOCATION_Z );
    #endif

    ESP_LOGI( TAG, "Added buzzer tune player action." );
#endif
#endif

    networkSetupScan();
    setupAdvertising();

#ifdef CONFIG_TAP_TEST_ADV
    // TEST: Create adverts
    uint8_t *adBytes;
    uint32_t adLength;
    
    // LED Action #1: Blueish
    adBytes = ProtoGen_NewRgbLedAction( 20, 1, 0x00, 0x00, 0xff, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // LED Action #1: Off
    adBytes = ProtoGen_NewRgbLedAction( 21, 1, 0x00, 0x00, 0x00, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    // LED Action #2: Blueish
    adBytes = ProtoGen_NewRgbLedAction( 22, 2, 0x00, 0x00, 0xff, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // LED Action #2: Off
    adBytes = ProtoGen_NewRgbLedAction( 23, 2, 0x00, 0x00, 0x00, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    // LED Action #3: Blueish
    adBytes = ProtoGen_NewRgbLedAction( 24, 3, 0x00, 0x00, 0xff, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // LED Action #3: Off
    adBytes = ProtoGen_NewRgbLedAction( 25, 3, 0x00, 0x00, 0x00, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

   
    /*********************************DEPROVISION*********************************/
    // // LED Action: Blueish
    // adBytes = ProtoGen_NewRgbLedAction( 20, 2, 0x00, 0x00, 0xff, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_HIGH, NON_PERSISTENT, DEPROVISION_FLAG );
    // free( adBytes );
    // // LED Action: Off
    // adBytes = ProtoGen_NewRgbLedAction( 21, 2, 0x00, 0x00, 0x00, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_HIGH, NON_PERSISTENT, DEPROVISION_FLAG );
    // free( adBytes );
    // /*****************************************************************************/

    // /*********************************REPROVISION*********************************/
    // adBytes = ProtoGen_NewRgbLedAction( 40, 2, 0x00, 0x00, 0xff, &adLength );
    // // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // LED Action: Off
    // adBytes = ProtoGen_NewRgbLedAction( 41, 2, 0x00, 0x00, 0x00, &adLength );
    // // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    /*****************************************************************************/
   

	// // Laser Action: on
	// adBytes = ProtoGen_NewLaserAction( 22, 1, 1, &adLength );
	// networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
	// free( adBytes );
	// // Laser Action: off
	// adBytes = ProtoGen_NewLaserAction( 23, 1, 0, &adLength );
	// networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
	// free( adBytes );

	// // IR recv on
	// adBytes = ProtoGen_NewIRRecvTrigger( 30, 1, 0, &adLength );
	// networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
	// free( adBytes );
	// // IR recv off
	// adBytes = ProtoGen_NewIRRecvTrigger( 31, 1, 1, &adLength );
	// networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
	// free( adBytes );

    // Button Trigger: #1 Down
    adBytes = ProtoGen_NewButtonTrigger( 40, 1, 0, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // Button Trigger: #1 Up
    adBytes = ProtoGen_NewButtonTrigger( 41, 1, 1, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    // Button Trigger: #2 Down
    adBytes = ProtoGen_NewButtonTrigger( 42, 2, 0, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // Button Trigger: #2 Up
    adBytes = ProtoGen_NewButtonTrigger( 43, 2, 1, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    // Button Trigger: #3 Down
    adBytes = ProtoGen_NewButtonTrigger( 44, 3, 0, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // Button Trigger: #3 Up
    adBytes = ProtoGen_NewButtonTrigger( 45, 3, 1, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    /********************************NEW PROVISION********************************/
    // // Button Trigger: #1 Down
    // adBytes = ProtoGen_NewButtonTrigger( 40, 2, 0, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT );
    // free( adBytes );
    // // Button Trigger: #1 Up
    // adBytes = ProtoGen_NewButtonTrigger( 41, 2, 1, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT );
    // free( adBytes );
    /*****************************************************************************/

    //---------------------- button 1 -> led 2 -------------------------//
    // Rule: Down -> Blue
    adBytes = ProtoGen_NewRule( 60, 40, 22, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // Rule: Up -> Off
    adBytes = ProtoGen_NewRule( 61, 41, 23, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    //---------------------- button 1 -> led 3 -------------------------//
    // Rule: Down -> Blue
    adBytes = ProtoGen_NewRule( 62, 40, 24, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // Rule: Up -> Off
    adBytes = ProtoGen_NewRule( 63, 41, 25, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    //---------------------- button 2 -> led 1 -------------------------//
    // Rule: Down -> Blue
    adBytes = ProtoGen_NewRule( 70, 42, 20, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // Rule: Up -> Off
    adBytes = ProtoGen_NewRule( 71, 43, 21, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    //---------------------- button 2 -> led 3 -------------------------//
    // Rule: Down -> Blue
    adBytes = ProtoGen_NewRule( 72, 42, 24, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // Rule: Up -> Off
    adBytes = ProtoGen_NewRule( 73, 43, 25, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    //---------------------- button 3 -> led 1 -------------------------//
    // Rule: Down -> Blue
    adBytes = ProtoGen_NewRule( 80, 44, 20, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // Rule: Up -> Off
    adBytes = ProtoGen_NewRule( 81, 45, 21, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    //---------------------- button 3 -> led 2 -------------------------//
    // Rule: Down -> Blue
    adBytes = ProtoGen_NewRule( 82, 44, 22, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // Rule: Up -> Off
    adBytes = ProtoGen_NewRule( 83, 45, 23, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );


    /********************************NEW PROVISION********************************/
    // // Rule: Down -> Blue
    // adBytes = ProtoGen_NewRule( 40, 40, 40, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // Rule: Up -> Off
    // adBytes = ProtoGen_NewRule( 41, 41, 41, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    /*****************************************************************************/

    // // Rule: Down -> laser on
    // adBytes = ProtoGen_NewRule( 22, 20, 22, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // Rule: Up -> laser off
    // adBytes = ProtoGen_NewRule( 23, 21, 23, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );

    // // Rule: IR on -> Blue
    // adBytes = ProtoGen_NewRule( 30, 30, 20, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // Rule: IR off -> Off
    // adBytes = ProtoGen_NewRule( 31, 31, 21, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );

	// // Joystick Trigger button down
	// adBytes = ProtoGen_NewJoystickTrigger(40, 1, 0, 0, 1, 0, &adLength);
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
	// // Joystick Trigger button up
	// adBytes = ProtoGen_NewJoystickTrigger(41, 1, 0, 0, 0, 0, &adLength);
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
	// // Joystick Trigger x > 20
	// adBytes = ProtoGen_NewJoystickTrigger(42, 1, 1, 2, 20, 0, &adLength);
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
	// // Joystick Trigger x < 5
	// adBytes = ProtoGen_NewJoystickTrigger(43, 1, 1, 1, 5, 0, &adLength);
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
	// // Joystick Trigger x home
	// adBytes = ProtoGen_NewJoystickTrigger(44, 1, 1, 5, 10, 11, &adLength);
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
	// // Joystick Trigger y > 20
	// adBytes = ProtoGen_NewJoystickTrigger(45, 1, 2, 2, 20, 0, &adLength);
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
	// // Joystick Trigger y < 5
	// adBytes = ProtoGen_NewJoystickTrigger(46, 1, 2, 1, 5, 0, &adLength);
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
	// // Joystick Trigger y home
	// adBytes = ProtoGen_NewJoystickTrigger(47, 1, 2, 5, 10, 11, &adLength);
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // LED Action: red
    // adBytes = ProtoGen_NewRgbLedAction( 50, 1, 0xff, 0x00, 0x00, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // LED Action: green
    // adBytes = ProtoGen_NewRgbLedAction( 51, 1, 0x00, 0xff, 0x00, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // LED Action: blue
    // adBytes = ProtoGen_NewRgbLedAction( 52, 1, 0x00, 0x00, 0xff, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // LED Action: cyan
    // adBytes = ProtoGen_NewRgbLedAction( 53, 1, 0x00, 0xff, 0xff, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // LED Action: magenta
    // adBytes = ProtoGen_NewRgbLedAction( 54, 1, 0xff, 0x00, 0xff, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // LED Action: yellow
    // adBytes = ProtoGen_NewRgbLedAction( 55, 1, 0xff, 0xff, 0x00, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // Rule: JsHome -> red
    // adBytes = ProtoGen_NewRule( 100, 41, 50, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // adBytes = ProtoGen_NewRule( 101, 44, 50, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // adBytes = ProtoGen_NewRule( 102, 47, 50, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // Rule: JsBdown -> green
    // adBytes = ProtoGen_NewRule( 110, 40, 51, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // Rule: JsBxpos -> blue
    // adBytes = ProtoGen_NewRule( 111, 42, 52, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // Rule: JsBxneg -> cyan
    // adBytes = ProtoGen_NewRule( 112, 43, 53, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // Rule: JsBypos -> magenta
    // adBytes = ProtoGen_NewRule( 113, 45, 54, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );
    // // Rule: JsByneg -> yellow
    // adBytes = ProtoGen_NewRule( 114, 46, 55, &adLength );
    // networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    // free( adBytes );

#endif

    // ESP_LOGI( TAG, "Size of available heap: %d", esp_get_free_heap_size() );
    // ESP_LOGI( TAG, "Size of available heap: %d", esp_get_minimum_free_heap_size() );
    // esp_chip_info_t chip_info;
    // esp_chip_info(&chip_info);
    // esp_chip_model_t chip_model = chip_info.model;
    // ESP_LOGI( TAG, "Chip info:");
    // ESP_LOGI( TAG, "Chip model: %d", chip_model);
    // ESP_LOGI( TAG, "Chip features: %d", chip_info.features);
    // ESP_LOGI( TAG, "Chip cores: %d", chip_info.cores);
    // ESP_LOGI( TAG, "Chip revision: %d", chip_info.revision);

}

/***********************************************************************/

