/*
Copyright (c) 2019 Teemu Kärkkäinen

Driver for the MBP180 sensor for use in trigger-action programming.
*/


/***********************************************************************/
/* Includes */
/***********************************************************************/
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_timer.h>
#include <esp_log.h>

#include "Bmp180Driver.h"
#include "trigger_driver.h"
#include "bmp180.h"
/***********************************************************************/


/***********************************************************************/
/* Types */
/***********************************************************************/
typedef struct {
    trigger_definition_s const *trigger;
    uint8_t operator;
    float operand1;
    float operand2;
} bmp_180_condition_s;

struct bmp_180_driver {
    TriggerSignalCallback onTriggerSignal;
    
    /** Protects the condition list. */
    SemaphoreHandle_t conditionListMutex;

    /** List of conditions to evaluate. */
    bmp_180_condition_s *conditionList;
    uint32_t const conditionListLength;
    uint32_t conditionListPos;

    i2c_port_t i2cPort;
    bmp180_eeprom_t eeprom;
};

/** Used to convert bytes to float. */
typedef union {
    float f;
    uint64_t i;
} ToFloat;
/***********************************************************************/

/***********************************************************************/
/* Constants */
/***********************************************************************/
static const char* TAG = "Bmp180Driver";
/***********************************************************************/


/***********************************************************************/
/* Internal functions */
/***********************************************************************/
static bool evaluateCondition( bmp_180_condition_s *condition, float value );

static float readFloat( uint8_t *buffer ) {
    ToFloat f;
    f.i = ( buffer[ 0 ] << 24 ) | ( buffer[ 1 ] << 16 ) | ( buffer[ 2 ] << 8 ) | ( buffer[ 3 ] );
    return f.f;
}

/** Parses the condition payload and sets the operator and operand values in the struct. */
static void parseCondition( bmp_180_condition_s *condition ) {
    if ( condition->trigger->params_length < 1 ) {
        ESP_LOGE( TAG, "Invalid params length (%u)", condition->trigger->params_length );
        condition->operator = BMP180DR_CONDITION_OP_ERROR;
        return;
    }

    // Parse the operator
    condition->operator = condition->trigger->params[ 0 ];

    // Parse the operand(s)
    switch ( condition->operator ) {
        case BMP180DR_CONDITION_OP_GT: // greather-than, one operand
        case BMP180DR_CONDITION_OP_LT: // less-than, one operand
            if ( condition->trigger->params_length < 5 ) {
                ESP_LOGE( TAG, "OP_GT/LT: Invalid params length (%u)", condition->trigger->params_length );
                condition->operator = BMP180DR_CONDITION_OP_ERROR;
                return;
            }
            condition->operand1 = readFloat( ( condition->trigger->params + 1 ) );
            break;

        case BMP180DR_CONDITION_OP_RG: // range, two operands
            if ( condition->trigger->params_length < 9 ) {
                ESP_LOGE( TAG, "OP_RG: Invalid params length (%u)", condition->trigger->params_length );
                condition->operator = BMP180DR_CONDITION_OP_ERROR;
                return;
            }
            condition->operand1 = readFloat( ( condition->trigger->params + 1 ) );
            condition->operand2 = readFloat( ( condition->trigger->params + 1 + 4 ) );
            break;

        default:
            ESP_LOGE( TAG, "Invalid operator (%u)", condition->operator );
            condition->operator = BMP180DR_CONDITION_OP_ERROR;
            return;
    }
}

static Bmp180Driver* newDriver( TriggerSignalCallback onTriggerSignal, i2c_port_t i2cPort ) {
    Bmp180Driver *driver = malloc( sizeof( Bmp180Driver ) );
    driver->onTriggerSignal = onTriggerSignal;
    driver->conditionListMutex = xSemaphoreCreateMutex();
    driver->conditionList = malloc( BMP180DR_CONDITION_LIST_SIZE * sizeof( bmp_180_condition_s ) );
    *( ( uint32_t* )( &( driver->conditionListLength ) ) ) = BMP180DR_CONDITION_LIST_SIZE;
    driver->conditionListPos = 0;
    
    driver->i2cPort = i2cPort;
    esp_err_t result = bmp180_read_eeprom( I2C_NUM_0, &( driver->eeprom ) );
    if ( result != ESP_OK ) {
        ESP_LOGI( TAG, "Failed to read EEPROM (%d)", result );
        return NULL; // This is leaking the driver, but at this point the whole thing is useless anyway
    }

    ESP_LOGI( TAG, "Read EEPROM: ac1 = %d,  ac2 = %d,  ac3 = %d,  ac4 = %u,  ac5 = %u,  ac6 = %u,  b1 = %d,  b2 = %d,  mb = %d,  mc = %d,  md = %d",
        driver->eeprom.ac1, driver->eeprom.ac2, driver->eeprom.ac3, driver->eeprom.ac4, driver->eeprom.ac5, driver->eeprom.ac6, driver->eeprom.b1, driver->eeprom.b2, driver->eeprom.mb, driver->eeprom.mc, driver->eeprom.md );

    // TODO: Check that the creation fully succeeded,
    // roll back and return NULL if not.
    return driver;
}

static bool evaluateCondition( bmp_180_condition_s *condition, float value ) {
    switch ( condition->operator ) {
        case BMP180DR_CONDITION_OP_GT: return ( value > condition->operand1 );
        case BMP180DR_CONDITION_OP_LT: return ( value < condition->operand1 );
        case BMP180DR_CONDITION_OP_RG: return ( value >= condition->operand1 && value <= condition->operand2 );
        default: ESP_LOGI( TAG, "Unknown condition operator (%d)", condition->operator );
    }
    return false;
}
/***********************************************************************/


/***********************************************************************/
/* API implementation */
/***********************************************************************/
void bmp180dr_task_function( void* state, uint32_t pulseMillis ) {
    ESP_LOGI( TAG, "Starting task loop." );

    Bmp180Driver *bmpDriver = state;
    int32_t temperature;
    float temperature_f;
    int64_t pressure;
    float pressure_f;
    while ( true ) {
        int64_t start = esp_timer_get_time();

        // Read temperature and pressure
        esp_err_t result;
        result = bmp180_read_true_temperature_pressure( bmpDriver->i2cPort, bmpDriver->eeprom, BMP180_SAMPLING_SINGLE, &temperature, &pressure );
        temperature_f = ( float )( temperature / 10.0 );
        pressure_f = ( float )( pressure );

        // Evaluate conditions and create trigger signals
        if ( result == ESP_OK ) {
            xSemaphoreTake( bmpDriver->conditionListMutex, portMAX_DELAY );
            {
                for ( int i = 0; i < bmpDriver->conditionListPos; i++ ) {
                    bmp_180_condition_s *condition = ( bmpDriver->conditionList + i );
                    
                    switch ( condition->trigger->trigger_type ) {
                        case TEMPERATURE_TRIGGER_TYPE:
                            if ( evaluateCondition( condition, temperature_f ) ) {
//                                if ( condition->operator == BMP180DR_CONDITION_OP_GT )
//                                    ESP_LOGI( TAG, "Temperature triggered: %f > %f", temperature_f, condition->operand1 );
//                                else if ( condition->operator == BMP180DR_CONDITION_OP_LT )
//                                    ESP_LOGI( TAG, "Temperature triggered: %f < %f", temperature_f, condition->operand1 );
//                                else if ( condition->operator == BMP180DR_CONDITION_OP_RG )
//                                    ESP_LOGI( TAG, "Temperature triggered: %f < %f <= %f", condition->operand1, temperature_f, condition->operand2 );

                                bmpDriver->onTriggerSignal( condition->trigger->trigger_id );
                            }
                            break;
                        case PRESSURE_TRIGGER_TYPE:
                            if ( evaluateCondition( condition, pressure_f ) ) {
                                bmpDriver->onTriggerSignal( condition->trigger->trigger_id );
                            }
                            break;
                        default:
                            ESP_LOGI( TAG, "Unknown trigger type (%u)", condition->trigger->trigger_type );
                    }
                }
            }
            xSemaphoreGive( bmpDriver->conditionListMutex );
        } else {
            ESP_LOGI( TAG, "Failed to read sensor (%d).", result );
        }

        // Delay to match the requested pulse rate
        int64_t end = esp_timer_get_time();
        int64_t delayMillis = pulseMillis - ( ( end - start ) / 1000 );
        delayMillis = ( delayMillis > 0 ) ? ( delayMillis ) : ( 0 );
//        ESP_LOGI( TAG, "Sleeping %lld millis.", delayMillis );
        vTaskDelay( delayMillis / portTICK_RATE_MS );
    }

    ESP_LOGI( TAG, "Exiting task loop." );
}

void bmp180dr_free_driver( TriggerDriver *driver ) {
    TriggerDriver *triggerDriver = driver;
    Bmp180Driver *bmpDriver = triggerDriver->state;
    vSemaphoreDelete( bmpDriver->conditionListMutex );
    free( bmpDriver->conditionList );
    free( bmpDriver );
    free( triggerDriver );
}

void bmp180dr_add_condition( void *state, trigger_definition_s const *trigger ) {
    // Only process temperature and pressure triggers
    if ( trigger->trigger_type != TEMPERATURE_TRIGGER_TYPE
           && trigger->trigger_type != PRESSURE_TRIGGER_TYPE ) {
        return;
    }

    Bmp180Driver* driver = state;

    // Add the condition
    xSemaphoreTake( driver->conditionListMutex, portMAX_DELAY );
    {
        // Preconditions
        if ( driver->conditionListPos == driver->conditionListLength ) {
            ESP_LOGE( TAG, "BMP180 ERROR: Condition list is full." );
            return;
        }

        // Add to the list and parse
        bmp_180_condition_s *conditionStruct = ( driver->conditionList + driver->conditionListPos );
        conditionStruct->trigger = trigger;
        parseCondition( conditionStruct );
        if ( conditionStruct->operator == BMP180DR_CONDITION_OP_ERROR ) {
            ESP_LOGE( TAG, "BMP180 ERROR: Couldn't parse condition." );
            return;
//        } else {
//            ESP_LOGI( TAG, "Added condition: operator=%u, operand1=%f, operand2=%f",
//                    conditionStruct->operator, conditionStruct->operand1, conditionStruct->operand2 );
        }
        driver->conditionListPos += 1;
    }
    xSemaphoreGive( driver->conditionListMutex );
}

void bmp180dr_remove_condition( void *state, trigger_definition_s const *trigger ) {
    Bmp180Driver* driver = state;

    xSemaphoreTake( driver->conditionListMutex, portMAX_DELAY );
    {
        for ( int i = 0 ; i < driver->conditionListPos; i++ ) {
            if ( driver->conditionList[ i ].trigger->trigger_id == trigger->trigger_id ) {
                driver->conditionListPos -= 1;
                driver->conditionList[ i ] = driver->conditionList[ driver->conditionListPos ];
            }
        }
    }
    xSemaphoreGive( driver->conditionListMutex );
}

TriggerDriver* bmp180dr_new_driver( TriggerSignalCallback onTriggerSignal, i2c_port_t i2cPort ) {
    Bmp180Driver *bmpDriver = newDriver( onTriggerSignal, i2cPort );

    TriggerDriver *driver = malloc( sizeof( TriggerDriver ) );
    driver->state = bmpDriver;

    driver->addTrigger = bmp180dr_add_condition;
    driver->removeTrigger = bmp180dr_remove_condition;
    driver->free = bmp180dr_free_driver;
    driver->taskLoop = bmp180dr_task_function;

    return driver;
}
/***********************************************************************/



