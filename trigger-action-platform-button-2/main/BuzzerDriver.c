/*
Copyright (c) 2019 Teemu Kärkkäinen

ESP32 library for trigger-action programming
*/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <stdint.h>
#include <string.h>
#include <esp_log.h>

#include "BuzzerDriver.h"
#include "BuzzerTunes.h"
#include "trigger_action.h"
#include "sdnv.h"

/***********************************************************************/
/* Types and Definitions */
/***********************************************************************/
#define BUZZER_ACTION_LIST_SIZE     (64)
#define BUZZER_DUTY_MAX             (2047)

#define BUZZER_ACTION_TYPE_FREQ          (0)
#define BUZZER_ACTION_TYPE_NOTE          (1)
#define BUZZER_ACTION_TYPE_TUNE          (2)

static const char* TAG = "BuzzerDriver";

typedef struct {
    uint8_t type;
    void *parameters;
} TaskQueueItem;

typedef struct {
    uint32_t actionId;
    TaskQueueItem *task;
} ActionListItem;

typedef struct {
    uint16_t frequency;
    uint8_t duration;
} BuzzerParams;

typedef struct {
    uint8_t bpm;
    uint8_t *notes;
    uint8_t noteCount;
} NotePlayerParams;

typedef struct {
    uint32_t tuneId;
} TunePlayerParams;

typedef struct {
    ledc_timer_config_t timer;
    ledc_channel_config_t channel;
    QueueHandle_t taskQueue;
    ActionListItem* actionList[ BUZZER_ACTION_LIST_SIZE ];
    uint16_t actionListCount;
} BuzzerDriver;

// API functions
void Buzzer_AddAction( void *state, action_definition_s *action );
void Buzzer_RemoveAction( void *state, action_definition_s *action );
void Buzzer_TakeAction( void *state, uint32_t actionId );
void Buzzer_TaskLoop( void *state, uint32_t pulseMillis );
void Buzzer_Free( ActionDriver *driver );

// Private functions
static BuzzerParams* parseBuzzerParams( uint8_t const *buffer, uint8_t const bufferLength );
static NotePlayerParams* parseNotePlayerParams( uint8_t const *buffer, uint8_t const bufferLength );
static TunePlayerParams* parseTunePlayerParams( uint8_t const *buffer, uint8_t const bufferLength );
static void freeBuzzerParams( BuzzerParams *params );
static void freeNotePlayerParams( NotePlayerParams *params );
static void freeTunePlayerParams( TunePlayerParams *params );
static BuzzerParams* copyBuzzerParams( BuzzerParams const *params );
static NotePlayerParams* copyNotePlayerParams( NotePlayerParams const *params );
static TunePlayerParams* copyTunePlayerParams( TunePlayerParams *params );
static TaskQueueItem* copyTaskQueueItem( TaskQueueItem* item );
static void freeTaskQueueItem( TaskQueueItem* item );
static void executeTask( BuzzerDriver *buzzerDriver, TaskQueueItem *task );
static void executeBuzzerTask( BuzzerDriver *buzzerDriver, BuzzerParams *params );
static void executeNotePlayerTask( BuzzerDriver *buzzerDriver, NotePlayerParams *params );
static void executeTunePlayerTask( BuzzerDriver *buzzerDriver, TunePlayerParams *params );
static void stopBuzzer( BuzzerDriver *buzzerDriver );
static void startBuzzer( BuzzerDriver *buzzerDriver );
static void setTone( BuzzerDriver *buzzerDriver, uint16_t frequency, uint32_t durationMillis );
/***********************************************************************/



/***********************************************************************/
/* Driver API */
/***********************************************************************/

extern ActionDriver* Buzzer_NewDriver( ledc_timer_t timer, ledc_channel_t channel, uint8_t gpio ) {
    ActionDriver *driver = calloc( 1, sizeof( ActionDriver ) );
    BuzzerDriver *buzzerDriver = calloc( 1, sizeof( BuzzerDriver ) );

    // Action driver setup
    driver->state = buzzerDriver;
    driver->addAction = Buzzer_AddAction;
    driver->removeAction = Buzzer_RemoveAction;
    driver->takeAction = Buzzer_TakeAction;
    driver->taskLoop = Buzzer_TaskLoop;
    driver->free = Buzzer_Free;

    // Buzzer driver setup
    buzzerDriver->timer.duty_resolution = LEDC_TIMER_12_BIT;
    buzzerDriver->timer.freq_hz = 1000;
    buzzerDriver->timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    buzzerDriver->timer.timer_num = timer;

    buzzerDriver->channel.channel    = channel;
    buzzerDriver->channel.duty       = BUZZER_DUTY_MAX;
    buzzerDriver->channel.gpio_num   = gpio;
    buzzerDriver->channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    buzzerDriver->channel.timer_sel  = timer;

    buzzerDriver->taskQueue = xQueueCreate( 20, sizeof( TaskFunction_t* ) );

    // Initialize
    esp_err_t result;
    ESP_LOGI( TAG, "Initializing timer." );
    result = ledc_timer_config( &( buzzerDriver->timer ) );
    if ( result != ESP_OK ) ESP_LOGI( TAG, "Failed to configure timer (%d)", result );

    ESP_LOGI( TAG, "Initializing channel." );
    result = ledc_channel_config( &( buzzerDriver->channel ) );
    if ( result != ESP_OK ) ESP_LOGI( TAG, "Failed to configure channel (%d)", result );

    stopBuzzer( buzzerDriver );

    return driver;
}

void Buzzer_TaskLoop( void *state, uint32_t pulseMillis ) {
    ESP_LOGI( TAG, "TaskLoop()" );
    BuzzerDriver *driver = state;
    
    TaskQueueItem *task;
    while ( true ) {
        if ( xQueueReceive( driver->taskQueue, &task, ( 1000 / portTICK_PERIOD_MS ) ) == pdTRUE ) {
            ESP_LOGI( TAG, "Executing task" );
            executeTask( driver, task );
            freeTaskQueueItem( task );
            ESP_LOGI( TAG, "Task execution done" );
        }
    }
}

void Buzzer_AddAction( void *state, action_definition_s *action ) {
    ESP_LOGI( TAG, "Buzzer_AddAction( %u )", action->action_id );
    BuzzerDriver *buzzerDriver = state;
    TaskQueueItem *task = calloc( 1, sizeof( TaskQueueItem ) );

    // Parse parameters for the supported types
    if ( action->action_type == BUZZER_ACTION_TYPE ) {
        BuzzerParams *buzzerParams = parseBuzzerParams( action->params, action->params_length );
        if ( buzzerParams != NULL ) {
            task->parameters = buzzerParams;
            task->type = BUZZER_ACTION_TYPE_FREQ;
        } else {
            free( task );
            return;
        }
        ESP_LOGI( TAG, "Buzzer action: frequency = %u", buzzerParams->frequency );
    } else if ( action->action_type == NOTE_PLAYER_ACTION_TYPE ) {
        NotePlayerParams *notePlayerParams = parseNotePlayerParams( action->params, action->params_length );
        if ( notePlayerParams != NULL ) {
            task->parameters = notePlayerParams;
            task->type = BUZZER_ACTION_TYPE_NOTE;
        } else {
            free( task );
            return;
        }
        ESP_LOGI( TAG, "Note action: noteCount = %u", notePlayerParams->noteCount );
    } else if ( action->action_type == TUNE_PLAYER_ACTION_TYPE ) {
        TunePlayerParams *tunePlayerParams = parseTunePlayerParams( action->params, action->params_length );
        if ( tunePlayerParams != NULL ) {
            task->parameters = tunePlayerParams;
            task->type = BUZZER_ACTION_TYPE_TUNE;
        } else {
            free( task );
            return;
        }
        ESP_LOGI( TAG, "Tune action: tuneId = %u", tunePlayerParams->tuneId );
    } else {
        ESP_LOGI( TAG, "Unsupported action type (%u). Ignoring.", action->action_type );
        free( task );
        return;
    }

    ActionListItem *actionListItem = calloc( 1, sizeof( ActionListItem ) );
    actionListItem->actionId = action->action_id;
    actionListItem->task = task;
    buzzerDriver->actionList[ buzzerDriver->actionListCount++ ] = actionListItem;

    ESP_LOGI( TAG, "Added new action. ID = %u, type = %u.", action->action_id, action->action_type );
}

void Buzzer_RemoveAction( void *state, action_definition_s *action ) {
    ESP_LOGI( TAG, "Buzzer_RemoveAction( %u )", action->action_id );
    BuzzerDriver *buzzerDriver = state;

    for ( int i = 0; i < buzzerDriver->actionListCount; i++ ) {
        ActionListItem *listItem = buzzerDriver->actionList[ i ];
        if ( listItem->actionId == action->action_id ) {
            buzzerDriver->actionListCount -= 1;
            buzzerDriver->actionList[ i ] = buzzerDriver->actionList[ buzzerDriver->actionListCount ];
            i--; // The moved item may be the same id
        }
    }
}

void Buzzer_TakeAction( void *state, uint32_t actionId ) {
    ESP_LOGI( TAG, "Buzzer_TakeAction( %u )", actionId );
    BuzzerDriver *buzzerDriver = state;

    // Find the action
    for ( int i = 0; i < buzzerDriver->actionListCount; i++ ) {
        ActionListItem *listItem = buzzerDriver->actionList[ i ];
        if ( listItem->actionId == actionId ) {
            // Create a copy of the task. This is needed for thread safety.
            TaskQueueItem *task = copyTaskQueueItem( listItem->task );

            // Queue the task
            if ( xQueueSendToBack( buzzerDriver->taskQueue, &( task ), 1000 / portTICK_PERIOD_MS ) != pdTRUE ) {
                ESP_LOGE( TAG, "Failed to add task to the queue. Aborting" );
                return;
            }

            ESP_LOGI( TAG, "Queued task for action %u.", actionId );
        }
    }   
}

void Buzzer_Free( ActionDriver *driver ) {
    ESP_LOGE( TAG, "Buzzer_Free not implemented." );
}

/***********************************************************************/


/***********************************************************************/
/* Task Execution */
/***********************************************************************/
static void executeTask( BuzzerDriver *buzzerDriver, TaskQueueItem *task ) {
    ESP_LOGI( TAG, "executeTask(): taskType = %u", task->type );
    switch ( task->type ) {
        case BUZZER_ACTION_TYPE_FREQ:
            executeBuzzerTask( buzzerDriver, task->parameters );
            break;

        case BUZZER_ACTION_TYPE_NOTE:
            executeNotePlayerTask( buzzerDriver, task->parameters );
            break;

        case BUZZER_ACTION_TYPE_TUNE:
            executeTunePlayerTask( buzzerDriver, task->parameters );
            break;

        default:
            ESP_LOGE( TAG, "Invalid action type (%u). Cannot execute.", task->type );
            return;
    }
}

static void executeBuzzerTask( BuzzerDriver *buzzerDriver, BuzzerParams *params ) {
    uint32_t durationMillis = params->duration * 10;
    startBuzzer( buzzerDriver );
    setTone( buzzerDriver, params->frequency, durationMillis );
    stopBuzzer( buzzerDriver );
}

static void executeNotePlayerTask( BuzzerDriver *buzzerDriver, NotePlayerParams *params ) {
    // uint8_t bpm;
    // uint8_t *notes;
    // uint8_t noteCount;


}

static void executeTunePlayerTask( BuzzerDriver *buzzerDriver, TunePlayerParams *params ) {
    ESP_LOGI( TAG, "executeTunePlayerTask(): tuneId = %u", params->tuneId );
    BuzzerTune *tune = NULL;
    for ( int i = 0; i < BUZZER_TUNE_COUNT; i++ ) {
        BuzzerTune *candidate = &( BUZZER_TUNES[ i ] );
        if ( candidate->tuneId == params->tuneId ) {
            ESP_LOGI( TAG, "Found tune." );
            tune = candidate;
            break;
        }
    }

    if ( tune == NULL ) {
        ESP_LOGI( TAG, "Couldn't find tune %u.", params->tuneId );
        return;
    }

    // Calculate timings
    uint32_t beatDuration = 60000 / tune->bpm;
    uint32_t pauseDuration = 0.2 * beatDuration;
    pauseDuration = ( pauseDuration < 10 ) ? ( 10 ) : ( pauseDuration );
    uint32_t noteDuration = beatDuration - pauseDuration;
    noteDuration = ( noteDuration < 10 ) ? ( 10 ) : ( noteDuration );

    ESP_LOGI( TAG, "Calculated timings: note = %u, pause = %u", noteDuration, pauseDuration );

    // Play the tune
    startBuzzer( buzzerDriver );
    for ( int i = 0; i < tune->noteCount; i++ ) {
        uint16_t frequency = tune->notes[ 2 * i ];
        uint16_t beats = tune->notes[ 2 * i + 1 ];

        uint32_t durationMillis = ( beats - 1 ) * beatDuration + noteDuration;

        ESP_LOGI( TAG, "Setting frequency: %u, duration: %u", frequency, durationMillis );
        setTone( buzzerDriver, frequency, durationMillis );
        stopBuzzer( buzzerDriver );
        vTaskDelay( pauseDuration / portTICK_PERIOD_MS );
        startBuzzer( buzzerDriver );
    }
    stopBuzzer( buzzerDriver );
}
/***********************************************************************/


/***********************************************************************/
/* Private - Tone generation */
/***********************************************************************/
static void stopBuzzer( BuzzerDriver *buzzerDriver ) {
    ledc_stop( LEDC_HIGH_SPEED_MODE, buzzerDriver->channel.channel, 0 );
    ledc_timer_pause( LEDC_HIGH_SPEED_MODE, buzzerDriver->timer.timer_num );
}

static void startBuzzer( BuzzerDriver *buzzerDriver ) {
    ledc_timer_resume( LEDC_HIGH_SPEED_MODE, buzzerDriver->timer.timer_num );
    ledc_update_duty( LEDC_HIGH_SPEED_MODE, buzzerDriver->channel.channel );
}

static void setTone( BuzzerDriver *buzzerDriver, uint16_t frequency, uint32_t durationMillis ) {
    esp_err_t song_err = ledc_set_freq( buzzerDriver->channel.speed_mode, buzzerDriver->channel.timer_sel, frequency );
    if ( song_err == ESP_ERR_INVALID_ARG ) {
        ESP_LOGI( TAG, "ledc_set_freg -> ESP_ERR_INVALID_ARG" );
    } else if ( song_err == ESP_FAIL ) {
        ESP_LOGI( TAG, "ledc_set_freg -> ESP_FAIL" );
    }
    vTaskDelay( durationMillis / portTICK_PERIOD_MS );
}
/***********************************************************************/


/***********************************************************************/
/* Parameter handling */
/***********************************************************************/
static BuzzerParams* parseBuzzerParams( uint8_t const *buffer, uint8_t const bufferLength ) {
    if ( bufferLength != 3 ) {
        ESP_LOGI( TAG, "Malformed buzzer action parameters (length = %u).", bufferLength );
        return NULL;
    }

    BuzzerParams *buzzerParams = calloc( 1, sizeof( BuzzerParams ) );
    buzzerParams->frequency = ( buffer[ 0 ] << 8 | buffer[ 1 ] );
    buzzerParams->duration = buffer[ 3 ];

    return buzzerParams;
}

static BuzzerParams* copyBuzzerParams( BuzzerParams const *params ) {
    BuzzerParams *copyParams = calloc( 1, sizeof( BuzzerParams ) );
    memcpy( copyParams, params, sizeof( BuzzerParams ) );
    return copyParams;
}

static void freeBuzzerParams( BuzzerParams *params ) {
    free( params );
}

static NotePlayerParams* parseNotePlayerParams( uint8_t const *buffer, uint8_t const bufferLength ) {
    if ( bufferLength < 2 ) {
        ESP_LOGI( TAG, "Malformed note player action parameters (length = %u).", bufferLength );
        return NULL;
    }

    NotePlayerParams *notePlayerParams = calloc( 1, sizeof( NotePlayerParams ) );
    notePlayerParams->bpm = buffer[ 0 ];
    notePlayerParams->noteCount = ( bufferLength - 1 );
    notePlayerParams->notes = calloc( notePlayerParams->noteCount, 1 );
    memcpy( notePlayerParams->notes, ( buffer + 1 ), notePlayerParams->noteCount );

    return notePlayerParams;
}

static NotePlayerParams* copyNotePlayerParams( NotePlayerParams const *params ) {
    NotePlayerParams *copyParams = calloc( 1, sizeof( NotePlayerParams ) );
    copyParams->notes = calloc( params->noteCount, 1 );
    copyParams->noteCount = params->noteCount;
    memcpy( copyParams->notes, params->notes, params->noteCount );
    return copyParams;
}

static void freeNotePlayerParams( NotePlayerParams *params ) {
    free( params->notes );
    free( params );
}

static TunePlayerParams* parseTunePlayerParams( uint8_t const *buffer, uint8_t const bufferLength ) {
    if ( bufferLength < 1 ) {
        ESP_LOGI( TAG, "Malformed tune player action parameters (length = %u).", bufferLength );
        return NULL;
    }

    TunePlayerParams *tunePlayerParams = calloc( 1, sizeof( TunePlayerParams ) );
    
    uint32_t pos = 0;
    SdnvDecodeResult tuneId = SDNV_Decode64( buffer + pos, bufferLength );
    if ( tuneId.byteCount <= 0 ) {
        ESP_LOGI( TAG, "Couldn't parse tune id from action definition message" );
        return NULL;
    }
    tunePlayerParams->tuneId = tuneId.value.u64;

    return tunePlayerParams;
}

static TunePlayerParams* copyTunePlayerParams( TunePlayerParams *params ) {
    TunePlayerParams *copyParams = calloc( 1, sizeof( TunePlayerParams ) );
    memcpy( copyParams, params, sizeof( TunePlayerParams ) );
    return copyParams;
}

static void freeTunePlayerParams( TunePlayerParams *params ) {
    free( params );
}

static TaskQueueItem* copyTaskQueueItem( TaskQueueItem* item ) {
    TaskQueueItem *copy = calloc( 1, sizeof( TaskQueueItem ) );
    copy->type = item->type;

    switch ( item->type ) {
        case BUZZER_ACTION_TYPE_FREQ:
            copy->parameters = copyBuzzerParams( item->parameters );
            break;

        case BUZZER_ACTION_TYPE_NOTE:
            copy->parameters = copyNotePlayerParams( item->parameters );
            break;

        case BUZZER_ACTION_TYPE_TUNE:
            copy->parameters = copyTunePlayerParams( item->parameters );
            break;

        default:
            ESP_LOGE( TAG, "Invalid action type (%u)", item->type );
            return NULL;
    }

    return copy;
}

static void freeTaskQueueItem( TaskQueueItem* item ) {
    switch ( item->type ) {
        case BUZZER_ACTION_TYPE_FREQ:
            freeBuzzerParams( item->parameters );
            break;

        case BUZZER_ACTION_TYPE_NOTE:
            freeNotePlayerParams( item->parameters );
            break;

        case BUZZER_ACTION_TYPE_TUNE:
            freeTunePlayerParams( item->parameters );
            break;

        default:
            ESP_LOGE( TAG, "Invalid action type (%u)", item->type );
    }
    free( item );
}
/***********************************************************************/



