#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/adc.h"


/********************************************************************************/
/* Types and definitions */
/********************************************************************************/
const static char *TAG = "vlc_rx";

#define AVRG_ALPHA  (0.01)

#define LOG_PERIOD_MICROS   (5000000)

#define PROTOCOL_PREABLE    (0x99)

#define PARSER_PREAMBLE         (0)
#define PARSER_LENGTH           (1)
#define PARSER_PAYLOAD          (2)

#define delay 100
#define timing_error 30

typedef void ( *FrameCallback )( uint8_t*, uint16_t );

typedef struct {
    FrameCallback on_receive;

    adc_unit_t adc_unit;
    adc_channel_t adc_channel;
    adc_atten_t adc_attenuation;

    int64_t delay_micros;
    int16_t sender_period;      // How often sender changes levels (micro secs)
    int16_t max_timing_error;   // How much timing error is allowed when detecting edges (micro secs)

    float average_alpha;
    float average_voltage;
    bool previous_level;
    int64_t previous_edge_timestamp;

    // Parser state
    uint8_t parser_state;
    uint8_t preamble_buffer;
    uint8_t frame_length;
    uint8_t frame_length_counter;
    uint8_t *frame_buffer;
    uint16_t frame_buffer_size;
    uint16_t frame_bit_counter;
} params_s;

static void rx_task( void *void_params );
static void setup_adc( params_s *params );
static void parser_fsm( params_s *params, bool edge );
static void reset_parser_fsm( params_s *params );
static void receive_callback( uint8_t *payload, uint16_t payload_length );
/********************************************************************************/


/********************************************************************************/
/* Vars */
/********************************************************************************/
uint32_t counter_bytes = 0;
uint32_t counter_syncs = 0;
int64_t counter_start;
uint32_t counter_sync_states[ 3 ];
uint32_t counter_preambles = 0;
int64_t counter_log_start;
uint8_t counter_mc = 0;
uint8_t edge_counter = 0;
/********************************************************************************/


/********************************************************************************/
/* Rx Task */
/********************************************************************************/
static void rx_task( void *void_params ) {
    params_s *params = void_params;

    // Take sample
    int32_t sample = 0;
    int64_t timestamp = esp_timer_get_time();
    if ( params->adc_unit == ADC_UNIT_1 ) {
        sample = adc1_get_raw( ( adc1_channel_t )params->adc_channel );
        // counter_mc++;
    } 
    else 
    {
        adc2_get_raw( ( adc2_channel_t )params->adc_channel, ADC_WIDTH_BIT_12, &sample );
    }

    // Update average voltage level
    // params->average_voltage = params->average_alpha * sample + ( 1.0 - params->average_alpha ) * params->average_voltage;
    params->average_voltage = 1000;
    // Interpret the level (high/low) and a change in level (edge)
    // ESP_LOGI( TAG, "Sample: %d.", sample);
    bool level = ( sample > params->average_voltage ) ? 1 : 0;
    if ( params->previous_level != level /*&& counter_mc > 1*/) {
        bool edge = ( params->previous_level < level ) ? 0 : 1;
        int64_t expected_time = params->previous_edge_timestamp + 2 * params->sender_period;
        if ( ( timestamp > expected_time - params->max_timing_error ) && ( timestamp < expected_time + params->max_timing_error ) ) {
            // Correct detection -> process
            params->previous_edge_timestamp = timestamp;
            // if (edge_counter > 10)
            // ESP_LOGI( TAG, "previous level: %d.", params->previous_level);
            // ESP_LOGI( TAG, "current level: %d.", sample);
            // ESP_LOGI( TAG, "Correct detection. Timestamp: %lld, expected timestamp: %lld.", timestamp, expected_time);
            parser_fsm( params, edge );
        } else if ( timestamp > expected_time + params->max_timing_error ) {
            // Out-of-sync -> reset and process
            params->previous_edge_timestamp = timestamp;
            counter_syncs += 1;
            counter_sync_states[ params->parser_state ] += 1;
            reset_parser_fsm( params );
            // ESP_LOGI( TAG, "Out of sync detection. Timestamp: %lld, expected timestamp: %lld.", timestamp, expected_time);
            parser_fsm( params, edge );
        }
        // counter_mc = 0;
        // edge_counter++;
    }

    // Update state
    params->previous_level = level;
    // Logging
    // if ( timestamp > counter_log_start + LOG_PERIOD_MICROS ) {
    //     float bytes_per_second = counter_bytes / ( ( timestamp - counter_log_start ) / 1000000.0 );
    //     float syncs_per_second = counter_syncs / ( ( timestamp - counter_log_start ) / 1000000.0 );
    //     float preambles_per_second = counter_preambles / ( ( timestamp - counter_log_start ) / 1000000.0 );
    //     ESP_LOGI( TAG, "Goodput: %f B/s", bytes_per_second );
    //     ESP_LOGI( TAG, "Out-of-sync: count=%d, rate=%f sync/s, state_counts=(%u, %u, %u)",
    //             counter_syncs, syncs_per_second,
    //             counter_sync_states[ 0 ], counter_sync_states[ 1 ], counter_sync_states[ 2 ] );
    //     ESP_LOGI( TAG, "Premables: count=%d, rate=%f", counter_preambles, preambles_per_second );
    //     counter_log_start = timestamp;
    //     counter_bytes = 0;
    //     counter_syncs = 0;
    //     counter_preambles = 0;
    //     memset( counter_sync_states, 0, sizeof( counter_sync_states ) );
    // }
}
/********************************************************************************/


/********************************************************************************/
/* Main */
/********************************************************************************/
void app_main( void ) {
    nvs_flash_init();

    params_s *params = calloc( 1, sizeof( params_s ) );
    params->adc_unit = ADC_UNIT_1;
    params->adc_channel = ADC_CHANNEL_6; // pin 34
    params->adc_attenuation = ADC_ATTEN_DB_0;
    params->delay_micros = delay;
    params->average_alpha = AVRG_ALPHA;
    params->sender_period = delay;
    params->max_timing_error = timing_error;
    params->on_receive = receive_callback;
    params->frame_buffer_size = 256;
    params->frame_buffer = calloc( params->frame_buffer_size, sizeof( uint8_t ) );

    setup_adc( params );

    memset( counter_sync_states, 0, sizeof( counter_sync_states ) );
    counter_start = esp_timer_get_time();
    counter_log_start = counter_start;

    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &rx_task,
            .arg = params,
            .name = "rx"
    };
    esp_timer_handle_t rx_timer;
    ESP_LOGW( TAG, "Delay: %d, Max Timing Error: %d", delay, timing_error );
    ESP_ERROR_CHECK( esp_timer_create( &periodic_timer_args, &rx_timer ) );
    ESP_ERROR_CHECK( esp_timer_start_periodic( rx_timer, params->delay_micros ) );
}


static void receive_callback( uint8_t *payload, uint16_t payload_length ) {
    // counter_bytes += payload_length;
    // ESP_LOG_BUFFER_HEX( TAG, payload, payload_length );
    memset(payload, 0, payload_length);
    if ( counter_bytes > 2000 ) {
        int64_t time = esp_timer_get_time();
        float bytes_per_second = counter_bytes / ( ( time - counter_start ) / 1000000.0 );
        float syncs_per_second = counter_syncs / ( ( time - counter_start ) / 1000000.0 );
        ESP_LOGI( TAG, "Goodput: %f B/s", bytes_per_second );
        ESP_LOGI( TAG, "Out-of-sync: count=%d, rate=%f sync/s, state_counts=(%u, %u, %u)",
                counter_syncs, syncs_per_second,
                counter_sync_states[ 0 ], counter_sync_states[ 1 ], counter_sync_states[ 2 ] );

        counter_start = time;
        counter_bytes = 0;
        counter_syncs = 0;
        memset( counter_sync_states, 0, sizeof( counter_sync_states ) );
    }
}
/********************************************************************************/


/********************************************************************************/
/* Private */
/********************************************************************************/
static void setup_adc( params_s *params ) {
    if ( params->adc_unit == ADC_UNIT_1 ) {
        ESP_LOGI( TAG, "Configuring ADC1" );
        adc1_config_width( ADC_WIDTH_BIT_12 );
        adc1_config_channel_atten( params->adc_channel, params->adc_attenuation );
    } else {
        ESP_LOGI( TAG, "Configuring ADC2" );
        adc2_config_channel_atten( ( adc2_channel_t )params->adc_channel, params->adc_attenuation );
    }
}

static void parser_fsm( params_s *params, bool edge ) {
    switch ( params->parser_state ) {
        case PARSER_PREAMBLE: {
            params->preamble_buffer = ( params->preamble_buffer << 1 ) | edge;
            // ESP_LOGI( TAG, "Searching preamble: %d.", params->preamble_buffer);
            if ( params->preamble_buffer == PROTOCOL_PREABLE ) {
                // ESP_LOGI(TAG, "Parsing preamble.");
                params->parser_state = PARSER_LENGTH;
                params->preamble_buffer = 0;
                params->frame_length = 0;
                params->frame_length_counter = 0;
                counter_preambles += 1;
            }
            break;
        }
        case PARSER_LENGTH: {
            params->frame_length |= ( edge << params->frame_length_counter );
            // ESP_LOGI( TAG, "Searching payload length: %d.", params->frame_length);
            params->frame_length_counter += 1;
            if ( params->frame_length_counter >= 8 ) {
                if ( params->frame_buffer_size >= params->frame_length ) {
                // ESP_LOGI( TAG, "Payload length: %d.", params->frame_length);
                    params->parser_state = PARSER_PAYLOAD;
                    params->frame_bit_counter = 0;
                } else {
                    ESP_LOGE( TAG, "Frame buffer is too small (%u < %u). Skipping frame.",
                                params->frame_buffer_size, params->frame_length );
                    params->parser_state = PARSER_PREAMBLE;
                }
            }
            break;
        }
        case PARSER_PAYLOAD: {
            // ESP_LOGI(TAG, "Parsing payload.");
            uint8_t byte_pos = params->frame_bit_counter / 8;
            uint8_t bit_pos = params->frame_bit_counter % 8;
            params->frame_bit_counter += 1;
            params->frame_buffer[ byte_pos ] |= ( edge << bit_pos );
            if ( params->frame_bit_counter >= 8 * params->frame_length ) {
                // Frame fully parsed
                // ESP_LOG_BUFFER_HEX( TAG, params->frame_buffer, params->frame_length );
                params->parser_state = PARSER_PREAMBLE;
                counter_bytes += params->frame_length;
                if ( params->on_receive != NULL ) params->on_receive( params->frame_buffer, params->frame_length );
            }
            break;
        }
        default:
            ESP_LOGE( TAG, "Invalid parser state (%d).", params->parser_state );
    }
}

static void reset_parser_fsm( params_s *params ) {
    params->parser_state = PARSER_PREAMBLE;
    params->preamble_buffer = 0;
}
/********************************************************************************/

