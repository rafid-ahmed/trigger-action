#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "manchester_encoding.h"

/********************************************************************************/
/* Types and definitions */
/********************************************************************************/
#define LED_GPIO    (12)

const static char *TAG = "vlc_tx";

typedef struct {
    uint8_t pin;
    int64_t delay_micros;
    bool state;

    uint8_t *buffer;
    uint32_t buffer_size;
    uint32_t buffer_byte_pos;
    uint8_t buffer_bit_pos;
} params_s;

static void init_gpio( uint8_t pin );
static inline void delay( int64_t delay_micros );
static int manchester_encode( uint8_t *input, uint32_t input_size, uint8_t *output, uint32_t output_size );
/********************************************************************************/


/********************************************************************************/
/* Tx Task */
/********************************************************************************/
static void tx_task( void *void_params ) {
    params_s *params = void_params;

    // Get the next bit form the buffer
    uint8_t level = ( ( params->buffer[ params->buffer_byte_pos ] >> params->buffer_bit_pos ) ^ 1 ) & 1;
    params->buffer_bit_pos = ( params->buffer_bit_pos + 1 ) % 8;
    if ( params->buffer_bit_pos == 0 ) params->buffer_byte_pos = ( params->buffer_byte_pos + 1 ) % params->buffer_size;

    // Set the level
    gpio_set_level( params->pin, level );
}
/********************************************************************************/


/********************************************************************************/
/* Main */
/********************************************************************************/
void app_main( void ) {
    nvs_flash_init();
    init_gpio( LED_GPIO );

    uint8_t input[] = { 0x99, 0x1D,
                        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
                        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
                        0x19, 0x1A, 0x1B, 0x1C, 0x1D };

    // uint8_t input[] = { 0x99, 0x03,
    //                     0x82, 0x01, 0x15};

    params_s *params = calloc( 1, sizeof( params_s ) );
    params->pin = LED_GPIO;
    params->delay_micros = 100;
    params->buffer_size = 62;
    params->buffer = calloc( params->buffer_size, sizeof( uint8_t ) );
    manchester_encode( input, 31, params->buffer, params->buffer_size );
    
    // gpio_set_level( params->pin, 1 );
    // vTaskDelay( 10000 / portTICK_PERIOD_MS );

    ESP_LOG_BUFFER_HEX( TAG, input, 31);
    ESP_LOG_BUFFER_HEX( TAG, params->buffer, params->buffer_size);
    // ESP_LOG_BUFFER_HEX( TAG, params->buffer, params->buffer_size );


    //xTaskCreate( tx_task, "txTask", 2048, params, 5, NULL );
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &tx_task,
            .arg = params,
            .name = "tx"
    };
    esp_timer_handle_t tx_timer;
    ESP_ERROR_CHECK( esp_timer_create( &periodic_timer_args, &tx_timer ) );
    ESP_ERROR_CHECK( esp_timer_start_periodic( tx_timer, params->delay_micros ) );
}
/********************************************************************************/


/********************************************************************************/
/* Private */
/********************************************************************************/
static void init_gpio( uint8_t pin ) {
    gpio_pad_select_gpio( pin );
    gpio_set_direction( pin, GPIO_MODE_OUTPUT);
}

static inline void delay( int64_t delay_micros ) {
    int64_t start = esp_timer_get_time();
    int64_t end = start + delay_micros;
    while( esp_timer_get_time() < end );
}

static int manchester_encode(uint8_t *input, uint32_t input_size, uint8_t *output, uint32_t output_size ) {
    // Preconditions
    if ( output_size < 2 * input_size ) {
        ESP_LOGE( TAG, "Invalid output size. Input=%d, Output=%d.", input_size, output_size );
        return -1;
    }

    for ( int i = 0; i < input_size; i++ ) {
        output[ 2 * i ]     = MANCHESTER_4_BITS[ input[ i ] & 0b1111 ];
        output[ 2 * i + 1 ] = MANCHESTER_4_BITS[ ( input[ i ] >> 4 ) & 0b1111 ];
    }

    return 0;
}
/********************************************************************************/
