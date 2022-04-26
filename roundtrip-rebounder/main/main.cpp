extern "C" 
{
    #include <freertos/FreeRTOS.h>
    #include <freertos/task.h>
    #include <esp_log.h>
    #include <nvs_flash.h>
    #include <string.h>
}

#include "TriggerActionNetwork.h"

#define BUFFER_SIZE     25
#define N_TESTS         10
#define TESTER_SIGNAL   255

#define TxRxInfo        1
#define NoRebound

static const char* TAG = "roundtrip-rebounder";

static uint8_t  rebound_buffer[BUFFER_SIZE];
static size_t   differences[N_TESTS];
static size_t   packet[N_TESTS];
static size_t   priorities[N_TESTS];
uint8_t bufferSize = 0, priority = 0;
size_t prevpackettime = 0;
uint32_t total = 0;
// uint32_t mindiff = 1000000;
uint32_t diff = 1000000;

void receiver ( char *media, uint8_t priority, uint8_t *payload, uint8_t payloadLength, bool deprovision ) ;

void receiver ( char *media, uint8_t priority_r, uint8_t *payload, uint8_t payloadLength, bool deprovision ) {
    if ( memcmp(rebound_buffer + 1, payload + 1, payloadLength - 1) != 0) {
        uint8_t idx = payload[payloadLength-1];
        if ( idx <= N_TESTS ) {
            if ( prevpackettime != 0 ) diff = (esp_timer_get_time() - prevpackettime) / 1000;
            // if ( mindiff > diff ) mindiff = diff;
            differences[idx-1] = diff; 
            packet[total] = idx;
            priority = priority_r;
            priorities[total] = priority; 
            #if (TxRxInfo)
            if ( prevpackettime != 0 ) {
                ESP_LOGW(TAG, "Packet no. %d received. Difference: %d ms", idx, diff );
            } else {
                ESP_LOGW(TAG, "Packet no. %d received.", idx);
            }
            #endif
            prevpackettime = esp_timer_get_time();   
        }
        memcpy(rebound_buffer, payload, payloadLength);
        bufferSize = payloadLength;
        if(idx == TESTER_SIGNAL) {
            ESP_LOGW(TAG, "Total received %d packets.", total);
            #if (!TxRxInfo)
            for (size_t i = 1; i < N_TESTS; i++)
            {
                if(differences[i] >= 70)
                    ESP_LOGI(TAG, "Priority: %d, Packet no.: %d, Arrival difference: %d ms", priorities[i], packet[i], differences[i] );
                else if(differences[i] < 65)
                    ESP_LOGE(TAG, "Packet no. %d arrival difference: %d ms", packet[i], differences[i] );
                else
                    ESP_LOGW(TAG, "Packet no. %d arrival difference: %d ms", packet[i], differences[i] );
            }
            #endif
            // ESP_LOGW(TAG, "Minimum difference in reception time %d ms.", mindiff);
        }
        if( idx <= N_TESTS )
            total++;
    }
}

extern "C" void app_main( void ) {

    ESP_ERROR_CHECK( nvs_flash_init() );
    networkInit( receiver );
    networkSetupScan();
    setupAdvertising();

    #ifndef NoRebound
    while(true) {

        if  (bufferSize != 0) {
            uint8_t byte[1];
            byte[0] = ~rebound_buffer[0];
            memcpy(rebound_buffer, byte, 1);
            #if (TxRxInfo)
            ESP_LOGI(TAG, "Rebounding..");
            #endif
            // ESP_LOG_BUFFER_HEX(TAG, rebound_buffer, bufferSize);
            // changePath( false );

            networkWrite( rebound_buffer, bufferSize, priority, NON_PERSISTENT);
            // changePath( true );
            bufferSize = 0;
        }
        vTaskDelay( 5 / portTICK_PERIOD_MS );
    }
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

