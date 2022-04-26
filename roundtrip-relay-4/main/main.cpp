extern "C" 
{
    #include <esp_log.h>
    #include <nvs_flash.h>
}

#include "TriggerActionNetwork.h"

static const char* TAG = "roundtrip-relay";

void receiver ( char *media, uint8_t priority, uint8_t *payload, uint8_t payloadLength, bool deprovision ) ;

void receiver ( char *media, uint8_t priority, uint8_t *payload, uint8_t payloadLength, bool deprovision ) {
}

extern "C" void app_main( void ) {

    ESP_ERROR_CHECK( nvs_flash_init() );
    networkInit( receiver );
    networkSetupScan();
    setupAdvertising();

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

