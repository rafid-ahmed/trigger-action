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

    #include "ProtocolGen.h"
}

#include "TriggerActionNetwork.h"

static const char* TAG = "trigger-action-tester";

#define LED
// #define LASER
// #define BUTTON_1
// #define BUTTON_2
// #define JOYSTICK
// #define TA_RULES
#define DUMMY_RULES

#define SAMPLE_COUNT    30
#define DUMMY_OFF       80
#define DUMMY_ON        81

void receiver ( char *media, uint8_t priority, uint8_t *payload, uint8_t payloadLength, bool deprovision ) ;

void receiver ( char *media, uint8_t priority_r, uint8_t *payload, uint8_t payloadLength, bool deprovision ) {
    
}

void queryPins (gpio_num_t node1, gpio_num_t node2, gpio_num_t node3, gpio_num_t node4, gpio_num_t node5, int64_t start_time, bool definition) {
    bool node1_done = false, node2_done = false, node3_done = false, node4_done = false, node5_done = false;
    uint32_t end_time, end_time1 = 0, end_time2 = 0, end_time3 = 0, end_time4 = 0, end_time5 = 0, node1_level, node2_level, node3_level, node4_level, node5_level;
   
    // TEST: Create adverts
    uint8_t *adBytes;
    uint32_t adLength;
    bool turned_on = true;
    bool switch_state = true;
    uint8_t switch_count = 0;
    uint32_t total_duration[5];
    total_duration[0] = 0;
    total_duration[1] = 0;
    total_duration[2] = 0;
    total_duration[3] = 0;
    total_duration[4] = 0;
    uint32_t max_duration[5];
    max_duration[0] = 0;
    max_duration[1] = 0;
    max_duration[2] = 0;
    max_duration[3] = 0;
    max_duration[4] = 0;
    uint32_t min_duration[5];
    min_duration[0] = 100000;
    min_duration[1] = 100000;
    min_duration[2] = 100000;
    min_duration[3] = 100000;
    min_duration[4] = 100000;
    uint8_t previos_level [5];
    previos_level[0] = 0;
    previos_level[1] = 0;
    previos_level[2] = 0;
    previos_level[3] = 0;
    previos_level[4] = 0;
    int64_t start = start_time;
    while (true)
    {
        if ( !definition && switch_state ) {
            
            if ( turned_on ) {
                ESP_LOGI( TAG, "Sending button trigger signal: %d, state: OFF", switch_count + 1 );
                adBytes = ProtoGen_NewSignal( DUMMY_OFF, &adLength );
                turned_on = false;
            } else{
                ESP_LOGI( TAG, "Sending button trigger signal: %d, state: ON", switch_count + 1 );
                adBytes = ProtoGen_NewSignal( DUMMY_ON, &adLength );
                turned_on = true;
            }

            vTaskDelay( 1000 / portTICK_PERIOD_MS );
            networkWrite( adBytes, adLength, PRIORITY_HIGH, NON_PERSISTENT);
            free( adBytes );

            node1_done = false, node2_done = false, node3_done = false, node4_done = false, node5_done = false;
            switch_state = false;

            start = esp_timer_get_time();
        }

        node1_level = gpio_get_level(node1);
        node2_level = gpio_get_level(node2);
        node3_level = gpio_get_level(node3);
        node4_level = gpio_get_level(node4);
        node5_level = gpio_get_level(node5);

        if ( ( (definition && node1_level > 0) || (!definition && node1_level != previos_level[0]) ) && !node1_done ) {
            end_time = (esp_timer_get_time() - start) / 1000;
            if ( definition )
                ESP_LOGW( TAG, "Node 1 total duration: %d ms", end_time );
            else {
                end_time1 = end_time;
                if ( end_time == 0 ) ESP_LOGW( TAG, "Node 1 previous level: %d, current level: %d", previos_level[0], node1_level );

                previos_level[0] = node1_level;
            }
            node1_done = true;
        }

        if ( ( (definition && node2_level > 0) || (!definition && node2_level != previos_level[1]) ) && !node2_done ) {
            end_time = (esp_timer_get_time() - start) / 1000;
            if ( definition )
                ESP_LOGW( TAG, "Node 2 total duration: %d ms", end_time );
            else {
                end_time2 = end_time;
                if ( end_time == 0 ) ESP_LOGW( TAG, "Node 2 previous level: %d, current level: %d", previos_level[1], node2_level );

                previos_level[1] = node2_level;
            }
            node2_done = true;
        }

        if ( ( (definition && node3_level > 0) || (!definition && node3_level != previos_level[2]) ) && !node3_done ) {
            end_time = (esp_timer_get_time() - start) / 1000;
            if ( definition )
                ESP_LOGW( TAG, "Node 3 total duration: %d ms", end_time );
            else {
                end_time3 = end_time;
                if ( end_time == 0 ) ESP_LOGW( TAG, "Node 3 previous level: %d, current level: %d", previos_level[2], node3_level );

                previos_level[2] = node3_level;
            }
            node3_done = true;
        }

        if ( ( (definition && node4_level > 0) || (!definition && node4_level != previos_level[3]) ) && !node4_done ) {
            end_time = (esp_timer_get_time() - start) / 1000;
            if ( definition )
                ESP_LOGW( TAG, "Node 4 total duration: %d ms", end_time );
            else {
                end_time4 = end_time;
                if ( end_time == 0 ) ESP_LOGW( TAG, "Node 4 previous level: %d, current level: %d", previos_level[3], node4_level );

                previos_level[3] = node4_level;
            }
            node4_done = true;
        }

        if ( ( (definition && node5_level > 0) || (!definition && node5_level != previos_level[4]) ) && !node5_done ) {
            end_time = (esp_timer_get_time() - start) / 1000;
            if ( definition )
                ESP_LOGW( TAG, "Node 5 total duration: %d ms", end_time );
            else {
                end_time5 = end_time;
                if ( end_time == 0 ) ESP_LOGW( TAG, "Node 5 previous level: %d, current level: %d", previos_level[4], node5_level );

                previos_level[4] = node5_level;
            }
            node5_done = true;
        }

        if ( ( (definition && node1_level == 0 && node1_done) || (!definition && node1_done) ) 
                                                && ( (definition && node2_level == 0 && node2_done) || (!definition && node2_done) ) 
                                                && ( (definition && node3_level == 0 && node3_done) || (!definition && node3_done) )
                                                && ( (definition && node4_level == 0 && node4_done) || (!definition && node4_done) ) 
                                                && ( (definition && node5_level == 0 && node5_done) || (!definition && node5_done) ) ) {
            if ( !definition ) {

                total_duration[0] += end_time1;
                total_duration[1] += end_time2;
                total_duration[2] += end_time3;
                total_duration[3] += end_time4;
                total_duration[4] += end_time5;

                if (max_duration[0] < end_time1) max_duration[0] = end_time1;
                if (min_duration[0] > end_time1) min_duration[0] = end_time1;
                if (max_duration[1] < end_time2) max_duration[1] = end_time2;
                if (min_duration[1] > end_time2) min_duration[1] = end_time2;
                if (max_duration[2] < end_time3) max_duration[2] = end_time3;
                if (min_duration[2] > end_time3) min_duration[2] = end_time3;
                if (max_duration[3] < end_time4) max_duration[3] = end_time4;
                if (min_duration[3] > end_time4) min_duration[3] = end_time4;
                if (max_duration[4] < end_time5) max_duration[4] = end_time5;
                if (min_duration[4] > end_time5) min_duration[4] = end_time5;

                // ESP_LOGI( TAG, "Node 1 current duration: %d ms, min duration: %d ms, max duration: %d ms", end_time1, min_duration[0], max_duration[0] );

                ESP_LOGW(TAG, "Previous Levels. State: %d %d %d %d %d.", previos_level[0], previos_level[1], previos_level[2], previos_level[3], previos_level[4]);

                switch_state = true;
                switch_count++;
            }
            
            if ( definition || switch_count == SAMPLE_COUNT ) {
                break;
            }

        } else if ( !definition && (esp_timer_get_time() - start) / 1000000 > 2 ) {
            switch_state = true;
            ESP_LOGE(TAG, "Reached Timeout. State: %d %d %d %d %d. Switching...", (uint8_t)node1_done, (uint8_t)node2_done, (uint8_t)node3_done, (uint8_t)node4_done, (uint8_t)node5_done);
        }

        vTaskDelay( 50 / portTICK_PERIOD_MS );
    }

    if ( !definition ) {
        for (size_t i = 0; i < 5; i++)
        {
            ESP_LOGW( TAG, "Node %d average duration: %d ms", i + 1, total_duration[i] / SAMPLE_COUNT );
            ESP_LOGW( TAG, "Max duration: %d ms, Min duration: %d ms", max_duration[i], min_duration[i] );
        }   
    }
}

extern "C" void app_main( void ) {

    ESP_ERROR_CHECK( nvs_flash_init() );
    networkInit( receiver );
    networkSetupScan();
    setupAdvertising();

    // TEST: Create adverts
    uint8_t *adBytes;
    uint32_t adLength;

    ESP_LOGI( TAG, "Test Started");
    ESP_LOGW( TAG, "Sending definiton packets");
    
    // 10 action definitions (2 action definitions per node)

    /*============================ LED ===============================*/

    // LED Action #1: Off
    adBytes = ProtoGen_NewRgbLedAction( 20, 1, 0x00, 0x00, 0xff, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // LED Action #1: Blueish
    adBytes = ProtoGen_NewRgbLedAction( 21, 1, 0x00, 0x00, 0x00, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    // LED Action #2: Off
    adBytes = ProtoGen_NewRgbLedAction( 22, 2, 0x00, 0x00, 0xff, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // LED Action #2: Blueish
    adBytes = ProtoGen_NewRgbLedAction( 23, 2, 0x00, 0x00, 0x00, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    // LED Action #3: Off
    adBytes = ProtoGen_NewRgbLedAction( 24, 3, 0x00, 0x00, 0xff, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // LED Action #3: Blueish
    adBytes = ProtoGen_NewRgbLedAction( 25, 3, 0x00, 0x00, 0x00, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    // LED Action #4: Off
    adBytes = ProtoGen_NewRgbLedAction( 26, 4, 0x00, 0x00, 0xff, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // LED Action #4: Blueish
    adBytes = ProtoGen_NewRgbLedAction( 27, 4, 0x00, 0x00, 0x00, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    // LED Action #5: Off
    adBytes = ProtoGen_NewRgbLedAction( 28, 5, 0x00, 0x00, 0xff, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // LED Action #5: Blueish
    adBytes = ProtoGen_NewRgbLedAction( 29, 5, 0x00, 0x00, 0x00, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    /*=========================== LASER ==============================*/

    #ifdef LASER
    {
        // Laser Action: on
        adBytes = ProtoGen_NewLaserAction( 30, 1, 1, &adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Laser Action: off
        adBytes = ProtoGen_NewLaserAction( 31, 1, 0, &adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );

        // Laser Action: on
        adBytes = ProtoGen_NewLaserAction( 32, 2, 1, &adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Laser Action: off
        adBytes = ProtoGen_NewLaserAction( 33, 2, 0, &adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );

        // Laser Action: on
        adBytes = ProtoGen_NewLaserAction( 34, 3, 1, &adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Laser Action: off
        adBytes = ProtoGen_NewLaserAction( 35, 3, 0, &adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );

        // Laser Action: on
        adBytes = ProtoGen_NewLaserAction( 36, 4, 1, &adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Laser Action: off
        adBytes = ProtoGen_NewLaserAction( 37, 4, 0, &adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );

        // Laser Action: on
        adBytes = ProtoGen_NewLaserAction( 38, 5, 1, &adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Laser Action: off
        adBytes = ProtoGen_NewLaserAction( 39, 5, 0, &adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
    }
    #endif

    // 10 trigger definitions (2 trigger definitions per node)

    /*========================== BUTTON 1 =============================*/

    #ifdef BUTTON_1
    {
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

        // Button Trigger: #4 Down
        adBytes = ProtoGen_NewButtonTrigger( 46, 4, 0, &adLength );
        // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Button Trigger: #4 Up
        adBytes = ProtoGen_NewButtonTrigger( 47, 4, 1, &adLength );
        // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes ); 

        // Button Trigger: #4 Down
        adBytes = ProtoGen_NewButtonTrigger( 48, 5, 0, &adLength );
        // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Button Trigger: #4 Up
        adBytes = ProtoGen_NewButtonTrigger( 49, 5, 1, &adLength );
        // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes ); 
    }
    #endif

    /*========================== BUTTON 2 =============================*/

    #ifdef BUTTON_2
    {
        // Button Trigger: #1 Down
        adBytes = ProtoGen_NewButtonTrigger( 50, 6, 0, &adLength );
        // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Button Trigger: #1 Up
        adBytes = ProtoGen_NewButtonTrigger( 51, 6, 1, &adLength );
        // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );

        // Button Trigger: #2 Down
        adBytes = ProtoGen_NewButtonTrigger( 52, 7, 0, &adLength );
        // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Button Trigger: #2 Up
        adBytes = ProtoGen_NewButtonTrigger( 53, 7, 1, &adLength );
        // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );

        // Button Trigger: #3 Down
        adBytes = ProtoGen_NewButtonTrigger( 54, 8, 0, &adLength );
        // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Button Trigger: #3 Up
        adBytes = ProtoGen_NewButtonTrigger( 55, 8, 1, &adLength );
        // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );

        // Button Trigger: #4 Down
        adBytes = ProtoGen_NewButtonTrigger( 56, 9, 0, &adLength );
        // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Button Trigger: #4 Up
        adBytes = ProtoGen_NewButtonTrigger( 57, 9, 1, &adLength );
        // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes ); 

        // Button Trigger: #4 Down
        adBytes = ProtoGen_NewButtonTrigger( 58, 10, 0, &adLength );
        // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Button Trigger: #4 Up
        adBytes = ProtoGen_NewButtonTrigger( 59, 10, 1, &adLength );
        // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes ); 
    }
    #endif

    /*========================== JOY STICK ============================*/

    #ifdef JOY_STICK
    {
        // Joystick Trigger button down
        adBytes = ProtoGen_NewJoystickTrigger(90, 1, 0, 0, 1, 0, &adLength);
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Joystick Trigger button up
        adBytes = ProtoGen_NewJoystickTrigger(91, 1, 0, 0, 0, 0, &adLength);
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Joystick Trigger x > 20
        adBytes = ProtoGen_NewJoystickTrigger(92, 1, 1, 2, 20, 0, &adLength);
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Joystick Trigger x < 5
        adBytes = ProtoGen_NewJoystickTrigger(93, 1, 1, 1, 5, 0, &adLength);
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Joystick Trigger x home
        adBytes = ProtoGen_NewJoystickTrigger(94, 1, 1, 5, 10, 11, &adLength);
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Joystick Trigger y > 20
        adBytes = ProtoGen_NewJoystickTrigger(95, 1, 2, 2, 20, 0, &adLength);
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Joystick Trigger y < 5
        adBytes = ProtoGen_NewJoystickTrigger(96, 1, 2, 1, 5, 0, &adLength);
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
        // Joystick Trigger y home
        adBytes = ProtoGen_NewJoystickTrigger(97, 1, 2, 5, 10, 11, &adLength);
        networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
        free( adBytes );
    }
    #endif

    /*============================ RULES ==============================*/

    #ifdef TA_RULES
    {
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
    }
    #endif
    
    #ifdef DUMMY_RULES
    //---------------------- dummy -> led 1 -------------------------//
    
    // Rule: Up -> Off
    adBytes = ProtoGen_NewRule( 90, DUMMY_OFF, 20, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // Rule: Down -> Blue
    adBytes = ProtoGen_NewRule( 91, DUMMY_ON, 21, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    //---------------------- dummy -> led 2 -------------------------//
    // Rule: Up -> Off
    adBytes = ProtoGen_NewRule( 92, DUMMY_OFF, 22, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // Rule: Down -> Blue
    adBytes = ProtoGen_NewRule( 93, DUMMY_ON, 23, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    //---------------------- dummy -> led 3 -------------------------//
    // Rule: Up -> Off
    adBytes = ProtoGen_NewRule( 94, DUMMY_OFF, 24, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // Rule: Down -> Blue
    adBytes = ProtoGen_NewRule( 95, DUMMY_ON, 25, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    //---------------------- dummy -> led 4 -------------------------//
    // Rule: Up -> Off
    adBytes = ProtoGen_NewRule( 96, DUMMY_OFF, 26, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // Rule: Down -> Blue
    adBytes = ProtoGen_NewRule( 97, DUMMY_ON, 27, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );

    //---------------------- dummy -> led 5 -------------------------//
    // Rule: Up -> Off
    adBytes = ProtoGen_NewRule( 98, DUMMY_OFF, 28, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    // Rule: Down -> Blue
    adBytes = ProtoGen_NewRule( 99, DUMMY_ON, 29, &adLength );
    // parseTriggerActionPayload( "TA", PRIORITY_LOW, adBytes, adLength );
    networkWrite( adBytes, adLength, PRIORITY_LOW, NON_PERSISTENT);
    free( adBytes );
    
    #endif

    int64_t start_time = esp_timer_get_time();
    
    gpio_num_t node1 = GPIO_NUM_38, node2 = GPIO_NUM_39, node3 = GPIO_NUM_37, node4 = GPIO_NUM_36, node5 = GPIO_NUM_17;
    gpio_set_direction(node1, GPIO_MODE_INPUT);
    gpio_set_direction(node2, GPIO_MODE_INPUT);
    gpio_set_direction(node3, GPIO_MODE_INPUT);
    gpio_set_direction(node4, GPIO_MODE_INPUT);
    gpio_set_direction(node5, GPIO_MODE_INPUT);
    
    queryPins(node1, node2, node3, node4, node5, start_time, true);

    // vTaskDelay( 1000 * 120 / portTICK_PERIOD_MS );

    Initiate_Random();
    queryPins(node1, node2, node3, node4, node5, start_time, false);

    ESP_LOGI( TAG, "Test Finished");

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

