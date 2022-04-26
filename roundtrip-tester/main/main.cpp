extern "C" 
{
    #include <freertos/FreeRTOS.h>
    #include <freertos/task.h>
    #include <stdio.h>
    #include <stdlib.h>
    #include <time.h>
    #include <inttypes.h>
    #include <errno.h>
    #include <string.h>
    #include <esp_system.h>
    #include <esp_timer.h>
    #include <esp_log.h>
    #include <nvs_flash.h>
    #include <math.h>
}

#include "TriggerActionNetwork.h"

// #define RANDOM_INTERVAL
// #define LORA
// #define VLC
// #define VLC_SYNCHRONIZE
// #define VLC_HOP 1
#define NO_REBOUND
// #define QOS_TEST

#define HOPS                        3

#ifdef  LORA
#define BLE_INTERVAL                200
#define LORA_HOP                    1
#define LORA_SCAN_PERIOD            10
#define LORA_IDLE_SCHEDULE_PERIOD   50
#define OTHER_LATENCY               20
#define ADDED_DELAY                 200
#define DECREMENT                   220
#define PACKET_INTERVAL             ( ( 250 * (LORA_HOP) ) /*+ ( LORA_IDLE_SCHEDULE_PERIOD * LORA_HOP )*/ + ( LORA_SCAN_PERIOD * LORA_HOP ) + ( OTHER_LATENCY * LORA_HOP ) + BLE_INTERVAL - DECREMENT )
#define PER_PACKET_DURATION         ( ( 250 * (LORA_HOP) ) /*+ ( LORA_IDLE_SCHEDULE_PERIOD * LORA_HOP )*/ + ( LORA_SCAN_PERIOD * LORA_HOP ) + ( OTHER_LATENCY * LORA_HOP ) + BLE_INTERVAL + ADDED_DELAY )
#endif

#ifdef  VLC
// #define VLCBLE
// #define VLCLORA
#define ALL
#define VLC_BIT_PERIOD      300 
#define VLC_RETRANSMISSION  5
#define VLC_HOP             1
#define OTHER_LATENCY       20
#define ADDED_DELAY         0
#if     !defined ( VLCBLE ) && !defined ( VLCLORA ) && !defined ( ALL )
#define PACKET_INTERVAL     ( ( ( VLC_BIT_PERIOD * 8 * 60 * VLC_RETRANSMISSION * 2 ) / 1000 ) + OTHER_LATENCY )
#define PER_PACKET_DURATION ( ( ( VLC_BIT_PERIOD * 8 * 60 * VLC_RETRANSMISSION * 2 ) / 1000 ) + OTHER_LATENCY )
#endif
#ifdef  VLCBLE
#define BLE_INTERVAL        160
#define DECREMENT           460
#define PACKET_INTERVAL     ( ( ( VLC_BIT_PERIOD * 8 * 60 * VLC_RETRANSMISSION * VLC_HOP ) / 1000 ) + OTHER_LATENCY + BLE_INTERVAL - DECREMENT )
#define PER_PACKET_DURATION ( ( ( VLC_BIT_PERIOD * 8 * 60 * VLC_RETRANSMISSION * VLC_HOP ) / 1000 ) + OTHER_LATENCY + BLE_INTERVAL + ADDED_DELAY )
#endif
#ifdef  VLCLORA
#define LORA_SCAN_PERIOD            10
#define LORA_IDLE_SCHEDULE_PERIOD   50
#define LORA_HOP                    2
#define LORA_INTERVAL               ( ( 250 * (LORA_HOP) ) /*+ ( LORA_IDLE_SCHEDULE_PERIOD * LORA_HOP )*/ + ( LORA_SCAN_PERIOD * LORA_HOP ) + ( OTHER_LATENCY * LORA_HOP ) )
#define DECREMENT                   600
#define PACKET_INTERVAL             ( ( ( VLC_BIT_PERIOD * 8 * 60 * VLC_RETRANSMISSION * VLC_HOP ) / 1000 ) + OTHER_LATENCY + LORA_INTERVAL - DECREMENT )
#define PER_PACKET_DURATION         ( ( ( VLC_BIT_PERIOD * 8 * 60 * VLC_RETRANSMISSION * VLC_HOP ) / 1000 ) + OTHER_LATENCY + LORA_INTERVAL + ADDED_DELAY )
#endif
#ifdef ALL
#define LORA_SCAN_PERIOD            10
#define LORA_IDLE_SCHEDULE_PERIOD   50
#define LORA_HOP                    1
#define BLE_INTERVAL                100
#define LORA_INTERVAL               ( ( 250 * (LORA_HOP) ) /*+ ( LORA_IDLE_SCHEDULE_PERIOD * LORA_HOP )*/ + ( LORA_SCAN_PERIOD * LORA_HOP ) + ( OTHER_LATENCY * LORA_HOP ) )
#define DECREMENT                   0
#define PACKET_INTERVAL             ( ( ( VLC_BIT_PERIOD * 8 * 60 * VLC_RETRANSMISSION * VLC_HOP ) / 1000 ) + OTHER_LATENCY + LORA_INTERVAL + BLE_INTERVAL - DECREMENT )
#define PER_PACKET_DURATION         ( ( ( VLC_BIT_PERIOD * 8 * 60 * VLC_RETRANSMISSION * VLC_HOP ) / 1000 ) + OTHER_LATENCY + LORA_INTERVAL + BLE_INTERVAL + ADDED_DELAY )
#endif
#endif

#ifdef  QOS_TEST
#define PRIORITIES                  4
#define QOS_IN_FLIGHT
#ifdef  QOS_IN_FLIGHT
#define BASE_PRIORITY               0
#define ONE_TIME_INTERVAL           750 * 5
#define SCHEDULE_INTERVAL           750 * 2
#endif
#endif

#define BUFFER_SIZE         25
#define N_TESTS             10
#define TESTER_SIGNAL       255
#if !defined ( VLC ) && !defined ( LORA )
#define PACKET_INTERVAL     800
#define PER_PACKET_DURATION 800
#endif
#define WAIT_PERIOD         ((PER_PACKET_DURATION - PACKET_INTERVAL) * N_TESTS) 
#define TEST_TYPE           "ONEWAY"

#define TxRxInfo 1
#define InFlight 10

static const char* TAG = "roundtrip-tester";

static uint8_t  source_buffer[BUFFER_SIZE];
static uint8_t* packets[N_TESTS];
static size_t   packet_lengths[N_TESTS], dispatch_times[N_TESTS], durations[N_TESTS], inFlightInfo[N_TESTS], prctIF[InFlight];
static bool     received[N_TESTS];
static uint8_t  order[N_TESTS];
static uint8_t  priorityarr[N_TESTS];

static int source_size, inFlight, maxInflight = 0;
size_t i, j, sendCount, receivedCount, matchCount, /*sendTime,*/ total, ordercount;
uint8_t firstByte[1];
// size_t prevPayloadLength = 0;

void receiver ( char *media, uint8_t priority, uint8_t *payload, uint8_t payloadLength, bool deprovision ) ;
bool isValid  ( uint8_t* payload, uint8_t payloadLength );

void receiver ( char *media, uint8_t priority_r, uint8_t *payload, uint8_t payloadLength, bool deprovision ) {
    uint8_t idx = payload[payloadLength-1];
    if ( /*prevPayloadLength != payloadLength*/ !received[idx-1] ) {
        if ( packet_lengths[idx-1] == payloadLength && memcmp(packets[idx-1], payload, payloadLength) == 0 ) {
            // ESP_LOGI(TAG, "received packet");
            durations[idx-1] = (esp_timer_get_time() - /*sendTime*/ dispatch_times[idx-1]) / 1000;
            received[idx-1] = true;
            #ifdef QOS_TEST
            order[ordercount] = idx;
            priorityarr[ordercount] = priority_r;
            ordercount++;
            #endif
            #if (TxRxInfo)
            ESP_LOGW(TAG, "Priority %d packet no. %d received.", idx, priority_r);
            #endif
            matchCount++;
            total += durations[idx-1];
            #if (TxRxInfo)
            ESP_LOGI(TAG, "%d-hop duration: %d ms.", HOPS, durations[idx-1]);
            #endif
        } else if ( payload[payloadLength-1] == TESTER_SIGNAL ) {
            return;
        }
        inFlight--;
        receivedCount++;
        // prevPayloadLength = payloadLength;
    }
}

void signalRebounder() {
    ESP_LOGI(TAG, "Signaling rebounder.");
    source_size = 2;
    memset(source_buffer, 0x00, sizeof(source_buffer));
    source_buffer[0] = 0;
    source_buffer[source_size-1] = TESTER_SIGNAL;
    networkWrite( source_buffer, source_size, 1, NON_PERSISTENT);
}

void sendPacket(uint8_t start, uint8_t end, bool base = true) {
    for (uint8_t i = start; i < end; ++i) {

        #if (TxRxInfo)
        ESP_LOGW(TAG, "Generating Packet...");
        #endif

        // source_size = rand() % BUFFER_SIZE;
        source_size = BUFFER_SIZE;
        // source_size = source_size < 2 ? 2 : source_size;
        memset(source_buffer, 0x00, sizeof(source_buffer));
        for (uint8_t j = 0; j < source_size - 1; ++j)
			source_buffer[j] = rand() % 256;
        source_buffer[source_size-1] = i+1;

        #if (TxRxInfo)
        ESP_LOGI(TAG, "Payload length: %d.", source_size);
        #ifndef RANDOM_INTERVAL
        ESP_LOGI(TAG, "In Flight Packets: %d. Sending new packet...", inFlight);
        #else
        ESP_LOGI(TAG, "In Flight Packets: %d. Sending packet no. %d with interval %d ms...", inFlight, i+1, random_interval);
        #endif
        #endif

        sendCount++;
        inFlight++;
        inFlightInfo[i] = inFlight;
        // sendTime = esp_timer_get_time();
        uint8_t priority = 0;
        #ifdef QOS
        #ifndef QOS_IN_FLIGHT
        priority = i % PRIORITIES; 
        #else
        if (base) {
            priority = BASE_PRIORITY;
        } else {
            do {
                priority = rand() % PRIORITIES;
            } while (priority == BASE_PRIORITY);
            // ESP_LOGW(TAG, "Sending priority %d packet.", priority);
        }
        #endif
        #else
        priority = PRIORITY_HIGH;
        #endif
        dispatch_times[i] = esp_timer_get_time();
        networkWrite( source_buffer, source_size, priority, NON_PERSISTENT);
        maxInflight = maxInflight < inFlight ? inFlight : maxInflight;

        firstByte[0] = ~source_buffer[0];
        memcpy(source_buffer, firstByte, 1);
        packets[i] = (uint8_t*) calloc(source_size, sizeof(uint8_t));
        // packets[i] = source_buffer;
        memcpy(packets[i], source_buffer, source_size);
        packet_lengths[i] = source_size;

        #ifndef RANDOM_INTERVAL
        if ( base ) {
            vTaskDelay( PACKET_INTERVAL / portTICK_PERIOD_MS );
        }
        else {
            #ifdef QOS_IN_FLIGHT
            vTaskDelay( SCHEDULE_INTERVAL / portTICK_PERIOD_MS );
            #endif
        }
        #else
        random_interval = (rand() % 5) * 100 + 600;
        vTaskDelay( random_interval / portTICK_PERIOD_MS );
        #endif
    }
}

extern "C" void app_main( void ) {

    ESP_ERROR_CHECK( nvs_flash_init() );
    networkInit( receiver );
    // #ifndef QOS_IN_FLIGHT
    networkSetupScan();
    setupAdvertising();
    // #endif
	srand(time(NULL));

    #ifdef RANDOM_INTERVAL
    int random_interval = 0;
    #endif

    #ifdef VLC_SYNCHRONIZE
    ESP_LOGI(TAG, "Synchronizing VLC");
    size_t syncpkt = N_TESTS < 50 ? 50 : N_TESTS;
    for (size_t i = syncpkt; i < syncpkt + VLC_HOP; ++i) {
        source_size = BUFFER_SIZE;
        memset(source_buffer, 0x00, sizeof(source_buffer));
        for (size_t j = 0; j < source_size - 1; ++j)
			source_buffer[j] = rand() % 256;
        source_buffer[source_size-1] = i+1;
        networkWrite( source_buffer, source_size, 1, NON_PERSISTENT);

        vTaskDelay( PER_PACKET_DURATION / portTICK_PERIOD_MS );
    }
    #endif

    ESP_LOGI(TAG, "%s Test Started", TEST_TYPE);

    #ifdef QOS_IN_FLIGHT
    sendPacket(0, N_TESTS / 2 + 5, true);
    // networkSetupScan();
    // setupAdvertising();
    vTaskDelay( ONE_TIME_INTERVAL / portTICK_PERIOD_MS );
    sendPacket(N_TESTS / 2 + 5, N_TESTS, false);
    #else
    sendPacket(0,N_TESTS);
    #endif

    ESP_LOGI(TAG, "%s Test Finished", TEST_TYPE);
    
    #ifndef NO_REBOUND
    vTaskDelay( WAIT_PERIOD / portTICK_PERIOD_MS );
    signalRebounder();
    #endif

    vTaskDelay( PER_PACKET_DURATION / portTICK_PERIOD_MS );

    #ifndef NO_REBOUND
    if ( sendCount == N_TESTS ) {
        if (matchCount > 0) {
            size_t average = total / matchCount;
            size_t tempvar = 0; size_t highest = 0; size_t lowest = 1000000;
            // #endif
            for( i = 0; i < N_TESTS; ++i ) {
                // #ifndef QOS_TEST
                if (durations[i] != 0) {
                    if( highest < durations[i] ) highest = durations[i];
                    if( lowest  > durations[i] ) lowest  = durations[i];
                    // tempvar += pow(durations[i] - average, 2);
                }
                // #endif
            }
            // size_t stdDeviation = sqrt(tempvar/matchCount);
            ESP_LOGW(TAG, "Average duration for %d hop %d packets: %d ms.", HOPS, matchCount, average);
            ESP_LOGW(TAG, "Highest duration: %d ms, Lowest duration: %d ms.", highest, lowest);
        }
        #ifndef QOS_TEST
        ESP_LOGW(TAG, "Max in-flight packets during the run: %d.", maxInflight - sendCount + receivedCount);
        #endif
        ESP_LOGW(TAG, "Corrupted packets received: %d.", receivedCount - matchCount);
        ESP_LOGW(TAG, "Packet lost: %d.", sendCount - receivedCount);


        #ifndef QOS_TEST
        for (size_t i = 0; i < N_TESTS - 1; i++)
        {
            if (!received[i]) {
                for (size_t j = i + 1; j < N_TESTS; j++) {
                    inFlightInfo[j]--;
                }
            }

            // ESP_LOGI(TAG, "%d in-flight packets after sending packet %d.", inFlightInfo[i], i+1);
            if ( inFlightInfo[i] <= InFlight && inFlightInfo[i] != 0 )
                prctIF[inFlightInfo[i] - 1]++;
        }

        // ESP_LOGI(TAG, "%d in-flight packets after sending packet %d.", inFlightInfo[N_TESTS-1], N_TESTS);
        if ( inFlightInfo[N_TESTS-1] <= InFlight && inFlightInfo[N_TESTS-1] != 0 )
                prctIF[inFlightInfo[N_TESTS-1] - 1]++;

        for (size_t i = 1; i <= InFlight; i++)
        {
            ESP_LOGI(TAG, "%d in-flight during %d packets.", i, prctIF[i-1]);
        }
        #endif

        // uint8_t lost_count;
        // for (size_t i = 0; i < 5; i++)
        // {
        //     lost_count = 0;
        //     for (size_t j = 0; j < 10; j++)
        //     {
        //         lost_count += found_packet[i * 10 + j] ? 0 : 1;
        //     }

        //     ESP_LOGW(TAG, "Packet lost between %d and %d: %d.", i * 10 + 1, i * 10 + 10, lost_count);
        // }
        
    }
    #endif

    #if defined ( VLC ) || defined ( LORA )
    ESP_LOGI(TAG, "Packet interval: %d ms.", PACKET_INTERVAL);
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

