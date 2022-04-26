
//Libraries for LoRa
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include <SPI.h>
#include <LoRa.h>

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 866E6

String LoRaData;

void receiveMessages(void* pvParameter)
{
    while (1) {
        //try to parse packet
        int packetSize = LoRa.parsePacket();
        if (packetSize) {
            //received a packet
            printf("Received packet ");

            //read packet
            while (LoRa.available()) {
                LoRaData = LoRa.readString();
                printf("%s",LoRaData.c_str());
            }

            //print RSSI of packet
            int rssi = LoRa.packetRssi();
            printf(" with RSSI ");    
            printf("%d\n",rssi);
        }
        
        TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
        TIMERG0.wdt_feed=1;
        TIMERG0.wdt_wprotect=0;
    }
}

extern "C" void app_main()
{
    initArduino();
    printf("LoRa Receiver Test\n");
    //SPI LoRa pins
    SPI.begin(SCK, MISO, MOSI, SS);
    //setup LoRa transceiver module
    LoRa.setPins(SS, RST, DIO0);
    
    if (!LoRa.begin(BAND)) {
        Serial.println("Starting LoRa failed!\n");
        while (1);
    }

    printf("LoRa Initializing OK!\n");
    delay(2000);
    xTaskCreate(receiveMessages, "receive_messages", 1024 * 4, (void* )0, 3, nullptr);
}