
//Libraries for LoRa
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

//packet counter
int counter = 0;

void sendMessages(void* pvParameter)
{
    while (1) {
        printf("Sending packet: ");
        printf("%d\n",counter);

        //Send LoRa packet to receiver
        LoRa.beginPacket();
        LoRa.print("Rafid ");
        LoRa.print(counter);
        LoRa.endPacket();

        counter++;
        delay(10000);
    }
}

extern "C" void app_main()
{
    initArduino();
    printf("LoRa Sender Test\n");

    //SPI LoRa pins
    SPI.begin(SCK, MISO, MOSI, SS);
    //setup LoRa transceiver module
    LoRa.setPins(SS, RST, DIO0);
    
    if (!LoRa.begin(BAND)) {
        printf("Starting LoRa failed!\n");
        while (1);
    }

    printf("LoRa Initializing OK!\n");
    delay(2000);
    xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, nullptr);
}