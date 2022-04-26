/*
Copyright (c) 2019 Teemu Kärkkäinen

ESP32 library for trigger-action programming
*/


#include <stdint.h>
#include <string.h>
#include <esp_log.h>
#include <lwip/sockets.h>

#include "ProtocolGen.h"
#include "sdnv.h"

static const char* TAG = "ProtocolGen";

/***********************************************************************/
/* Definitions */
/***********************************************************************/
#define TRIGGER_TYPE_BUTTON     (3)
#define TRIGGER_TYPE_JOYSTICK	(4)
#define TRIGGER_TYPE_IRRECV		(6)

#define ACTION_TYPE_RGB_LED     (1)
#define ACTION_TYPE_LASER		(5)

#define DESCRIPTOR_TYPE_TRIGGER (1)
#define DESCRIPTOR_TYPE_ACTION  (2)

#define BLE_HEADER_LENGTH       (0)
#define TA_HEADER_LENGTH        (1)

static void encode_ble_header( uint8_t *buffer, uint32_t bufferLength );
/***********************************************************************/


/***********************************************************************/
/* API */
/***********************************************************************/
uint8_t* ProtoGen_NewRgbLedAction(  /* in */ uint32_t actionId, uint32_t instanceId, uint8_t red, uint8_t green, uint8_t blue,
                                    /* out */ uint32_t *createdLength ) {
    uint32_t length = BLE_HEADER_LENGTH + 1 + SDNV_ByteCount( actionId ) + SDNV_ByteCount( ACTION_TYPE_RGB_LED ) + SDNV_ByteCount( instanceId )
		+ 3;
    uint8_t *buffer = calloc( 1, length );
    // encode_ble_header( buffer, length );

    uint16_t pos = BLE_HEADER_LENGTH; // Payload starts at position 4 after the BLE header
    buffer[ pos++ ] = ( 0b01000000 | ( length - ( BLE_HEADER_LENGTH + TA_HEADER_LENGTH ) ) ); // type + length
    pos += SDNV_Encode64( actionId, buffer + pos, length - pos );
    pos += SDNV_Encode64( ACTION_TYPE_RGB_LED, buffer + pos, length - pos );
    pos += SDNV_Encode64( instanceId, buffer + pos, length - pos );
    buffer[ pos++ ] = red;
    buffer[ pos++ ] = green;
    buffer[ pos++ ] = blue;

    *createdLength = length;
    ESP_LOGI( TAG, "Encoding RGB led action protocol. Length: %u", length );
    return buffer;
}

uint8_t* ProtoGen_NewLaserAction(  /* in */ uint32_t actionId, uint32_t instanceId, uint8_t eventType,
                                    /* out */ uint32_t *createdLength ) {
    uint32_t length = BLE_HEADER_LENGTH + 1 + SDNV_ByteCount( actionId ) + SDNV_ByteCount( ACTION_TYPE_LASER ) + SDNV_ByteCount( instanceId )
	   	+ 1;
    uint8_t *buffer = calloc( 1, length );
    // encode_ble_header( buffer, length );

    uint16_t pos = BLE_HEADER_LENGTH; // Payload starts at position 4 after the BLE header
    buffer[ pos++ ] = ( 0b01000000 | ( length - ( BLE_HEADER_LENGTH + TA_HEADER_LENGTH ) ) ); // type + length
    pos += SDNV_Encode64( actionId, buffer + pos, length - pos );
    pos += SDNV_Encode64( ACTION_TYPE_LASER, buffer + pos, length - pos );
    pos += SDNV_Encode64( instanceId, buffer + pos, length - pos );
    buffer[ pos++ ] = eventType;

    *createdLength = length;
    return buffer;
}

uint8_t* ProtoGen_NewButtonTrigger( /* in */ uint32_t triggerId, uint32_t instanceId, uint8_t eventType,
                                    /* out */ uint32_t *createdLength ) {
    uint32_t length = BLE_HEADER_LENGTH + 1 + SDNV_ByteCount( triggerId ) + SDNV_ByteCount( TRIGGER_TYPE_BUTTON ) + SDNV_ByteCount( instanceId )
        + 1;
    uint8_t *buffer = calloc( 1, length );
    // encode_ble_header( buffer, length );

    int pos = BLE_HEADER_LENGTH;
    buffer[ pos++ ] = ( 0b00100000 | ( length - ( BLE_HEADER_LENGTH + TA_HEADER_LENGTH ) ) ); // type + length
    pos += SDNV_Encode64( triggerId, buffer + pos, length - pos );
    pos += SDNV_Encode64( TRIGGER_TYPE_BUTTON, buffer + pos, length - pos );
    pos += SDNV_Encode64( instanceId, buffer + pos, length - pos );
    buffer[ pos++ ] = eventType;

    *createdLength = length;
    ESP_LOGI( TAG, "Encoding button trigger protocol. Length: %u", length );
    return buffer;
}

uint8_t* ProtoGen_NewJoystickTrigger( /* in */ uint32_t triggerId, uint32_t instanceId, uint8_t axis, uint8_t op, uint8_t value, uint8_t range,
                                    /* out */ uint32_t *createdLength ) {
    uint32_t length = BLE_HEADER_LENGTH + 1 + SDNV_ByteCount( triggerId ) + SDNV_ByteCount( TRIGGER_TYPE_JOYSTICK ) + SDNV_ByteCount( instanceId )
        + 1 + 1 + 1 + 1 + 1;
    uint8_t *buffer = calloc( 1, length );
    // encode_ble_header( buffer, length );

    int pos = BLE_HEADER_LENGTH;
    buffer[ pos++ ] = ( 0b00100000 | ( length - ( BLE_HEADER_LENGTH + TA_HEADER_LENGTH ) ) ); // type + length
    pos += SDNV_Encode64( triggerId, buffer + pos, length - pos );
    pos += SDNV_Encode64( TRIGGER_TYPE_JOYSTICK, buffer + pos, length - pos );
    pos += SDNV_Encode64( instanceId, buffer + pos, length - pos );
	buffer[ pos++ ] = axis;
	buffer[ pos++ ] = op;
	buffer[ pos++ ] = value;
	buffer[ pos++ ] = range;

    *createdLength = length;
    return buffer;
}

uint8_t* ProtoGen_NewIRRecvTrigger( /* in */ uint32_t triggerId, uint32_t instanceId, uint8_t eventType,
                                    /* out */ uint32_t *createdLength ) {
    uint32_t length = BLE_HEADER_LENGTH + 1 + SDNV_ByteCount( triggerId ) + SDNV_ByteCount( TRIGGER_TYPE_IRRECV ) + SDNV_ByteCount( instanceId )
        + 1;
    uint8_t *buffer = calloc( 1, length );
    // encode_ble_header( buffer, length );

    int pos = BLE_HEADER_LENGTH;
    buffer[ pos++ ] = ( 0b00100000 | ( length - ( BLE_HEADER_LENGTH + TA_HEADER_LENGTH ) ) ); // type + length
    pos += SDNV_Encode64( triggerId, buffer + pos, length - pos );
    pos += SDNV_Encode64( TRIGGER_TYPE_IRRECV, buffer + pos, length - pos );
    pos += SDNV_Encode64( instanceId, buffer + pos, length - pos );
    buffer[ pos++ ] = eventType;

    *createdLength = length;
    return buffer;
}

uint8_t* ProtoGen_NewRule(  /* in */ uint32_t ruleId, uint32_t triggerId, uint32_t actionId,
                            /* out */ uint32_t *createdLength ) {
    uint32_t length = BLE_HEADER_LENGTH + 1 + SDNV_ByteCount( ruleId ) + SDNV_ByteCount( triggerId ) + SDNV_ByteCount( actionId );
    uint8_t *buffer = calloc( 1, length );
    // encode_ble_header( buffer, length );

    int pos = BLE_HEADER_LENGTH;
    buffer[ pos++ ] = ( 0b01100000 | ( length - ( BLE_HEADER_LENGTH + TA_HEADER_LENGTH ) ) ); // type + length
    pos += SDNV_Encode64( ruleId, buffer + pos, length - pos );
    pos += SDNV_Encode64( triggerId, buffer + pos, length - pos );
    pos += SDNV_Encode64( actionId, buffer + pos, length - pos );

    *createdLength = length;
    ESP_LOGI( TAG, "Encoding rule protocol. Length: %u", length );
    return buffer;
}

uint8_t* ProtoGen_NewSignal( /* in */ uint32_t triggerId,
                             /* out */ uint32_t *createdLength ) {
    uint32_t length = BLE_HEADER_LENGTH + TA_HEADER_LENGTH + 1 + SDNV_ByteCount( triggerId );
    uint8_t *buffer = calloc( 1, length );
    // encode_ble_header( buffer, length );

    int pos = BLE_HEADER_LENGTH;
    buffer[ pos++ ] = ( 0b10000000 | ( length - ( BLE_HEADER_LENGTH + TA_HEADER_LENGTH ) ) );
    buffer[ pos++ ] = 1; // trigger signal type (1 = id, 2 = bloomfilter)
    SDNV_Encode64( triggerId, buffer + pos, length - pos );

    *createdLength = length;
    ESP_LOGI( TAG, "Encoding signal protocol. Length: %u", length );
    return buffer;
}

uint8_t* ProtoGen_NewTriggerLocationDescriptor(
        /* in */ uint32_t type, uint32_t instanceId, float x, float y, float z,
        /* out */ uint32_t *createdLength ) {
    uint32_t length = BLE_HEADER_LENGTH + TA_HEADER_LENGTH + 1 + SDNV_ByteCount( type ) + SDNV_ByteCount( instanceId ) + 3 * 4;
    uint8_t *buffer = calloc( 1, length );
    // encode_ble_header( buffer, length );

    int pos = BLE_HEADER_LENGTH;
    buffer[ pos++ ] = ( 0b10100000 | ( length - ( BLE_HEADER_LENGTH + TA_HEADER_LENGTH ) ) ); // type + length
    pos += SDNV_Encode64( 1, buffer + pos, length - pos );
    pos += SDNV_Encode64( type, buffer + pos, length - pos );
    pos += SDNV_Encode64( instanceId, buffer + pos, length - pos );

    *( uint32_t* )( buffer + pos ) = htonl( *( uint32_t* )( &x ) );
    pos += 4;
    *( uint32_t* )( buffer + pos ) = htonl( *( uint32_t* )( &y ) );
    pos += 4;
    *( uint32_t* )( buffer + pos ) = htonl( *( uint32_t* )( &z ) );
    pos += 4;

    *createdLength = length;
    return buffer;
}

uint8_t* ProtoGen_NewActionLocationDescriptor(
        /* in */ uint32_t type, uint32_t instanceId, float x, float y, float z,
        /* out */ uint32_t *createdLength ) {
    uint32_t length = BLE_HEADER_LENGTH + TA_HEADER_LENGTH + 1 + SDNV_ByteCount( type ) + SDNV_ByteCount( instanceId ) + 3 * 4;
    uint8_t *buffer = calloc( 1, length );
    // encode_ble_header( buffer, length );

    int pos = BLE_HEADER_LENGTH;
    buffer[ pos++ ] = ( 0b10100000 | ( length - ( BLE_HEADER_LENGTH + TA_HEADER_LENGTH ) ) ); // type + length
    pos += SDNV_Encode64( 2, buffer + pos, length - pos );
    pos += SDNV_Encode64( type, buffer + pos, length - pos );
    pos += SDNV_Encode64( instanceId, buffer + pos, length - pos );

    *( uint32_t* )( buffer + pos ) = htonl( *( uint32_t* )( &x ) );
    pos += 4;
    *( uint32_t* )( buffer + pos ) = htonl( *( uint32_t* )( &y ) );
    pos += 4;
    *( uint32_t* )( buffer + pos ) = htonl( *( uint32_t* )( &z ) );
    pos += 4;

    *createdLength = length;
    return buffer;
}
/***********************************************************************/
