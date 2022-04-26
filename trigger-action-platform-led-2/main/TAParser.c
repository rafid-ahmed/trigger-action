/*
Copyright (c) 2019 Teemu Kärkkäinen

ESP32 library for trigger-action programming.
*/

#include <freertos/FreeRTOS.h>
#include <lwip/sockets.h>
#include <string.h>

#include "esp_log.h"

#include "TAParser.h"
#include "sdnv.h"

static const char* TAG = "TAParser";

// #define DEBUG
/***********************************************************************/
/* Protocol definitions */
/***********************************************************************/
#define TA_BLE_MANUFACTURER     (0x8888)

#define TA_MSG_TYPE_TRIGGER_DEF     (0b001)
#define TA_MSG_TYPE_ACTION_DEF      (0b010)
#define TA_MSG_TYPE_RULE_DEF        (0b011)
#define TA_MSG_TYPE_TRIGGER_SIG     (0b100)
#define TA_MSG_TYPE_DESCRIPTOR      (0b101)

#define TRIGGER_SIGNAL_ID_TYPE              (1)
#define TRIGGER_SIGNAL_BLOOMFILTER_TYPE     (2)

#define TA_PAYLOAD_OFFSET   (1)
/***********************************************************************/


/***********************************************************************/
/* Types */
/***********************************************************************/
struct ta_parser {
    ErrorCallback onError;
    TriggerDefinitionCallback onTriggerDefinition;
    ActionDefinitionCallback onActionDefinition;
    RuleDefinitionCallback onRuleDefinition;
    TriggerSignalCallback onTriggerSignal;
};
/***********************************************************************/


/***********************************************************************/
/* API implementation */
/***********************************************************************/
extern TAParser* ta_new_parser(ErrorCallback onError, TriggerDefinitionCallback onTriggerDefinition, ActionDefinitionCallback onActionDefinition,
                                RuleDefinitionCallback onRuleDefinition, TriggerSignalCallback onTriggerSignal) {
    TAParser *parser = malloc( sizeof( TAParser ) );
    parser->onError = onError;
    parser->onTriggerDefinition = onTriggerDefinition;
    parser->onActionDefinition = onActionDefinition;
    parser->onRuleDefinition = onRuleDefinition;
    parser->onTriggerSignal = onTriggerSignal;
    return parser;
}

extern void ta_free_parser( TAParser *parser ) {
    free( parser );
}

extern uint8_t ta_parse( TAParser *parser, uint8_t bufferLength, uint8_t *buffer, bool deprovision ) {
    /*  Trigger-Action Header (Bits)
        +--+--+--+--+--+--+--+--+--
        + type   | length       | ...
        +--+--+--+--+--+--+--+--+--
        type    = 0b000 reserved
                = 0b001 trigger definition
                = 0b010 action definition
                = 0b011 rule definition
                = 0b100 trigger signal
                = 0b101 action signal
                = 0b110 descriptor (These are for GUIs, no need to parse)
                = 0b111 reserved
        length = length of the following trigger-action message
    */

    if ( bufferLength < 1 )  {
        if ( parser->onError != NULL ) parser->onError( ERROR_NOT_ENOUGH_INPUT );
        return 0;
    }

    uint8_t ta_type = ( ( buffer[ 0 ] & 0b11100000 ) >> 5 );
    uint8_t ta_len =    ( buffer[ 0 ] & 0b00011111 );

    #ifdef DEBUG
    ESP_LOGI( TAG, "ta_type = %u, ta_len = %u", ta_type, ta_len );
    #endif
    if ( ta_type != TA_MSG_TYPE_TRIGGER_DEF && ta_type != TA_MSG_TYPE_ACTION_DEF && ta_type != TA_MSG_TYPE_RULE_DEF && ta_type != TA_MSG_TYPE_TRIGGER_SIG ) {
        if ( parser->onError != NULL ) 
            parser->onError( ERROR_UNKNOWN_TA_MESSAGE_TYPE );
        return 0;
    }

    /*  Trigger Definition (Bytes)
        +--+--+--+--+--+--+--+--+--+--+--+--+--    ....   --+
        | id        | type      | inst. id  | type-specific |
        +--+--+--+--+--+--+--+--+--+--+--+--+--    ....   --+
        id = trigger identifier (SDNV)
        type = trigger type (SDNV)
        inst. id = instance id (SDNV)
        type-specific = type specific parameters (ta_len - len(id) - len(type) - len(inst. id))
    */
    if ( ta_type == TA_MSG_TYPE_TRIGGER_DEF ) {
        #ifdef DEBUG
        ESP_LOGI( TAG, "TA_MSG_TYPE_TRIGGER_DEF" );
        #endif

        if ( bufferLength < TA_PAYLOAD_OFFSET + 3 ) { // Will take at least three bytes (might take more)
            if ( parser->onError != NULL ) 
                parser->onError( ERROR_NOT_ENOUGH_INPUT );
            return 0;
        }

        trigger_definition_s *trigger_def = allocate_trigger_definition( ta_len, ( buffer + TA_PAYLOAD_OFFSET ) );

        if ( parser->onTriggerDefinition != NULL ) {
            parser->onTriggerDefinition( trigger_def, deprovision ); // Callback must eventually free the trigger definition
        } else {
            free_trigger_definition( trigger_def );
        }

        return TA_PAYLOAD_OFFSET + ta_len; // Done parsing.
    }

    /*  Action Definition (Bytes)
        +--+--+--+--+--+--+--+--+--+--+--+--+--    ....   --+
        | id        | type      | inst. id  | type-specific |
        +--+--+--+--+--+--+--+--+--+--+--+--+--    ....   --+
        id = action identifier (SDNV)
        type = action type (SDNV)
        inst. id = instance id (SDNV)
        type-specific = type specific parameters (ta_len - len(id) - len(type) - len(inst. id))
    */
    if ( ta_type == TA_MSG_TYPE_ACTION_DEF ) {
        #ifdef DEBUG
        ESP_LOGI( TAG, "TA_MSG_TYPE_ACTION_DEF" );
        #endif
        if ( bufferLength < TA_PAYLOAD_OFFSET + 2 ) { // Need at least two bytes, might need more (SDNVs)
            if ( parser->onError != NULL ) 
                parser->onError( ERROR_NOT_ENOUGH_INPUT );
            return 0;
        }

        if ( parser->onActionDefinition != NULL ) {
            action_definition_s *action_def = allocate_action_definition( ta_len, ( buffer + TA_PAYLOAD_OFFSET ) );
            parser->onActionDefinition( action_def, deprovision ); // Callback must eventually free the trigger definition
        }

        return TA_PAYLOAD_OFFSET + ta_len; // Done parsing.
    }

    /*  Rule Definition (Bytes)
        +--+--+--+--+--+--+--+--+--+--+--+--+
        | rule id   |trigger id | action id |
        +--+--+--+--+--+--+--+--+--+--+--+--+
        rule id = rule identifier (4 Bytes)
        trigger id = trigger identifier (4 Bytes)
        action id = action identifier (4 Bytes)
    */
    if ( ta_type == TA_MSG_TYPE_RULE_DEF ) {
        #ifdef DEBUG
        ESP_LOGI( TAG, "TA_MSG_TYPE_RULE_DEF" );
        #endif
        if ( bufferLength < TA_PAYLOAD_OFFSET + 3 ) { // Need at least 3 bytes, might need more (SDNVs)
            if ( parser->onError != NULL ) 
                parser->onError( ERROR_NOT_ENOUGH_INPUT );
            return 0;
        }

        if ( parser->onRuleDefinition != NULL ) {
            rule_s *rule = allocate_rule( ta_len, ( buffer + TA_PAYLOAD_OFFSET ) );
            parser->onRuleDefinition( rule );
        }

        return TA_PAYLOAD_OFFSET + ta_len; // Done parsing.
    }


    /*  Trigger Signal (Bytes)
        +------+--    ...    --+
        | type | type-specific |
        +------+--    ...    --+
        type    = 0     trigger id
                = 1     bloomfilter (not implemented)
    */
    if ( ta_type == TA_MSG_TYPE_TRIGGER_SIG ) {
        #ifdef DEBUG
        ESP_LOGI( TAG, "TA_MSG_TYPE_TRIGGER_SIG" );
        #endif
        if ( bufferLength < TA_PAYLOAD_OFFSET + 1 ) {
            if ( parser->onError != NULL ) {
                ESP_LOGE( TAG, "Buffer doesn't contain signal type. (bufferLength = %u)", bufferLength );
                parser->onError( ERROR_NOT_ENOUGH_INPUT );
                return 0;
            }
        }

        uint8_t signal_type = buffer[ TA_PAYLOAD_OFFSET ];

        if ( signal_type == TRIGGER_SIGNAL_ID_TYPE ) {
            // Need at least one byte for the ID (possibly more)
            if ( bufferLength < TA_PAYLOAD_OFFSET + 1 + 1 ) {
                if ( parser->onError != NULL ) {
                    ESP_LOGE( TAG, "Buffer doesn't contain ID. (bufferLength = %u)", bufferLength );
                    parser->onError( ERROR_NOT_ENOUGH_INPUT );
                    return 0;
                }
            }

            if ( parser->onTriggerSignal != NULL ) {
                SdnvDecodeResult id = SDNV_Decode64( ( uint8_t* )( buffer + TA_PAYLOAD_OFFSET + 1 ), bufferLength - TA_PAYLOAD_OFFSET - 1 );
                if ( id.byteCount <= 0 ) {
                    ESP_LOGE( TAG, "Couldn't parse trigger id from signal message" );
                }

                parser->onTriggerSignal( ( uint32_t )id.value.u64 );
            }
        } else {
            if ( parser->onError != NULL ) parser->onError( ERROR_UNKNWON_TRIGGER_SIGNAL_TYPE );
            return TA_PAYLOAD_OFFSET + ta_len; // Correctly parsed, just can't handle it
        }

        return TA_PAYLOAD_OFFSET + ta_len; // Done parsing.
    }

    return TA_PAYLOAD_OFFSET + ta_len; // Done parsing.
}
/***********************************************************************/
