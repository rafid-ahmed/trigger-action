/*
Copyright (c) 2019 Teemu Kärkkäinen

ESP32 library for trigger-action programming.
*/

#include <freertos/FreeRTOS.h>
#include <lwip/sockets.h>
#include <esp_log.h>
#include <string.h>

#include "trigger_action.h"
#include "sdnv.h"

static const char* TAG = "TriggerAction";

// #define DEBUG
/***********************************************************************/
/* Private */
/***********************************************************************/


void write_header(
        uint8_t type,
        uint8_t length,
        uint8_t *buffer ) {
    buffer[ 0 ] = ( ( type & 0b111 ) << 5 );
    buffer[ 0 ] |= ( length & 0x11111 );
}
/***********************************************************************/



/***********************************************************************/
/* API */
/***********************************************************************/

/*  Trigger Definition (Bytes)
    +--+--+--+--+--+--+--+--+--+--+--+--+--    ....   --+
    | id        | type      | inst. id  | type-specific |
    +--+--+--+--+--+--+--+--+--+--+--+--+--    ....   --+
    id = trigger identifier (SDNV)
    type = trigger type (SDNV)
    inst. id = instance id (SDNV)
    type-specific = type specific parameters (ta_len - len(id) - len(type) - len(inst. id))
*/
trigger_definition_s* allocate_trigger_definition( size_t const buffer_length, uint8_t *buffer ) {
    #ifdef DEBUG
    ESP_LOGI( TAG, "Allocation trigger definition. Payload length: %u", buffer_length);
    #endif
    SdnvDecodeResult id = SDNV_Decode64( buffer, buffer_length );
    if ( id.byteCount <= 0 ) return NULL;

    SdnvDecodeResult type = SDNV_Decode64( ( buffer + id.byteCount ), buffer_length );
    if ( type.byteCount <= 0 ) return NULL;

    SdnvDecodeResult instance_id = SDNV_Decode64( ( buffer + id.byteCount + type.byteCount ), buffer_length );
    if ( instance_id.byteCount <= 0 ) return NULL;

    trigger_definition_s *trigger = malloc( sizeof( trigger_definition_s ) );

    trigger->trigger_id = id.value.u64;
    trigger->trigger_type = type.value.u64;
    trigger->instance_id = instance_id.value.u64;

    uint32_t params_start = id.byteCount + type.byteCount + instance_id.byteCount;
    trigger->params_length = buffer_length - params_start;
    trigger->params = malloc( trigger->params_length );
    memcpy( trigger->params, ( buffer + params_start ), trigger->params_length );

    return trigger;
}

void free_trigger_definition( trigger_definition_s *trigger ) {
    free( trigger->params );
    free( trigger );
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
action_definition_s* allocate_action_definition( size_t const buffer_length, uint8_t *buffer ) {
    #ifdef DEBUG
    ESP_LOGI( TAG, "Allocation action definition. Payload length: %u", buffer_length);
    #endif
    //action_definition_s *action = malloc( sizeof( action_definition_s ) );
    //action->action_id = ntohl( *( uint32_t* )( buffer ) );
    //action->action_type = ntohl( *( uint32_t* )( buffer + 4 ) );

    SdnvDecodeResult id = SDNV_Decode64( buffer, buffer_length );
    if ( id.byteCount <= 0 ) return NULL;

    SdnvDecodeResult type = SDNV_Decode64( ( buffer + id.byteCount ), buffer_length );
    if ( type.byteCount <= 0 ) return NULL;

    SdnvDecodeResult instance_id = SDNV_Decode64( ( buffer + id.byteCount + type.byteCount ), buffer_length );
    if ( instance_id.byteCount <= 0 ) return NULL;

    action_definition_s *action = malloc( sizeof( action_definition_s ) );

    action->action_id = id.value.u64;
    action->action_type = type.value.u64;
    action->instance_id = instance_id.value.u64;

    uint32_t params_start = id.byteCount + type.byteCount + instance_id.byteCount;
    action->params_length = buffer_length - params_start;
    action->params = malloc( action->params_length );
    memcpy( action->params, ( buffer + params_start ), action->params_length );

    return action;
}

void free_action_definition( action_definition_s *action ) {
    free( action->params );
    free( action );
}


/*  Rule Definition (Bytes)
    +--+--+--+--+--+--+--+--+--+--+--+--+
    | rule id   |trigger id | action id |
    +--+--+--+--+--+--+--+--+--+--+--+--+
    rule id = rule identifier (SDNV)
    trigger id = trigger identifier (SDNV)
    action id = action identifier (SDNV)
*/
rule_s* allocate_rule( uint8_t buffer_length, uint8_t *buffer ) {
    #ifdef DEBUG
    ESP_LOGI( TAG, "Allocation rule definition. Payload length: %u", buffer_length);
    #endif
    SdnvDecodeResult rule_id = SDNV_Decode64( buffer, buffer_length );
    if ( rule_id.byteCount <= 0 ) return NULL;

    int pos = rule_id.byteCount;
    SdnvDecodeResult trigger_id = SDNV_Decode64( buffer + pos, buffer_length - pos );
    if ( trigger_id.byteCount <= 0 ) return NULL;
    pos += trigger_id.byteCount;

    SdnvDecodeResult action_id = SDNV_Decode64( buffer + pos, buffer_length - pos );
    if ( action_id.byteCount <= 0 ) return NULL;

    rule_s *rule = malloc( sizeof( rule_s ) );
    rule->rule_id = rule_id.value.u64;
    rule->trigger_id = trigger_id.value.u64;
    rule->action_id = action_id.value.u64;

    return rule;
}

void free_rule( rule_s *rule ) {
    free( rule );
}

int32_t serialize_temperature_trigger(
        uint8_t buffer_len,
        uint8_t *buffer,
        uint32_t trigger_id,
        char op,
        float value ) {
    // Some offsets to make things clearer
    uint16_t trigger_id_offset = 1;
    uint16_t trigger_type_offset = trigger_id_offset + 4;
    uint16_t temperature_op_offset = trigger_type_offset + 4;
    uint16_t temperature_operand_offset = temperature_op_offset + 1;

    // Message header. Assumes all operators take one operand.
    uint8_t msg_length = ( 3 + 4 + 1 + 4 ); /* trigger id + type + operator + value */
    write_header( TRIGGER_DEF_TYPE, msg_length, buffer );

    // Trigger header - trigger id
    *( uint32_t* )( buffer + trigger_id_offset ) = htonl( trigger_id );

    // Trigger header - trigger type
    *( buffer + trigger_type_offset ) = htonl( TEMPERATURE_TRIGGER_TYPE );

    // type-specific data
    switch ( op ) {
        case '>':
            buffer[ temperature_op_offset ] = TEMPERATURE_GT_OP;
            *( uint32_t* )( buffer + temperature_operand_offset ) = htonl( *( uint32_t* )( &value ) );
            break;

        case '<':
            buffer[ temperature_op_offset ] = TEMPERATURE_LT_OP;
            *( uint32_t* )( buffer + temperature_operand_offset ) = htonl( *( uint32_t* )( &value ) );
            break;

        case '_':

            break;
    
        default:
            return -1;
    }

    return 0;
}

int32_t serialize_combined_trigger(
        uint8_t buffer_len,
        uint8_t *buffer,
        uint32_t trigger_id,
        uint8_t op, /* One of COMBINED_X_OP */
        uint32_t *operands /* number of operands must match what is required by the operator */ ) {
    
    // Some offsets to make things clearer
    uint16_t trigger_id_offset = 1;
    uint16_t trigger_type_offset = trigger_id_offset + 4;
    uint16_t combined_op_offset = trigger_type_offset + 4;
    uint16_t combined_operand_offset = combined_op_offset + 1;

    // Message header.
    uint8_t length = ( 3 + 4 + 1 + ( op == COMBINED_NOT_OP ) ? 4 : 8 );
    write_header( TRIGGER_DEF_TYPE, length, buffer );

    // Trigger header - trigger id
    *( uint32_t* )( buffer + trigger_id_offset ) = htonl( trigger_id );

    // Trigger header - trigger type
    *( buffer + trigger_type_offset ) = htonl( COMBINED_TRIGGER_TYPE );

    // type-specific data
    buffer[ combined_op_offset ] = op;

    // operand(s)
    *( uint32_t* )( buffer + combined_operand_offset ) = htonl( *( uint32_t* )( operands[ 0 ] ) );
    if ( op != COMBINED_NOT_OP ) {
        *( uint32_t* )( buffer + combined_operand_offset + 4 ) = htonl( *( uint32_t* )( operands[ 1 ] ) );
    }

    return 0;
}
/***********************************************************************/
