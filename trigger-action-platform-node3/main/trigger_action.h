/*
Copyright (c) 2019 Teemu Kärkkäinen

ESP32 library for trigger-action programming
*/

#pragma once

#include <stdint.h>

/***********************************************************************/
/* Values */
/***********************************************************************/
#define TRIGGER_DEF_TYPE    (0b001)
#define ACTION_DEF_TYPE     (0b010)
#define RULE_DEF_TYPE       (0b011)
#define TRIGGER_SIGNAL_TYPE (0b100)
#define ACTION_SIGNAL_TYPE  (0b101)

#define COMBINED_TRIGGER_TYPE       (0)
#define TEMPERATURE_TRIGGER_TYPE    (1)
#define PRESSURE_TRIGGER_TYPE       (2)
#define BUTTON_TRIGGER_TYPE         (3)
#define JOYSTICK_TRIGGER_TYPE		(4)
#define IRRECV_TRIGGER_TYPE			(6)

#define RGB_LED_ACTION_TYPE         (1)
#define BUZZER_ACTION_TYPE          (2)
#define NOTE_PLAYER_ACTION_TYPE     (3)
#define TUNE_PLAYER_ACTION_TYPE     (4)
#define LASER_ACTION_TYPE			(5)

#define TEMPERATURE_GT_OP       (0x01)
#define TEMPERATURE_LT_OP       (0x02)
#define TEMPERATURE_GTEQ_OP     (0x03)
#define TEMPERATURE_LTEQ_OP     (0x04)
#define TEMPERATURE_RANGE_OP    (0x05)

#define COMBINED_AND_OP     (0x01)
#define COMBINED_OR_OP      (0x02)
#define COMBINED_NOT_OP     (0x03)
#define COMBINED_XOR_OP     (0x04)

/***********************************************************************/


/***********************************************************************/
/* Types */
/***********************************************************************/

// rule definition
// params: rule_id, trigger_id, action_id
typedef struct {
    uint32_t rule_id;
    uint32_t trigger_id;
    uint32_t action_id;
} rule_s;

// action definition
// params: action_id, action_type, instance_id, params_length, params*
typedef struct {
    uint32_t action_id;
    uint32_t action_type;
    uint32_t instance_id;
    uint8_t params_length;
    uint8_t *params;
} action_definition_s;

// trigger definition
// params: trigger_id, trigger_type, instance_id, params_length, params*
typedef struct {
    uint32_t trigger_id;
    uint32_t trigger_type;
    uint32_t instance_id;
    uint8_t params_length;
    uint8_t *params;
} trigger_definition_s;
/***********************************************************************/


/***********************************************************************/
/* Functions */
/***********************************************************************/

action_definition_s* allocate_action_definition( size_t const buffer_length, uint8_t *buffer );
void free_action_definition( action_definition_s *action );

trigger_definition_s* allocate_trigger_definition( size_t const buffer_length, uint8_t *buffer );
void free_trigger_definition( trigger_definition_s *trigger );

rule_s* allocate_rule( uint8_t buffer_length, uint8_t *buffer );
void free_rule( rule_s *rule );

/**
 * Serializes a trigger into the given buffer.
 * 
 * \param buffer_len is the length of the buffer
 */
int32_t serialize_temperature_trigger(
        uint8_t buffer_len,
        uint8_t *buffer,
        uint32_t trigger_id, /* 3 bytes */
        char op,
        float value  );
/***********************************************************************/
