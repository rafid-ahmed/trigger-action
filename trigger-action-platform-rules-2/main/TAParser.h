#pragma once

#include <stdint.h>

#include "trigger_action.h"

// Guard against missing include by user.
// Causes linking to fail if the header is not included.
#define TAParser_NewParser   ta_new_parser
#define TAParser_FreeParser  ta_free_parser
#define TAParser_Parse       ta_parse

/***********************************************************************/
/* Types */
/***********************************************************************/
typedef struct ta_parser TAParser;

#define ParseError  uint16_t
#define ERROR_NOT_ENOUGH_INPUT                  (1)
#define ERROR_NOT_TA_MESSAGE                    (2)
#define ERROR_UNKNOWN_TA_MESSAGE_TYPE           (3)
#define ERROR_INVALID_LENGTH                    (4)
#define ERROR_UNKNWON_TRIGGER_SIGNAL_TYPE       (5)

typedef void ( *ErrorCallback )( ParseError );
typedef void ( *TriggerDefinitionCallback )( trigger_definition_s*, bool );
typedef void ( *ActionDefinitionCallback )( action_definition_s*, bool );
typedef void ( *RuleDefinitionCallback )( rule_s* );
typedef void ( *TriggerSignalCallback )( uint32_t );
/***********************************************************************/


/***********************************************************************/
/* Functions */
/***********************************************************************/
extern TAParser* TAParser_NewParser(ErrorCallback onError, TriggerDefinitionCallback onTriggerDefinition, ActionDefinitionCallback onActionDefinition, 
                                        RuleDefinitionCallback onRuleDefinition, TriggerSignalCallback onTriggerSignal);

extern void TAParser_FreeParser( TAParser *parser );

extern uint8_t TAParser_Parse(TAParser *parser, uint8_t bufferLength, uint8_t *buffer, bool deprovision );
/***********************************************************************/
