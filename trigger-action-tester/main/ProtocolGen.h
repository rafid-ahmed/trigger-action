/*
Copyright (c) 2019 Teemu Kärkkäinen

ESP32 library for trigger-action programming
*/

#pragma once

#include <stdint.h>


/***********************************************************************/
/* API */
/***********************************************************************/
uint8_t* ProtoGen_NewLaserAction( /* in */ uint32_t actionId, uint32_t instanceId, uint8_t eventType,
                                   /* out */ uint32_t *createdLength );
uint8_t* ProtoGen_NewRgbLedAction( /* in */ uint32_t actionId, uint32_t instanceId, uint8_t red, uint8_t green, uint8_t blue,
                                   /* out */ uint32_t *createdLength );
uint8_t* ProtoGen_NewButtonTrigger( /* in */ uint32_t triggerId, uint32_t instanceId, uint8_t eventType,
                                    /* out */ uint32_t *createdLength );
uint8_t* ProtoGen_NewJoystickTrigger( /* in */ uint32_t triggerId, uint32_t instanceId, uint8_t axis, uint8_t op, uint8_t value, uint8_t range,
                                    /* out */ uint32_t *createdLength );
uint8_t* ProtoGen_NewIRRecvTrigger( /* in */ uint32_t triggerId, uint32_t instanceId, uint8_t eventType,
                                    /* out */ uint32_t *createdLength );
uint8_t* ProtoGen_NewRule( /* in */ uint32_t ruleId, uint32_t triggerId, uint32_t actionId,
                         /* out */ uint32_t *createdLength );
uint8_t* ProtoGen_NewSignal( /* in */ uint32_t triggerId,
                             /* out */ uint32_t *createdLength );
uint8_t* ProtoGen_NewTriggerLocationDescriptor(
        /* in */ uint32_t type, uint32_t instanceId, float x, float y, float z,
        /* out */ uint32_t *createdLength );
uint8_t* ProtoGen_NewActionLocationDescriptor(
        /* in */ uint32_t type, uint32_t instanceId, float x, float y, float z,
        /* out */ uint32_t *createdLength );
void Initiate_Random();
/***********************************************************************/
