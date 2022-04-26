/*
Copyright (c) 2019 Teemu Kärkkäinen

ESP32 library for trigger-action programming
*/

#pragma once

#include <stdint.h>
#include <stdlib.h>

// Guard against missing include by user.
// Causes linking to fail if the header is not included.
#define SDNV_Encode64   sdnv_encode64
#define SDNV_Decode64   sdnv_decode64
#define SDNV_ByteCount  sdnv_byte_count


/***********************************************************************/
/* Types */
/***********************************************************************/
typedef union {
    uint64_t u64;
} SdnvValue;

typedef struct {
    SdnvValue value;
    int32_t byteCount;
} SdnvDecodeResult;
/***********************************************************************/


/***********************************************************************/
/* Functions */
/***********************************************************************/
/** Writes the given values as an SDNV into the buffer. Returns the number of bytes written, or 0 on error. */
extern uint32_t SDNV_Encode64( uint64_t const value, uint8_t *buffer, size_t const buffer_len );
extern SdnvDecodeResult SDNV_Decode64( uint8_t const * buffer, size_t const buffer_len );
extern uint32_t SDNV_ByteCount( uint64_t const value );
/***********************************************************************/

