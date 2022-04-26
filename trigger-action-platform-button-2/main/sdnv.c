/*
Copyright (c) 2019 Teemu Kärkkäinen

ESP32 library for trigger-action programming.
*/

#include <math.h>
#include <stdio.h>

#include "sdnv.h"

/***********************************************************************/
/* Types */
/***********************************************************************/
struct decode_result {
    SdnvValue value;
    int32_t byteCount;
};
/***********************************************************************/


/***********************************************************************/
/* API implementation */
/***********************************************************************/
extern uint32_t SDNV_Encode64( uint64_t const value, uint8_t *buffer, size_t const buffer_len ) {
    // TODO: There is probably a better algorithm for this

    // Count number of bits
    uint64_t tmp = value;
    uint8_t count = 64;
    while ( ( tmp >> 63 ) == 0 ) {
        count--;
        tmp <<= 1;
    }
    uint8_t numBytes = ( uint8_t )( ceil( count / 7.0 ) );

    // Check that there is enough space in the buffer
    if ( buffer_len < numBytes ) {
        printf( "Need %u bytes, have %zu bytes of space.\n", numBytes, buffer_len );
        return 0;
    }

    // Build the SDNV
    uint64_t val = value;
    for ( int i = 0; i < numBytes; i++ ) {
        buffer[ numBytes - i - 1 ] = ( uint8_t )( val & 0x7f );
        if ( i != 0 ) buffer[ numBytes - i - 1 ] |= 0x80;
        val >>= 7;
    }

    return numBytes;
}

extern SdnvDecodeResult SDNV_Decode64( uint8_t const * buffer, size_t const buffer_len ) {
    SdnvDecodeResult result = { .value.u64 = 0, .byteCount = 0 };
    do {
        if ( result.byteCount == buffer_len ) { // Don't overrun the buffer
            result.value.u64 = 0;
            result.byteCount = 0; // signal that there isn't enough data
        } else if ( result.byteCount < 9 ) { // 9 first bytes contain 7 bits of payload
            result.value.u64 <<= 7;
            result.value.u64 |= ( *buffer ) & 0b01111111;
            result.byteCount += 1;
        } else if ( result.byteCount == 9 ) { // 10th byte can only contain one bit of payload
            result.value.u64 <<= 1;
            result.value.u64 |= ( *buffer ) & 0b00000001;
            result.byteCount += 1;
        } else { // Overflow
            result.value.u64 = 0;
            result.byteCount = -1; // signal error
            break;
        }
    } while ( *( buffer++ ) & 0b10000000 );

    return result;
}

extern uint32_t SDNV_ByteCount( uint64_t const value ) {
    if ( value == 0 ) return 0;
    return ( uint32_t )( floor( log( value ) / ( 7 * log( 2 ) ) ) ) + 1;
}
/***********************************************************************/


/***********************************************************************/
/* Test Main */
/***********************************************************************/
int main( int argc, char **argv ) {
    // Test 1
    uint64_t original1 = 0b01111111;
    
    uint8_t encoded1[] = { 0 };
    uint32_t encodedLen1 = SDNV_Encode64( original1, encoded1, 1 );

    if ( encodedLen1 != 1 ) {
        printf( "TEST1: Invalid encoded length: %u\n", encodedLen1 );
        return -1;
    }

    SdnvDecodeResult decoded1 = SDNV_Decode64( encoded1, 1 );

    if ( decoded1.byteCount != 1 ) {
        printf( "TEST1: Invalid decoded length: %u\n", decoded1.byteCount );
        return -1;
    }

    if ( decoded1.value.u64 != original1 ) {
        printf( "TEST1: Invalid decoded value: %llu\n", decoded1.value.u64 );
        return -1;
    }

    // Test 2
    uint64_t original2 = 0b11111111;
    
    uint8_t encoded2[] = { 0, 0 };
    uint32_t encodedLen2 = SDNV_Encode64( original2, encoded2, 2 );

    if ( encodedLen2 != 2 ) {
        printf( "TEST2: Invalid encoded length: %u\n", encodedLen2 );
        return -1;
    }

    SdnvDecodeResult decoded2 = SDNV_Decode64( encoded2, 2 );

    if ( decoded2.byteCount != 2 ) {
        printf( "TEST2: Invalid decoded length: %u\n", decoded2.byteCount );
        return -1;
    }

    if ( decoded2.value.u64 != original2 ) {
        printf( "TEST2: Invalid decoded value: %llu\n", decoded2.value.u64 );
        return -1;
    }

    // Test 2
    uint64_t original3 = 987654321;
    
    uint8_t encoded3[] = { 0, 0, 0, 0, 0 };
    uint32_t encodedLen3 = SDNV_Encode64( original3, encoded3, 5 );

    if ( encodedLen3 != 5 ) {
        printf( "TEST3: Invalid encoded length: %u\n", encodedLen3 );
        return -1;
    }

    SdnvDecodeResult decoded3 = SDNV_Decode64( encoded3, 5 );

    if ( decoded3.byteCount != 5 ) {
        printf( "TEST3: Invalid decoded length: %u\n", decoded3.byteCount );
        return -1;
    }

    if ( decoded3.value.u64 != original3 ) {
        printf( "TEST3: Invalid decoded value: %llu\n", decoded3.value.u64 );
        return -1;
    }

    printf( "TESTS PASSED\n" );
}
/***********************************************************************/
