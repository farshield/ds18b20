/**
 * @file crc.c
 * @author Gabi
 * @date 28 Jul 2017
 *
 * Cyclic Redundancy Check routines
 */

/******************************************************************************/
/*    INCLUDED FILES                                                          */
/******************************************************************************/

#include "std_types.h"
#include <limits.h>

/******************************************************************************/
/*    DEFINITIONS                                                             */
/******************************************************************************/

/** Used for computing CRC-8 */
#define CRC8_POLYNOMIAL 0x8C

/******************************************************************************/
/*    IMPLEMENTATION                                                          */
/******************************************************************************/

/**
 * @return CRC-16 initial value
 */
uint16_t CRC_Init16(void)
{
    return (uint16_t) 0xFFFF;
}

/**
 * Computes CRC-16 with the accumulator method
 * @param new_byte Inpute byte
 * @param crc16 Intermediate result
 * @return CRC-16 computed value
 */
uint16_t CRC_Compute16(uint8_t new_byte, uint16_t crc16)
{
    uint16_t x = ((crc16 >> 8) ^ new_byte) & 0xFF;

    x ^= x >> 4;
    crc16 = (crc16 << 8) ^ (x << 12) ^ (x << 5) ^ x;

    return crc16;
}

/**
 * Computes CRC-8 from an array of bytes
 * @param buffer Address to byte array
 * @param length Size in bytes of given array
 * @return CRC-8 computed value
 */
uint8_t CRC_Compute8(uint8_t *buffer, uint8_t length)
{
    uint8_t crc8 = 0, valid = 0;
    uint8_t inByte, byteCount, bitCount, mix;

    for (byteCount = 0; byteCount < length; byteCount++)
    {
        inByte = buffer[byteCount];
        if (inByte)
        {
            valid = 1;
        }
        for (bitCount = 0; bitCount < CHAR_BIT; bitCount++)
        {
            mix = (crc8 ^ inByte) & 0x01;
            crc8 >>= 1;
            if (mix)
            {
                crc8 ^= CRC8_POLYNOMIAL;
            }
            inByte >>= 1;
        }
    }
    if (!valid)
    {
        /* If all bytes are 0, return a different CRC so that the test will fail */
        return 0xFF;
    }
    return crc8;
}
