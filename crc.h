/**
 * @file crc.h
 * @author Gabi
 * @date 28 Jul 2017
 *
 * Cyclic Redundancy Check routines
 */

#ifndef CRC_H
#define	CRC_H

/******************************************************************************/
/*    PUBLIC FUNCTIONS                                                        */
/******************************************************************************/

uint16_t CRC_Init16(void);
uint16_t CRC_Compute16(uint8_t new_byte, uint16_t crc16);
uint8_t CRC_Compute8(uint8_t *buffer, uint8_t length);

#endif	/* CRC_H */
