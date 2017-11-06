/**
 * @file hwmap.h
 * @author Gabi
 * @date 28 Jul 2017
 * @brief Hardware Map
 */

#ifndef HWMAP_H
#define	HWMAP_H

/******************************************************************************/
/*    INCLUDED FILES                                                          */
/******************************************************************************/

#include <xc.h>

/******************************************************************************/
/*    DEFINITIONS                                                             */
/******************************************************************************/

/* OneWire bidirectional PORTB[3] */
#define OW_OUTPUT               LATBbits.LATB3
#define OW_INPUT                PORTBbits.RB3
#define OW_IOMODE               TRISBbits.TRISB3
#define OW_TIMER_TCS            T2CONbits.TCS
#define OW_TIMER_TON            T2CONbits.TON
#define OW_TIMER_TCKPS          T2CONbits.TCKPS
#define OW_TIMER_T32            T2CONbits.T32
#define OW_TIMER_VALUE          TMR2
#define OW_TIMER_IP             IPC1bits.T2IP
#define OW_TIMER_IF             IFS0bits.T2IF
#define OW_TIMER_IE             IEC0bits.T2IE
#define OW_TIMER_INTERRUPT      _T2Interrupt

#endif	/* HWMAP_H */
