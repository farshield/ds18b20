/**
 * @file onewire.h
 * @author Gabi
 * @date 5 Jul 2017
 *
 * OneWire driver
 */

#ifndef ONEWIRE_H
#define	ONEWIRE_H

/******************************************************************************/
/*    DEFINITIONS                                                             */
/******************************************************************************/

#define OW_MAX_DEVICES 6  /**< Max number of devices on the bus that will be read */
#define OW_ID_SIZE 8  /**< ID size in bytes */

/******************************************************************************/
/*    PUBLIC TYPES                                                            */
/******************************************************************************/

typedef uint8_t OneWire_Id_t[OW_ID_SIZE];

typedef enum
{
    OW_EVENT_IDS_CHANGED,
    OW_MAX_EVENTS
} OneWire_Events_t;

/******************************************************************************/
/*    PUBLIC FUNCTIONS                                                        */
/******************************************************************************/

uint8_t OneWire_GetIDs(OneWire_Id_t *ids[]);
uint8_t OneWire_AddEventListener(OneWire_Events_t eventType, callback_t eventListener);

void OneWire_Init(void);
void OneWire_DeInit(void);
void OneWire_Task(void);
void OneWire_TimerISR(void);

#endif	/* ONEWIRE_H */
