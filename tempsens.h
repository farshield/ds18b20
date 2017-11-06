/**
 * @file tempsens.h
 * @author Gabi
 * @date 28 Jul 2017
 *
 * Temperature Sensor
 * @see https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf
 */

/*******************************************************************************
 *    DEFINITIONS
 ******************************************************************************/

#ifndef TEMPSENS_H
#define	TEMPSENS_H

#define TS_MAX_DEVICES 4  /**< Maximum number of temperature sensors */
#define TS_DS18B20_FAMILY 0x28  /**< Sensor ID family code */
#define TS_CONVERT_T 0x44  /**< Command code for starting a temperature sample */
#define TS_READ_SCRATCHPAD 0xBE  /**< Command code for starting a Scratchpad read */
#define TS_SCRATCHPAD_SIZE 9  /**< Size of sensor Scratchpad in bytes */
#define TS_SAMPLING_DONE 0xFF  /**< Status code returned after all sensors have been sampled */

/*******************************************************************************
 *    PUBLIC TYPES
 ******************************************************************************/

typedef enum
{
    TS_READ_OK = 0x00,
    TS_READ_ERROR = 0x01,
    TS_NOT_READ_YET = 0x0A
} TempSens_Device_State_t;

typedef struct
{
    uint8_t number;
    int16_t temperature;
    TempSens_Device_State_t state;
    uint8_t errorCounter;
} TempSens_Device_t;

/*******************************************************************************
 *    PUBLIC FUNCTIONS
 ******************************************************************************/

uint8_t TempSens_GetDevices(TempSens_Device_t *devices[]);

void TempSens_Init(void);
void TempSens_Clear(void);
void TempSens_Discovered(uint8_t deviceNr);
void TempSens_Process(uint8_t deviceNr, uint8_t *data, uint8_t length);
uint8_t TempSens_GetNextDevice(void);
void TempSens_CopyToShadowBuffer(void);
uint8_t TempSens_TotalDevices(void);

#endif	/* TEMPSENS_H */
