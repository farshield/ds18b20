/**
 * @file tempsens.c
 * @author Gabi
 * @date 28 Jul 2017
 *
 * Temperature Sensor
 * @see https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf
 */

/******************************************************************************/
/*    INCLUDED FILES                                                          */
/******************************************************************************/

#include "std_types.h"
#include <limits.h>
#include "crc.h"
#include "onewire.h"

#include "tempsens.h"

/******************************************************************************/
/*    DEFINITIONS                                                             */
/******************************************************************************/

#define TS_SMALLEST_RESOLUTION 9  /**< Smallest configurable sensor resolution */
#define TS_DECIMAL_PLACES 10  /**< Set 10 for 1 digit after decimal point, 100 for 2 digits, etc. */
#define TS_MAX_ERROR 3  /**< Max retries until sensor status is set to error */

/** Computes the resolution from the sensor configuration byte. Resolution is in the interval [9, 12] */
#define TS_GET_RESOLUTION(configByte) \
    (((configByte >> 5) & 0x03) + TS_SMALLEST_RESOLUTION)

/******************************************************************************/
/*    PRIVATE TYPES                                                           */
/******************************************************************************/

typedef enum
{
    TS_TEMPERATURE_LSB = 0,
    TS_TEMPERATURE_MSB = 1,
    TS_TH_REGISTER = 2,
    TS_TL_REGISTER = 3,
    TS_CONFIG_REGISTER = 4,
    TS_CRC = 8
} TempSens_ScratchpadBytePos_t;

typedef struct
{
    TempSens_Device_t device[TS_MAX_DEVICES];
    uint8_t count;
} TempSens_Data_t;

/******************************************************************************/
/*    PRIVATE DATA                                                            */
/******************************************************************************/

static TempSens_Data_t TempSens_Data;
static TempSens_Data_t TempSens_DataShadow;

/******************************************************************************/
/*    PRIVATE FUNCTIONS                                                       */
/******************************************************************************/

static TempSens_Device_t * TempSens_DeviceByNumber(uint8_t deviceNr);
static int16_t TempSens_Convert(uint8_t *data);

/******************************************************************************/
/*    IMPLEMENTATION                                                          */
/******************************************************************************/

/**
 * Initialization function should be called by OneWire module before starting a
 * device search
 */
void TempSens_Init(void)
{
    TempSens_Data.count = 0;
}

/**
 * Set the states of all devices before the temperature read state of the
 * OneWire task module
 */
void TempSens_Clear(void)
{
    uint8_t devCount;
    for (devCount = 0; devCount < TempSens_Data.count; devCount++)
    {
        TempSens_Data.device[devCount].state = TS_NOT_READ_YET;
    }
}

/**
 * Called when a temperature sensor is detected
 * @param deviceNr Device number (OneWire ID buffer position)
 */
void TempSens_Discovered(uint8_t deviceNr)
{
    uint8_t count = TempSens_Data.count;

    if (count < TS_MAX_DEVICES)
    {
        TempSens_Data.device[count].number = deviceNr;
        TempSens_Data.count++;
    }
}

/**
 * Searches the temperature device data by device number
 * @param deviceNr Device number (OneWire ID buffer position)
 * @return Address to device data structure if present; NULL otherwise
 */
static TempSens_Device_t * TempSens_DeviceByNumber(uint8_t deviceNr)
{
    uint8_t devCount;
    for (devCount = 0; devCount < TempSens_Data.count; devCount++)
    {
        if (deviceNr == TempSens_Data.device[devCount].number)
        {
            return &TempSens_Data.device[devCount];
        }
    }
    return NULL;
}

/**
 * Computes the temperature from the Scratchpad data
 * @param[in] data The Scratchpad memory of the sensor
 * @return 16-bit signed temperature data with 1 decimal place
 *  Range: [-550, 1250]; Example: 253 equals 25.3 degrees Celsius
 */
static int16_t TempSens_Convert(uint8_t *data)
{
    int16_t temperature = ((uint16_t) data[TS_TEMPERATURE_MSB] << CHAR_BIT) | data[TS_TEMPERATURE_LSB];
    uint8_t resolution = TS_GET_RESOLUTION(data[TS_CONFIG_REGISTER]);
    uint8_t shiftAmount = 2 * CHAR_BIT - resolution;

    temperature = (temperature << shiftAmount) >> shiftAmount;
    temperature = (temperature * TS_DECIMAL_PLACES) >> (resolution - TS_SMALLEST_RESOLUTION + 1);
    return temperature;
}

/**
 * Processes data from Scratchpad into actual temperature values and sets the correct status
 * @param deviceNr Device number (OneWire ID buffer position)
 * @param data Data received from the sensor (should be Scratchpad data)
 * @param length Data length in bytes (should be the size of the Scratchpad)
 */
void TempSens_Process(uint8_t deviceNr, uint8_t *data, uint8_t length)
{
    TempSens_Device_t *device = TempSens_DeviceByNumber(deviceNr);
    uint8_t crc8 = 0xFF;

    if ( (TS_SCRATCHPAD_SIZE == length) && (NULL != data) && (NULL != device) )
    {
        crc8 = CRC_Compute8(data, length - 1);
        if (data[TS_CRC] == crc8)
        {
            device->temperature = TempSens_Convert(data);
            device->state = TS_READ_OK;
        }
        else
        {
            device->errorCounter++;
            if (TS_MAX_ERROR <= device->errorCounter)
            {
                device->errorCounter = 0;
                device->state = TS_READ_ERROR;
            }
        }
    }
}

/**
 * Determines the next temperature sensor to be sampled
 * @return Device number of the sensor to be sampled
 */
uint8_t TempSens_GetNextDevice(void)
{
    uint8_t devCount;
    for (devCount = 0; devCount < TempSens_Data.count; devCount++)
    {
        if (TS_NOT_READ_YET == TempSens_Data.device[devCount].state)
        {
            return TempSens_Data.device[devCount].number;
        }
    }
    return TS_SAMPLING_DONE;
}

/**
 * Retrieve data on the temperature sensors
 * @param[out] Address to an array of temperature devices
 * @return Total number of active temperature sensors
 */
uint8_t TempSens_GetDevices(TempSens_Device_t *devices[])
{
    uint8_t count = TempSens_DataShadow.count;
    if ( (0 < count) && (NULL != devices) )
    {
        *devices = &TempSens_DataShadow.device[0];
    }
    return count;
}

/**
 * Copies data from the shadow buffer to the main buffer
 */
void TempSens_CopyToShadowBuffer(void)
{
    TempSens_DataShadow = TempSens_Data;
}

/**
 * Total devices found so far
 */
uint8_t TempSens_TotalDevices(void)
{
    return TempSens_Data.count;
}
