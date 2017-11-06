/**
 * @file onewire.c
 * @author Gabi
 * @date 5 Jul 2017
 *
 * OneWire driver
 */

/******************************************************************************/
/*    INCLUDED FILES                                                          */
/******************************************************************************/

#include "std_types.h"
#include <limits.h>
#include <string.h>
#include "hwmap.h"
#include "crc.h"
#include "tempsens.h"

#include "onewire.h"

/******************************************************************************/
/*    DEFINITIONS                                                             */
/******************************************************************************/

#define OW_INBUF_SIZE 12  /**< Max input buffer size for reading from the bus */
#define OW_CMD_QUEUE_SIZE 24  /**< Max command queue size */
#define OW_NO_CONFLICT 0xFF  /**< No conflict marker in ROM Search Mode */
#define OW_NO_PARAMETER 0  /**< Used in case command parameters can be ignored */

#define OW_TIMEBASE 5  /**< Scaling factor to obtain required timings for the protocol */
#define OW_KICKSTART_TIMER 100  /**< Wait time (in timer ticks) between issuing a command after an idle state and calling the ISR */
#define OW_WAIT_PERIOD 40  /**< Number of states the Main Task should wait between searching (and sampling) */

#define OW_MAX_EVENT_LISTENERS 2  /**< Total number of listeners which can be subscribed to a given event */
#define OW_ID_FAMILY_BYTE 0  /**< Family code byte position in device ID */
#define OW_ID_CRC_BYTE 7  /**< CRC byte position in device ID */

/******************************************************************************/
/*    PRIVATE TYPES                                                           */
/******************************************************************************/

typedef enum
{
    OW_MODE_WRITE = 0,
    OW_MODE_READ = 1
} OneWire_IoMode_t;

typedef enum
{
    OW_STATUS_IDLE,
    OW_STATUS_RUNNING
} OneWire_QueueStatus_t;

typedef enum
{
    OW_DEVICE_PRESENT = 0,
    OW_NO_DEVICE_PRESENT = 1
} OneWire_AtrStatus_t;

/** Possible ROM commands that can be issued by master after a RESET pulse */
typedef enum
{
    OW_ROM_READ = 0x33,
    OW_ROM_MATCH = 0x55,
    OW_ROM_SEARCH = 0xF0,
    OW_ROM_SKIP = 0xCC,
    OW_ROM_OVERDRIVE_SKIP = 0x3C,
    OW_ROM_OVERDRIVE_MATCH = 0x69
} OneWire_RomCmd_t;

typedef enum
{
    OW_NOP,
    OW_OP_RESET,
    OW_OP_READ,
    OW_OP_WRITE_0,
    OW_OP_WRITE_1,
    OW_OP_COUNT
} OneWire_Ops_t;

typedef enum
{
    OW_ISR_STAGE1,
    OW_ISR_STAGE2,
    OW_ISR_STAGE3,
    OW_ISR_STAGE_COUNT
} OneWire_IsrState_t;

typedef enum
{
    OW_STATE_PERFORM_RESET,
    OW_STATE_CHECK_PRESENCE,
    OW_STATE_DEVICE_SEARCH,
    OW_STATE_ID_VERIFICATION,
    OW_STATE_TEMP_START_SAMPLE,
    OW_STATE_TEMP_READ,
    OW_STATE_TEMP_PROCESS,
    OW_STATE_WAIT
} OneWire_State_t;

/** High-level 1-Wire commands issued from Main Task */
typedef enum
{
    OW_CMD_RESET,
    OW_CMD_WRITE_BYTE,
    OW_CMD_READ_BYTE,
    OW_CMD_SEARCH,
    OW_CMD_IDLE
} OneWire_CmdName_t;

typedef enum
{
    OW_READ_ACTUAL_BIT = 0,
    OW_READ_COMPLEMENT_BIT = 1,
    OW_WRITE_ROM_BIT = 2,
    OW_SEARCH_STEPS = 3
} OneWire_SearchSteps_t;

typedef enum
{
    OW_MAKE_DECISION = 0b00,
    OW_TAKE_PATH_0 = 0b10,
    OW_TAKE_PATH_1 = 0b01,
    OW_END_SEARCH = 0b11
} OneWire_BstPath_t;

typedef enum
{
    OW_SEARCH_START_NEXT_CYCLE,
    OW_SEARCH_PERFORMED,
    OW_TEMP_SAMPLING_STARTED
} OneWire_SearchStatus_t;

typedef struct
{
    OneWire_CmdName_t name;
    uint8_t param;
} OneWire_Cmd_t;

typedef struct
{
    OneWire_QueueStatus_t status;
    OneWire_Cmd_t cmd[OW_CMD_QUEUE_SIZE];
    OneWire_Cmd_t *head;
    OneWire_Cmd_t *tail;
    OneWire_Cmd_t *buffer_start;
    OneWire_Cmd_t *buffer_end;
    uint8_t count;
    uint8_t uStep;
} OneWire_CmdQueue_t;

typedef struct
{
    OneWire_AtrStatus_t atr;
    OneWire_Id_t id[OW_MAX_DEVICES];
    uint8_t count;
} OneWire_Devices_t;

typedef struct
{
    uint8_t conflict[OW_ID_SIZE];
    uint8_t bitPos;
    uint8_t bytePos;
    uint8_t actualBit : 1;
    uint8_t complementBit : 1;
    OneWire_SearchStatus_t status;
} OneWire_SearchData_t;

typedef struct
{
    uint8_t buffer[OW_INBUF_SIZE];
    uint8_t count;
    uint8_t prevCrc[OW_MAX_DEVICES];
    uint8_t prevDevCount;
} OneWire_InputData_t;

typedef struct
{
    callback_t listener[OW_MAX_EVENT_LISTENERS];
    uint8_t count;
} OneWire_EventListeners_t;

/******************************************************************************/
/*    PRIVATE DATA                                                            */
/******************************************************************************/

/** Standard Speed Mode Time Table [microseconds] */
uint16_t OneWire_StandardSpeed[OW_OP_COUNT][OW_ISR_STAGE_COUNT] =
{
    {  0,  0,   0}, /* No Operation */
    {480, 70, 410}, /* Reset */
    {  6,  9,  55}, /* Read 1 bit */
    { 60, 10,   0}, /* Write 0 */
    {  6, 64,   0}  /* Write 1 */
};

/* Data model */
static OneWire_CmdQueue_t OneWire_CmdQueue;
static OneWire_Devices_t OneWire_Devices;
static OneWire_Devices_t OneWire_DevicesShadow;
static OneWire_SearchData_t OneWire_SearchData;
static OneWire_InputData_t OneWire_InputData;

/* State machine internal variables */
static OneWire_IsrState_t OneWire_IsrState;
static OneWire_IsrState_t OneWire_IsrNextState;
static OneWire_State_t OneWire_CurrentState;
static OneWire_State_t OneWire_NextState;

/* Event listener data */
static OneWire_EventListeners_t OneWire_EventListeners[OW_MAX_EVENTS];

/******************************************************************************/
/*    PRIVATE FUNCTIONS                                                       */
/******************************************************************************/

/** MCU peripherals helper functions */
static inline void OneWire_DriveBusLow(void);
static inline void OneWire_ReleaseBus(void);
static inline uint8_t OneWire_ReadBus(void);
static bool_t OneWire_SetTimer(uint16_t delay);
static bool_t OneWire_Delay(OneWire_Ops_t operation, OneWire_IsrState_t stage);

/** State machine helper functions */
static OneWire_State_t OneWire_SearchOrSample(void);
static OneWire_Ops_t OneWire_GetNextOp(void);
static void OneWire_PostProcess(uint8_t busState, OneWire_Ops_t operation);
static OneWire_Ops_t OneWire_HandleConflict(uint8_t conflictMarker);
static bool_t OneWire_ConflictDetected(void);
static uint8_t OneWire_LastBitConflict(void);
static void OneWire_SelectDevice(uint8_t deviceNr);
static void OneWire_CopyToShadowBuffer(void);

/* Event listener methods */
static void OneWire_NotifyListeners(OneWire_Events_t eventType);
static void OneWire_ProcessNotifications(OneWire_Events_t eventType);

/** Circular buffer helper functions */
static void OneWire_ClearCmdQueue(void);
static bool_t OneWire_Enqueue(OneWire_CmdName_t name, uint8_t param);
static bool_t OneWire_Dequeue(void);
static inline OneWire_Cmd_t OneWire_GetCurrentCmd(void);
static inline bool_t OneWire_UpdateParam(uint8_t param);

/******************************************************************************/
/*    IMPLEMENTATION                                                          */
/******************************************************************************/

/**
 * OneWire setup function should be called by the main initialization routine
 */
void OneWire_Init(void)
{
    /* Set bus to input (or HiZ) as the default state */
    OneWire_ReleaseBus();

    /* Clear device count */
    OneWire_Devices.atr = OW_NO_DEVICE_PRESENT;
    OneWire_Devices.count = 0;
    OneWire_DevicesShadow.atr = OW_NO_DEVICE_PRESENT;
    OneWire_DevicesShadow.count = 0;
    OneWire_SearchData.status = OW_SEARCH_START_NEXT_CYCLE;

    /* Clear command queue */
    OneWire_CmdQueue.status = OW_STATUS_IDLE;
    OneWire_CmdQueue.buffer_start = &OneWire_CmdQueue.cmd[0];
    OneWire_CmdQueue.buffer_end = &OneWire_CmdQueue.cmd[OW_CMD_QUEUE_SIZE - 1];
    OneWire_ClearCmdQueue();

    /* Clear input data and event listeners */
    OneWire_InputData.count = 0;
    OneWire_InputData.prevDevCount = 0;
    OneWire_EventListeners[OW_EVENT_IDS_CHANGED].count = 0;

    /* FSM initial states */
    OneWire_IsrState = OW_ISR_STAGE1;
    OneWire_IsrNextState = OW_ISR_STAGE1;
    OneWire_CurrentState = OW_STATE_PERFORM_RESET;
    OneWire_NextState = OW_STATE_PERFORM_RESET;

    /* Timer setup */
    OW_TIMER_TON = 0;
    OW_TIMER_TCS = 0;
    OW_TIMER_TCKPS = 0b01;
    OW_TIMER_T32 = 0;
    OW_TIMER_VALUE = 0;

    /* Interrupt setup */
    OW_TIMER_IP = 1;
    OW_TIMER_IF = 0;
    OW_TIMER_IE = 0;
}

/**
 * OneWire deactivate all communication
 */
void OneWire_DeInit(void)
{
    OneWire_ReleaseBus();
    OW_TIMER_TON = 0;
    OW_TIMER_IF = 0;
    OW_TIMER_IE = 0;
}

/**
 * OneWire Main Task should be called by the Scheduler periodically
 */
void OneWire_Task(void)
{
    static uint16_t waitCounter = 0;
    static uint8_t temperatureSensor = 0;
    bool_t waitNextCall = FALSE;
    bool_t resetPerformed = FALSE;
    bool_t deviceIdValid = FALSE;
    uint8_t crc8 = 0, readCount = 0;

    while (FALSE == waitNextCall)
    {
        OneWire_CurrentState = OneWire_NextState;
        switch (OneWire_CurrentState)
        {
        case OW_STATE_PERFORM_RESET:
            OneWire_Devices.atr = OW_NO_DEVICE_PRESENT;
            OneWire_Enqueue(OW_CMD_RESET, OW_NO_PARAMETER);
            OneWire_NextState = OW_STATE_CHECK_PRESENCE;
            waitNextCall = TRUE;
            break;

        case OW_STATE_CHECK_PRESENCE:
            if (OW_DEVICE_PRESENT == OneWire_Devices.atr)
            {
                /* Answer to reset detected */
                resetPerformed = TRUE;
                OneWire_NextState = OneWire_SearchOrSample();
            }
            else
            {
                /* No answer to reset; clear devices */
                OneWire_Devices.count = 0;
                OneWire_CopyToShadowBuffer();
                TempSens_Init();
                TempSens_CopyToShadowBuffer();
                OneWire_ProcessNotifications(OW_EVENT_IDS_CHANGED);
                /* Try again after a wait period */
                OneWire_SearchData.status = OW_SEARCH_START_NEXT_CYCLE;
                OneWire_NextState = OW_STATE_WAIT;
            }
            waitNextCall = FALSE;
            break;

        case OW_STATE_WAIT:
            if (waitCounter < OW_WAIT_PERIOD)
            {
                waitCounter++;
                OneWire_NextState = OW_STATE_WAIT;
                waitNextCall = TRUE;
            }
            else
            {
                waitCounter = 0;
                OneWire_NextState = OW_STATE_PERFORM_RESET;
                waitNextCall = FALSE;
            }
            break;

        case OW_STATE_DEVICE_SEARCH:
            OneWire_InputData.count = 0;
            OneWire_SearchData.bitPos = 0;
            OneWire_SearchData.bytePos = 0;
            if (resetPerformed)
            {
                /* Initial reset already performed */
                OneWire_Devices.count = 0;
                TempSens_Init();
                memset(OneWire_SearchData.conflict, 0, OW_ID_SIZE);
                resetPerformed = FALSE;
            }
            else
            {
                OneWire_Enqueue(OW_CMD_RESET, OW_NO_PARAMETER);
            }
            OneWire_Enqueue(OW_CMD_WRITE_BYTE, OW_ROM_SEARCH);
            OneWire_Enqueue(OW_CMD_SEARCH, OneWire_LastBitConflict());
            OneWire_NextState = OW_STATE_ID_VERIFICATION;
            waitNextCall = TRUE;
            break;

        case OW_STATE_ID_VERIFICATION:
            deviceIdValid = FALSE;
            if ((OW_ID_SIZE == OneWire_InputData.count) &&
                (OneWire_Devices.count < OW_MAX_DEVICES))
            {
                crc8 = CRC_Compute8(OneWire_InputData.buffer, OW_ID_SIZE - 1);
                if (OneWire_InputData.buffer[OW_ID_SIZE - 1] == crc8)
                {
                    deviceIdValid = TRUE;
                }
            }
            if (deviceIdValid)
            {
                /* 8 bytes have been read, CRC is OK and device buffer is not full yet */
                if (TS_DS18B20_FAMILY == OneWire_InputData.buffer[OW_ID_FAMILY_BYTE])
                {
                    TempSens_Discovered(OneWire_Devices.count);
                }
                /* Copy device ID from the input buffer */
                memcpy(OneWire_Devices.id[OneWire_Devices.count], OneWire_InputData.buffer, OW_ID_SIZE);
                OneWire_Devices.count++;
                if (OneWire_ConflictDetected())
                {
                    /* Continue searching for devices */
                    OneWire_NextState = OW_STATE_DEVICE_SEARCH;
                }
                else
                {
                    /* No conflicts; Notify that all devices should be discovered */
                    OneWire_CopyToShadowBuffer();
                    OneWire_ProcessNotifications(OW_EVENT_IDS_CHANGED);
                    OneWire_SearchData.status = OW_SEARCH_PERFORMED;
                    OneWire_NextState = OW_STATE_PERFORM_RESET;
                }
            }
            else
            {
                /* Search failed; try again after a wait period */
                OneWire_SearchData.status = OW_SEARCH_START_NEXT_CYCLE;
                OneWire_NextState = OW_STATE_WAIT;
            }
            waitNextCall = FALSE;
            break;

        case OW_STATE_TEMP_START_SAMPLE:
            OneWire_SearchData.status = OW_TEMP_SAMPLING_STARTED;
            OneWire_Enqueue(OW_CMD_WRITE_BYTE, OW_ROM_SKIP);
            OneWire_Enqueue(OW_CMD_WRITE_BYTE, TS_CONVERT_T);
            OneWire_NextState = OW_STATE_WAIT;
            waitNextCall = FALSE;
            break;

        case OW_STATE_TEMP_READ:
            if (resetPerformed)
            {
                /* Initial reset already performed */
                TempSens_Clear();
                temperatureSensor = TempSens_GetNextDevice();
                resetPerformed = FALSE;
            }
            else
            {
                OneWire_Enqueue(OW_CMD_RESET, OW_NO_PARAMETER);
            }
            OneWire_SelectDevice(temperatureSensor);
            OneWire_Enqueue(OW_CMD_WRITE_BYTE, TS_READ_SCRATCHPAD);
            OneWire_InputData.count = 0;
            for (readCount = 0; readCount < TS_SCRATCHPAD_SIZE; readCount++)
            {
                OneWire_Enqueue(OW_CMD_READ_BYTE, OW_NO_PARAMETER);
            }
            OneWire_NextState = OW_STATE_TEMP_PROCESS;
            waitNextCall = TRUE;
            break;

        case OW_STATE_TEMP_PROCESS:
            TempSens_Process(temperatureSensor, OneWire_InputData.buffer, OneWire_InputData.count);
            temperatureSensor = TempSens_GetNextDevice();
            if (TS_SAMPLING_DONE == temperatureSensor)
            {
                TempSens_CopyToShadowBuffer();
                OneWire_SearchData.status = OW_SEARCH_START_NEXT_CYCLE;
                OneWire_NextState = OW_STATE_PERFORM_RESET;
            }
            else
            {
                OneWire_NextState = OW_STATE_TEMP_READ;
            }
            waitNextCall = FALSE;
            break;

        default:
            OneWire_NextState = OW_STATE_PERFORM_RESET;
            waitNextCall = FALSE;
            break;
        }
    }
}

/**
 * OneWire Interrupt Handler
 */
void OneWire_TimerISR(void)
{
    static OneWire_Ops_t currentOperation = OW_NOP;
    uint8_t busState = 0;
    bool_t waitNextCall = FALSE;

    OneWire_SetTimer(0);
    while (FALSE == waitNextCall)
    {
        OneWire_IsrState = OneWire_IsrNextState;
        switch(OneWire_IsrState)
        {
        case OW_ISR_STAGE1:
            currentOperation = OneWire_GetNextOp();
            if (OW_NOP == currentOperation)
            {
                /* No commands in queue; do nothing for now and wait until next call */
                OneWire_CmdQueue.status = OW_STATUS_IDLE;
                waitNextCall = TRUE;
                OneWire_IsrNextState = OW_ISR_STAGE1;
            }
            else
            {
                /* Command found; start first stage of operation */
                OneWire_DriveBusLow();
                waitNextCall = OneWire_Delay(currentOperation, OW_ISR_STAGE1);
                OneWire_IsrNextState = OW_ISR_STAGE2;
            }
            break;

        case OW_ISR_STAGE2:
            OneWire_ReleaseBus();
            waitNextCall = OneWire_Delay(currentOperation, OW_ISR_STAGE2);
            OneWire_IsrNextState = OW_ISR_STAGE3;
            break;

        case OW_ISR_STAGE3:
            busState = OneWire_ReadBus();
            OneWire_PostProcess(busState, currentOperation);
            waitNextCall = OneWire_Delay(currentOperation, OW_ISR_STAGE3);
            OneWire_IsrNextState = OW_ISR_STAGE1;
            break;

        default:
            OneWire_IsrNextState = OW_ISR_STAGE1;
            waitNextCall = FALSE;
            break;
        }
    }
}

/**
 * Retrieve data on the IDs of the active devices
 * @param ids[out] Address to an array of device IDs
 * @return Total number of active devices on the bus
 */
uint8_t OneWire_GetIDs(OneWire_Id_t *ids[])
{
    uint8_t count = OneWire_DevicesShadow.count;
    if ( (0 < count) && (NULL != ids) )
    {
        *ids = OneWire_DevicesShadow.id;
    }
    return count;
}

/**
 * Subscribe listener to OneWire events
 * @param eventType Type of event raised
 * @param eventListener Callback address of function
 * @return TRUE if listener was added successfully; FALSE otherwise
 */
uint8_t OneWire_AddEventListener(OneWire_Events_t eventType, callback_t eventListener)
{
    if (OneWire_EventListeners[eventType].count < OW_MAX_EVENT_LISTENERS)
    {
        OneWire_EventListeners[eventType].listener[OneWire_EventListeners[eventType].count] = eventListener;
        OneWire_EventListeners[eventType].count++;
        return TRUE;
    }
    return FALSE;
}

/**
 * Notifies all the subscribed event listeners (if any)
 * @param eventType Type of event raised
 */
static void OneWire_NotifyListeners(OneWire_Events_t eventType)
{
    uint8_t idx;
    for (idx = 0; idx < OneWire_EventListeners[eventType].count; idx++)
    {
        if (NULL != OneWire_EventListeners[eventType].listener[idx])
        {
            (OneWire_EventListeners[eventType].listener[idx])();
        }
    }
}

/**
 * Notifies event listeners on-change or periodically
 * @param eventType Type of event raised
 */
static void OneWire_ProcessNotifications(OneWire_Events_t eventType)
{
    uint8_t idx;
    bool_t devicesChanged = FALSE;

    switch (eventType)
    {
    case OW_EVENT_IDS_CHANGED:
        /* Notify on-change only */
        if (OneWire_InputData.prevDevCount != OneWire_Devices.count)
        {
            devicesChanged = TRUE;
        }
        else
        {
            for (idx = 0; idx < OneWire_Devices.count; idx++)
            {
                /* Check CRC for every active device */
                if (OneWire_InputData.prevCrc[idx] != OneWire_Devices.id[idx][OW_ID_CRC_BYTE])
                {
                    devicesChanged = TRUE;
                }
            }
        }
        if (devicesChanged)
        {
            OneWire_NotifyListeners(eventType);
            OneWire_InputData.prevDevCount = OneWire_Devices.count;
            for (idx = 0; idx < OneWire_Devices.count; idx++)
            {
                OneWire_InputData.prevCrc[idx] = OneWire_Devices.id[idx][OW_ID_CRC_BYTE];
            }
        }
        break;

    default:
        /* Do nothing */
        break;
    }
}

/**
 * Determines the next action: search for devices or sample temperature sensors
 * @return Next step to be taken in the Task State Machine
 */
static OneWire_State_t OneWire_SearchOrSample(void)
{
    OneWire_State_t nextTaskState = OW_STATE_DEVICE_SEARCH;
    switch (OneWire_SearchData.status)
    {
    case OW_SEARCH_START_NEXT_CYCLE:
        nextTaskState = OW_STATE_DEVICE_SEARCH;
        break;
    case OW_SEARCH_PERFORMED:
        if (TempSens_TotalDevices())
        {
            /* Temperature sensors present */
            nextTaskState = OW_STATE_TEMP_START_SAMPLE;
        }
        else
        {
            /* No temperature sensors detected; schedule another device search */
            OneWire_SearchData.status = OW_SEARCH_START_NEXT_CYCLE;
            nextTaskState = OW_STATE_WAIT;
        }
        break;
    case OW_TEMP_SAMPLING_STARTED:
        nextTaskState = OW_STATE_TEMP_READ;
        break;
    default:
        nextTaskState = OW_STATE_DEVICE_SEARCH;
        break;
    }
    return nextTaskState;
}

/**
 * Determine next operation
 * @return Next operation to be executed by the ISR State Machine
 */
static OneWire_Ops_t OneWire_GetNextOp(void)
{
    OneWire_Ops_t nextOp = OW_NOP;
    OneWire_Cmd_t currentCmd = OneWire_GetCurrentCmd();

    switch (currentCmd.name)
    {
    case OW_CMD_RESET:
        nextOp = OW_OP_RESET;
        break;

    case OW_CMD_WRITE_BYTE:
        if ( 0 < (currentCmd.param & (0x01 << OneWire_CmdQueue.uStep)) )
        {
            nextOp = OW_OP_WRITE_1;
        }
        else
        {
            nextOp = OW_OP_WRITE_0;
        }
        break;

    case OW_CMD_READ_BYTE:
        nextOp = OW_OP_READ;
        break;

    case OW_CMD_SEARCH:
        if (OW_WRITE_ROM_BIT == (OneWire_CmdQueue.uStep % OW_SEARCH_STEPS))
        {
            /* Check for potential conflicts */
            nextOp = OneWire_HandleConflict(currentCmd.param);
        }
        else
        {
            /* Still at reading phase */
            nextOp = OW_OP_READ;
        }
        break;

    case OW_CMD_IDLE:
    default:
        nextOp = OW_NOP;
        break;
    }

    return nextOp;
}

/**
 * Perform additional actions in the final ISR stage
 * @param busState Value read from the 1-Wire bus
 * @param operation Current operation that is being executed
 */
static void OneWire_PostProcess(uint8_t busState, OneWire_Ops_t operation)
{
    static uint8_t valueRead = 0;
    OneWire_SearchSteps_t searchStep = OW_READ_ACTUAL_BIT;
    OneWire_Cmd_t currentCmd = OneWire_GetCurrentCmd();

    switch (currentCmd.name)
    {
    case OW_CMD_RESET:
        if (0 == busState)
        {
            OneWire_Devices.atr = OW_DEVICE_PRESENT;
        }
        else
        {
            OneWire_Devices.atr = OW_NO_DEVICE_PRESENT;
        }
        OneWire_UpdateParam(busState);
        OneWire_Dequeue();
        break;

    case OW_CMD_WRITE_BYTE:
        OneWire_CmdQueue.uStep++;
        if (CHAR_BIT <= OneWire_CmdQueue.uStep)
        {
            OneWire_Dequeue();
        }
        break;

    case OW_CMD_READ_BYTE:
        valueRead |= ((busState & 0x01u) << OneWire_CmdQueue.uStep);
        OneWire_CmdQueue.uStep++;
        if (CHAR_BIT <= OneWire_CmdQueue.uStep)
        {
            if (OW_INBUF_SIZE <= OneWire_InputData.count)
            {
                OneWire_InputData.count = 0;
            }
            OneWire_InputData.buffer[OneWire_InputData.count] = valueRead;
            OneWire_InputData.count++;
            OneWire_UpdateParam(valueRead);
            valueRead = 0;
            OneWire_Dequeue();
        }
        break;

    case OW_CMD_SEARCH:
        searchStep = OneWire_CmdQueue.uStep % OW_SEARCH_STEPS;
        switch (searchStep)
        {
        case OW_READ_ACTUAL_BIT:
            OneWire_SearchData.actualBit = busState;
            OneWire_CmdQueue.uStep++;
            break;

        case OW_READ_COMPLEMENT_BIT:
            OneWire_SearchData.complementBit = busState;
            OneWire_CmdQueue.uStep++;
            break;

        case OW_WRITE_ROM_BIT:
            if ( (1 == OneWire_SearchData.actualBit) &&
                 (1 == OneWire_SearchData.complementBit) )
            {
                /* No devices still active */
                OneWire_Dequeue();
            }
            else
            {
                if (OW_OP_WRITE_1 == operation)
                {
                    OneWire_InputData.buffer[OneWire_SearchData.bytePos] |= (0x01 << OneWire_SearchData.bitPos);
                }
                else
                {
                    OneWire_InputData.buffer[OneWire_SearchData.bytePos] &= ~(0x01 << OneWire_SearchData.bitPos);
                }
                OneWire_SearchData.bitPos++;
                if (CHAR_BIT <= OneWire_SearchData.bitPos)
                {
                    OneWire_SearchData.bitPos = 0;
                    OneWire_SearchData.bytePos++;
                    OneWire_InputData.count = OneWire_SearchData.bytePos;
                }

                OneWire_CmdQueue.uStep++;
                if (OW_ID_SIZE * CHAR_BIT * OW_SEARCH_STEPS <= OneWire_CmdQueue.uStep)
                {
                    /* Search finished */
                    OneWire_Dequeue();
                }
            }
            break;

        default:
            break;
        }
        break;

    case OW_CMD_IDLE:
    default:
        break;
    }
}

/**
 * Determines the next step in the Binary Search Tree algorithm
 * @param conflictMarker Last conflict bit position (0-63 or 255 if no conflict)
 * @return Next 1-Wire operation to be executed by ISR FSM
 */
static OneWire_Ops_t OneWire_HandleConflict(uint8_t conflictMarker)
{
    OneWire_Ops_t nextOp = OW_NOP;
    uint8_t romBit = 0;
    uint8_t *lastDeviceId = NULL;
    OneWire_BstPath_t bstPath = (OneWire_SearchData.complementBit << 1) | OneWire_SearchData.actualBit;

    switch (bstPath)
    {
    case OW_MAKE_DECISION:
        /* Conflict detected */
        romBit = OneWire_SearchData.bytePos * CHAR_BIT + OneWire_SearchData.bitPos;
        if ( (romBit < conflictMarker) && (OW_NO_CONFLICT != conflictMarker) )
        {
            /* Write bit from last device found (if existing) */
            if (0 < OneWire_Devices.count)
            {
                lastDeviceId = &OneWire_Devices.id[OneWire_Devices.count - 1][0];
                if (lastDeviceId[OneWire_SearchData.bytePos] & (0x01 << OneWire_SearchData.bitPos))
                {
                    nextOp = OW_OP_WRITE_1;
                }
                else
                {
                    nextOp = OW_OP_WRITE_0;
                }
            }
        }
        else if (romBit == conflictMarker)
        {
            /* Clear conflict and write 1 to bus */
            OneWire_SearchData.conflict[OneWire_SearchData.bytePos] &= ~(0x01 << OneWire_SearchData.bitPos);
            nextOp = OW_OP_WRITE_1;
        }
        else
        {
            /* Mark conflict and write 0 to bus */
            OneWire_SearchData.conflict[OneWire_SearchData.bytePos] |= (0x01 << OneWire_SearchData.bitPos);
            nextOp = OW_OP_WRITE_0;
        }
        break;

    case OW_TAKE_PATH_0:
        /* No conflict; Actual = 0 and Complement = 1, so write 0 */
        nextOp = OW_OP_WRITE_0;
        break;

    case OW_TAKE_PATH_1:
        /* No conflict; Actual = 1 and Complement = 0, so write 1 */
        nextOp = OW_OP_WRITE_1;
        break;

    case OW_END_SEARCH:
        /* No devices are responding anymore; Write something random on the bus
        and wait for the PostProcess to terminate the search */
        nextOp = OW_OP_WRITE_0;
        break;

    default:
        break;
    }
    return nextOp;
}

/**
 * Checks if conflicts were detected in the previous search step
 * @return TRUE if conflict buffer is non-empty; FALSE otherwise
 */
static bool_t OneWire_ConflictDetected(void)
{
    uint8_t byteCount = 0;
    for (byteCount = 0; byteCount < OW_ID_SIZE; byteCount++)
    {
        if (OneWire_SearchData.conflict[byteCount])
        {
            return TRUE;
        }
    }
    return FALSE;
}

/**
 * Last bit position where a conflict has been found
 * @return 0xFF if no conflict found or a number between 0 and 63 otherwise
 */
static uint8_t OneWire_LastBitConflict(void)
{
    int8_t byteCount = 0;
    int8_t bitCount = 0;
    for (byteCount = OW_ID_SIZE - 1; 0 <= byteCount; byteCount--)
    {
        if (0 < OneWire_SearchData.conflict[byteCount])
        {
            for (bitCount = CHAR_BIT - 1; 0 <= bitCount; bitCount--)
            {
                if ( 0 < (OneWire_SearchData.conflict[byteCount] & (0x01 << bitCount)) )
                {
                    return byteCount * CHAR_BIT + bitCount;
                }
            }
        }
    }
    return OW_NO_CONFLICT;
}

/**
 * Select device by Match ROM commands
 * @param deviceNr Device number (OneWire ID buffer position)
 */
static void OneWire_SelectDevice(uint8_t deviceNr)
{
    uint8_t byteCount;
    OneWire_Enqueue(OW_CMD_WRITE_BYTE, OW_ROM_MATCH);
    for (byteCount = 0; byteCount < OW_ID_SIZE; byteCount++)
    {
        OneWire_Enqueue(OW_CMD_WRITE_BYTE, OneWire_Devices.id[deviceNr][byteCount]);
    }
}

/**
 * Copies data from the shadow buffer to the main buffer
 */
static void OneWire_CopyToShadowBuffer(void)
{
    uint8_t devIndex;

    OneWire_DevicesShadow.atr = OneWire_Devices.atr;
    OneWire_DevicesShadow.count = OneWire_Devices.count;
    for (devIndex = 0; devIndex < OneWire_Devices.count; devIndex++)
    {
        memcpy(OneWire_DevicesShadow.id[devIndex], OneWire_Devices.id[devIndex], sizeof(OneWire_Id_t));
    }
}

/**
 * Helper function to set 1-Wire line to output and logic low
 */
static inline void OneWire_DriveBusLow(void)
{
    OW_IOMODE = OW_MODE_WRITE;
    OW_OUTPUT = 0;
}

/**
 * Helper function to set 1-Wire to input (or HiZ - default)
 */
static inline void OneWire_ReleaseBus(void)
{
    OW_IOMODE = OW_MODE_READ;
}

/**
 * Helper function to read the current logic level of the 1-Wire bus
 * @return Logic level
 */
static inline uint8_t OneWire_ReadBus(void)
{
    return OW_INPUT;
}

/**
 * Helper function to start timer if necessary
 * @param delay Delay in timer ticks
 * @return TRUE if timer has been started; FALSE otherwise
 */
static bool_t OneWire_SetTimer(uint16_t delay)
{
    bool_t timerStarted = FALSE;
    if (0 < delay)
    {
        timerStarted = TRUE;
        OW_TIMER_VALUE = 0xFFFFu - delay;
        OW_TIMER_IE = 1;
        OW_TIMER_TON = 1;
    }
    else
    {
        OW_TIMER_IE = 0;
        OW_TIMER_TON = 0;
    }
    return timerStarted;
}

/**
 * Helper function to configure the delay for the current stage of the specified command
 * @param operation One of the 4 basic commands
 * @param stage Stage number of specified command
 * @return TRUE if delay is non-zero and timer has been started; FALSE otherwise
 */
static bool_t OneWire_Delay(OneWire_Ops_t operation, OneWire_IsrState_t stage)
{
    uint16_t computedDelay = OneWire_StandardSpeed[operation][stage] * OW_TIMEBASE;
    return OneWire_SetTimer(computedDelay);
}

/**
 * Empties the command circular buffer
 */
static void OneWire_ClearCmdQueue(void)
{
    OneWire_CmdQueue.head = OneWire_CmdQueue.buffer_start;
    OneWire_CmdQueue.tail = OneWire_CmdQueue.buffer_start;
    OneWire_CmdQueue.uStep = 0;
    OneWire_CmdQueue.count = 0;
}

/**
 * Adds a command to the circular buffer
 * @param name Command name
 * @param param Command parameter (OW_NO_PARAMETER if not required)
 * @return TRUE if command has been added successfully; FALSE otherwise
 */
static bool_t OneWire_Enqueue(OneWire_CmdName_t name, uint8_t param)
{
    bool_t opSuccess = FALSE;

    if (OW_CMD_QUEUE_SIZE > OneWire_CmdQueue.count)
    {
        opSuccess = TRUE;
        OneWire_CmdQueue.count++;
        (*OneWire_CmdQueue.tail).name = name;
        (*OneWire_CmdQueue.tail).param = param;

        if (OneWire_CmdQueue.buffer_end == OneWire_CmdQueue.tail)
        {
            OneWire_CmdQueue.tail = OneWire_CmdQueue.buffer_start;

        }
        else
        {
            OneWire_CmdQueue.tail++;
        }

        if (OW_STATUS_IDLE == OneWire_CmdQueue.status)
        {
            OneWire_CmdQueue.status = OW_STATUS_RUNNING;
            OneWire_SetTimer(OW_KICKSTART_TIMER);
        }
    }
    return opSuccess;
}

/**
 * Removes top command from the circular buffer
 * @return TRUE if command has been removed successfully; FALSE otherwise
 */
static bool_t OneWire_Dequeue(void)
{
    bool_t alreadyEmpty = TRUE;

    if (0 < OneWire_CmdQueue.count)
    {
        alreadyEmpty = FALSE;
        OneWire_CmdQueue.count--;
        OneWire_CmdQueue.uStep = 0;
        if (OneWire_CmdQueue.buffer_end == OneWire_CmdQueue.head)
        {
            OneWire_CmdQueue.head = OneWire_CmdQueue.buffer_start;

        }
        else
        {
            OneWire_CmdQueue.head++;
        }
    }
    return alreadyEmpty;
}

/**
 * Fetch command from head of buffer
 * @return Current command to be executed
 */
static inline OneWire_Cmd_t OneWire_GetCurrentCmd(void)
{
    OneWire_Cmd_t command = {OW_CMD_IDLE, 0};
    if (0 < OneWire_CmdQueue.count)
    {
        command = *OneWire_CmdQueue.head;
    }
    return command;
}

/**
 * Update the parameter of the current command
 * @param param Parameter update value
 * @return TRUE if buffer is empty and update failed; FALSE otherwise
 */
static inline bool_t OneWire_UpdateParam(uint8_t param)
{
    bool_t isEmpty = TRUE;

    if (0 < OneWire_CmdQueue.count)
    {
        isEmpty = FALSE;
        (*OneWire_CmdQueue.head).param = param;
    }
    return isEmpty;
}
