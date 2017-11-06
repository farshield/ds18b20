# DS18B20

1-Wire and DS18B20 temperature sensor driver for PIC24 controllers

## Usage

Make sure `OneWire_Init()` is called once at the initialization sequence. `OneWire_Task()` must be called cyclically (for example every 500 milliseconds).

The timer interrupt service routine must call the OneWire handler like this:

```C
void _ISR_NO_PSV OW_TIMER_INTERRUPT(void)
{
    IFS0bits.T2IF = 0;
    OneWire_TimerISR();
}
```
