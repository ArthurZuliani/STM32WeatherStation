# STM32L052WeatherStation :thermometer:

The objective of this repository is to share code solution for Weather Station Phase1 and Phase2. This task was developed to use the microcontroller `STM32L052K6`.

This code purpose is to assess knowledge regarding C language and the microcontroller specifics. If you have any suggestion or feedback, please reachout through [issues](https://github.com/ArthurZuliani/STM32WeatherStation/issues/new) tab.

## Phase 1 - A little more hardware-based

:dart: **Goals** 

- Use of built-in Real Time Clock
- Configuration and use of USB peripheral as Virtual Communication Port (VCP)
- Setup and application of Pulse Width Modulation (PWM) and Analog-Digital Converter (ADC)

:warning: **Important Notes** 

 Due to hardware limitations, the RTC must be configured with the following parameters values in order to have a `1s` update rate[^1].

| Parameter                | Value         | 
| -------------------------|:-------------:| 
| Clock source             | `LSI (37KHz)` | 
| Asynchronous Pre-devider | `124`         | 
| Synchronous Pre-devider  | `333`         | 

:computer: **Remarkable code snippets for this phase**

This section is to share code snippets that might help you during your code analysis.

1. CPU board does not have a hardware to handle USB indexation in Windows. So, a [workaround](https://github.com/ArthurZuliani/STM32WeatherStation/blob/06770d298640cae5f74216863db1b754b39d3986/WS2022_TP16_Phase1_Solution/USB_DEVICE/App/usb_device.c#L68) is to reset `DP` pin before initialize the microcontroller to use USB.

2. An important skill in C is to know how to use variables from other files/libraries through [`extern`](https://github.com/ArthurZuliani/STM32WeatherStation/blob/06770d298640cae5f74216863db1b754b39d3986/WS2022_TP16_Phase1_Solution/USB_DEVICE/App/usbd_cdc_if.c#L53) keyword. It gives freedom to share values in the project, and mainly, to organize your code in multiple files.

3. Sometimes `string` manipulation looks complicated in C codes. However, if you have a good understand of standards libraries, you can use them in your [favor](https://github.com/ArthurZuliani/STM32WeatherStation/blob/06770d298640cae5f74216863db1b754b39d3986/WS2022_TP16_Phase1_Solution/USB_DEVICE/App/usbd_cdc_if.c#L270) to communicate easily through USB.

## Phase 2 - Weather Station using AHT20

:dart: **Goals** 

- Read and comprehension of ready-made library
- Datasheet reading and I2C protocol usage
- `WS2812` LED control
- String manipulation

:warning: **Important Notes** 

 The `WS2812` LEDs have been connected to an I/O pin that does not have TIM_PWM_Channel, which led to a code that required a precise time control[^2]. Due to that restriction, when updating LEDs status, it is necessary to disable interruption, Update LEDs state, and then enable interruptions again. You can find the code snippet that does it, down below:

 ```C
 void ws2812b_set_colors (uint32_t grb)
{
  .
  .
  .

  uint32_t prim = __get_PRIMASK ();
  __disable_irq ();

  //LED state update 
  .
  .
  .

  if (!prim) __enable_irq ();
}
 ```

:computer: **Remarkable code snippets for this phase**

This section is to share code snippets that might help you during your code analysis.

1. Knowing ASCII is essential, or at least understand how to [compare](https://github.com/ArthurZuliani/STM32WeatherStation/blob/06770d298640cae5f74216863db1b754b39d3986/WS2022_TP16_Phase2_Solution/USB_DEVICE/App/usbd_cdc_if.c#L273) values using ASCII characters in C.

2. `Cortex-M0` does not support unaligned `read/write` [operations](https://github.com/ArthurZuliani/STM32WeatherStation/blob/06770d298640cae5f74216863db1b754b39d3986/WS2022_TP16_Phase2_Solution/Core/Src/main.c#L118).

3. Enum are very powerful to handle `state machines` and also to handle selectable options. However, how can you [share](https://github.com/ArthurZuliani/STM32WeatherStation/blob/06770d298640cae5f74216863db1b754b39d3986/WS2022_TP16_Phase2_Solution/USB_DEVICE/App/usbd_cdc_if.c#L52) them among different files?

4. `string` manipulation and special characters can help a lot when multiple reports are necessary. Sometimes it is about copy+paste and adjust formatting (Sometimes [ternary](https://github.com/ArthurZuliani/STM32WeatherStation/blob/06770d298640cae5f74216863db1b754b39d3986/WS2022_TP16_Phase2_Solution/Core/Src/main.c#L343) operators can be useful).

---

[^1]: CPU card does not have external low-frequency crystal (32.768KHz)
[^2]: Here is a good example to show the efficiency of assembly code when compared to C.
