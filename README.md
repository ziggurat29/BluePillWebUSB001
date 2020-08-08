# BluePillWebUSB001
A baseline project for the BluePill (STM32F103) implementing a WebUSB endpoint

This is an experiment in several things:

1) implement a WebUSB endpoint on the device.  This is the primary objective.
1) avoiding using the HAL as much as possible, and instead relying on the LL libraries.  This is an attempt to reduce bloat that CubeMX emits via the HAL.  This is the secondary objective.
1) use the TinyUSB library.  This sevices the prior two objectives because it suports the WebUSB device class, and also obviates the need for the HAL (which is the only method of getting USB support out-of-box with CubeMX).  We'll see how this works out.  Hopefully well!

In addition to those objectives, be aware that I have also included my usual assortment of infrastructure components, which include:

* FreeRTOS is used -- obviously not my creation, but I use it to realize some of the following stuff.
* a serial I/O stream abstraction -- I use this to abstract UART and USB CDC APIs, and make other components 'bind' to the abstract interface rather than invoke the technology-specific APIs.  The design is vaguely like a non-blocking socket.  The implementation provides data buffering.
* a replacement heap implementation -- I prefer this because it give me more visibility and control into heap usage than the out-of-box libc malloc().  'Library interpositioning' is used to redirect malloc() calls in other code (even pre-compiled; e.g. internally in libc's implementation itself) into this heap.  There's also some debug features such as fill patterns and heapwalks.
* a 'monitor' task.  This is a rudimentary CLI via a stream, and provides some ability to control the board via serial port.
* a rudimentary lamp manager to allow you to, say, 'turn on the lamp for 1 second' and not have to concern yourself with turning it off after the time has elapsed.  (it's quite rudimentary; I haven't implemented 'blink patterns', etc)
* sundry minor utilities and routines such as circular buffer and some simplified implementations of things in libc that save a bunch of flash if you don't need the full monty implementation.

Note:  if you use CubeMX, it is not possible to completely get off the HAL -- the internal implementation of the framework uses only the HAL in some cases (e.g. the systick into the FreeRTOS).  As such, this project should be considered 'HAL-light' more than 'HAL-less'.
