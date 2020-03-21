# STM32_UART_MIDI
A direct port of the MIDI arduino library to the STM32 system

This implements the MidiInterface class as you know it from the arduino ecosystem and should work exactly the same way. 

# How to use this Class with your STM32F0 project
The port is intended for use with HAL drivers. For STM32Cube IDE:
- Copy the MIDI.h file to the Core/Inc folder
- Copy the MIDI.cpp file to the Core/Src folder
- Configure your project to use cpp. It seems that for the compiler to use C++ for your main file, it needs to be renamed to main.cpp . Remember to change it back to main.c before generating hardware configuration files using the CubeMX tool
- Add #include "MIDI.h" in your main.cpp
- Create a MidiInterface object outside the main() function (ie declare as a global variable)
- Before while(1) and after all hardware initialization has taken place, call myInterface.begin(Channel, serial_in, serial_out). Directly below set the callback handlers used, if any. (Refer to the arduino documentation for Callback usage)
- Run myInterface.read() in the while(1) loop. Try to minimise the time between calls to avoid missing messages.


The USART needs to be configured as per MIDI specification:
- Asynchronous
- Baud Rate: 31250
- 8 bit messages
- 1 stop bit

# Using this Class with other STM32 families
I have only verified functionality on the STM32F072 microcontroller. However, if there are HAL drivers for the board you are using, changing #include "stm32f0xx_hal.h" should suffice.
If HAL drivers are not available, you will have to manually change the HAL_UART_ commands to those appropriate for your device. No other changes should be needed.

# Example Usage
The MIDIlib folder contains a project for STM32Cube IDE configured for the 32f072B discovery board. It is configured to use callbacks and flash an LED in sync with an incoming MIDI clock. All the callback functions are in the MIDIHandlers.h and MIDIHandlers.cpp files. 
To use this project with a different board:
- Create a new project and select your board in the configuration wizard. Set language to C++
- Configure the appropriate USART peripheral of your board with the settings described above
- Generate code, rename main.c to main.cpp
- Add the class initialization and callbacks in your main.cpp (refer to the guide above and the original main.cpp file)
- Add the MIDIHandler .cpp and .h files in the appropriate Core/Inc and Core/Src folders and add #include "MIDIHandler.h" and #include "MIDI.h" in main.cpp
- If using a family other than stm32f0 remember to change #include "stm32f0xx_hal.h" to the appropriate one for your model
