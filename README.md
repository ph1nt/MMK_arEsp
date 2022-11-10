# My Mechanical Keyboard

![IMG_MMK](https://user-images.githubusercontent.com/21249992/201203570-80a2f8f8-d87c-41e3-9a09-08e1af2ed91f.jpg)

----
## Assumptions:
- compact (under 40 keys)
- wireless (bluetooth)
- OLED information display
- encoder knob
- 3D printed case
- handwired
- ESP32 microcontroller
- OTA updates (WiFi)

## Problems:
- no working QMK for ESP32
- no other working firmware for ESP32
- no circuitpython for ESP32
- no good librares for micropython
- no spare time for coding..

## Solutions:
- write own firmware for ESP32 keyboard
- use as many as possible existing code
- use Arduino style code
- use PlatformIO and Visual Studio Code
- write all under open licence

## This project include:
	NimBLE-Arduino
	olikraus/U8g2@^2.34.3
	mprograms/SimpleRotary@^1.1.3
	fbiego/ESP32Time@^2.0.0
	
and wildly modified:
    t-vk/ESP32 BLE Keyboard@^0.3.2

action_code.h & keycode.h & quantum_keycodes.h from [QMK](https://github.com/qmk/qmk_firmware)
    
##Build number generator for C and PlatformIO:
https://gitlab.com/pvojnisek/buildnumber-for-platformio/tree/master
