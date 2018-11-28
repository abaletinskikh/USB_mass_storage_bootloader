# OVERVIEW

This is a mass storage bootloader for the STM32F103 chip board (Blue Pill).
It appears as a regular external drive (formatted with FAT12) when plugged into a PC,
 no drivers or custom software needed!

It takes up 18kB of flash memory. 
Bootloader is entered when USB plugged in PC and wait enumeration.
If enumeration is not started, Bootloader will run the user code.

*REMEMBER TO EDIT YOUR PROGRAM'S LINKER SCRIPT TO START THE CODE AT ADDRESS 0x4800*

This software is still experimental, please report any issues!


# INSTALLATION:

* Run make
* Flash build/USBMassStorage.bin onto your Blue Pill board
For example,
stm32flash -v -w build/USBMassStorage.bin /dev/ttyUSB0


# HOW TO USE:

* Edit the linker script of the firmware you want to flash using the bootloader, so that the code starts at address 0x4800.
 usually it means you have to find this in projectname.ld:

    FLASH (rx) : ORIGIN = **0x00000000**, LENGTH = 64k
    
     and change it to:

	FLASH (rx) : ORIGIN = **0x00004800**, LENGTH = 46k

    then rebuild your project
    ( for CubeMX projects also need to change VECT_TAB_OFFSET )

* Plug in your board , the system should recognize it as a 212 kB mass storage device. The 110 kB space will be used by the *FIRMWARE.BIN* file. This file contains the user code. 
The red LED (PC13) will blink when the bootloader is running.
* You can download the FIRMWARE.BIN found on the drive to download the contents of flash memory
* You can upload your firmware to the board by copying your firmware to the device (the first file you put on the device will be considered new firmware)
* Safely eject the drive, power your board, and after some time it should jump to your code


