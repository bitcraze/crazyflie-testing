



Contents
========

* [Bootloaders](#bootloaders)
	* [reset_to_bootloader](#reset_to_bootloader)
	* [reset_to_firmware](#reset_to_firmware)
	* [reliability](#reliability)


<!-- This file is auto-generated from: bootloaders.toml -->


# Bootloaders


These requirements targets the functionality of the Crazyflie bootloaders.
## reset_to_bootloader


It must be possible, by way of radio, to reset to bootloader from main firmware.

|Field|Value|
| :--- | :--- |
|rational|Design|

## reset_to_firmware


It must be possible, by way of radio, to reset to main firmware from bootloader.

|Field|Value|
| :--- | :--- |
|rational|Design|

## reliability


Switching between bootloader and main firmware
|Field|Value|
| :--- | :--- |
|background|The design is that there should never be any timing dependencies in the python API So we should be able to go back and forth between bootloader and main firmware for ever. However, testing infinity is hard so, as a compromise, we are testing with 20 iterations per device. |
|iterations|20|
|rational|Design|
|timeout_per_iteration|12|
