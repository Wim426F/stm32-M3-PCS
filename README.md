# Tesla Model 3 PCS Controller Firmware
02/09/22: V1 of new firmware for STM32 based Tesla Model 3 PCS Controller.
30/12/25: Complete rewrite of Damien's work by Wim Boone. PCS controller is now slave to the Zombieverter VCU and compatible as "Tesla Charger"

# Compiling
You will need the arm-none-eabi toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads

The only external depedencies are libopencm3 and libopeninv. You can download and build these dependencies by typing

`make get-deps`

Now you can compile stm32_pcs by typing

`make all`

And upload it to your board using a JTAG/SWD adapter by typing

`make flash`

Or use Openinverter CAN tool to update firmware via CAN-bus


