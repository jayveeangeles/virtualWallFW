# virtualWallFW

## Project Brief:
Repository to hold firmware of Roomba Virtual Wall using Microchip PIC12F683

Inspired by various virtual wall implementations for Roomba, this project combines ideas from those implementations with some additional features such as battery monitoring and indicator and low power RTC to maintain accurate timing even when PIC is in sleep mode. 

## Features:
1) Sleep mode after 10 minutes to conserve battery.
2) Sleep mode in betwwen pulse train (approx. 1s) to conserve battery; use TMR1 interrupt to wake up from sleep
3) Use 32.768 kHz crystal for sleep mode clock
4) Use native PWM module of PIC12F683 to generate train pulse
5) Use ADC to monitor coin battery voltage; hardware uses CAT3200 to charge pump voltage from 3.3V to 5V
6) Added visual indicator (red LED) to pulse together with IR to indicate low battery

## Repo Files:
Only the main C file, Makefile and dist directories are included. The dist directory contains the latest hex file that can be programmed into the PIC.

## Hardware:
Link to hardware (schematic and board files) will be posted soon...

## References:
1) [Tiny Remote for iRobot Roomba](http://www.enide.net/webcms/index.php?page=tiny-remote)
2) [Small Virtual Wall for iRobot Roomba](http://www.enide.net/webcms/index.php?page=virtual-wall-for-roomba)
3) [Roomba 620 infrared signals](http://astrobeano.blogspot.com/2013/10/roomba-620-infrared-signals.html)
4) [Raspberry Pi IR blaster and Roomba IR codes](http://astrobeano.blogspot.com/2013/11/raspberry-pi-ir-blaster-and-roomba-ir.html)
5) [DIY Virtual Wall for Roomba – Part Two](http://misc.ws/2014/08/09/diy-virtual-wall-for-roomba-part-two/)
6) [An AVR program for ATTiny85 to transmit a virtual wall signal for an iRobot Roomba](https://github.com/Petezah/roomba_wall_v2)
