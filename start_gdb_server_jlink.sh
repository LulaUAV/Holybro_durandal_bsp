#!/bin/sh

#JLinkGDBServer -device nrf52 -speed 1200 -if swd  -port 2331
#JLinkGDBServer -device stm32f303cc -speed 1200 -if swd  -port 2331
#JLinkGDBServer -device stm32f401cb -speed 1200 -if swd  -port 2331
#JLinkGDBServer -device stm32f427vi -speed 1200 -if swd  -port 2331
JLinkGDBServer -device stm32h743zi -speed 2400 -if swd  -port 2331

