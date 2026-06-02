TPM-PWM-Circular-Robot-Control-KL46Z

Embedded systems project developed on the FRDM-KL46Z microcontroller platform using low-level C programming and direct register manipulation.

Project Overview

This project implements real-time motor speed and direction control for a two-wheel robot using TPM2 hardware PWM and GPIO-based motor driver interfacing. The robot traces circular motion by applying different PWM duty cycles to the left and right wheels.

Features

* TPM2 hardware PWM generation
* GPIO-based DC motor direction control
* Circular motion control using differential wheel speeds
* Switch-triggered directional behavior using SW1 and SW3
* Real-time embedded control using ARM Cortex-M microcontroller
* Register-level peripheral configuration
* Motor helper abstraction functions

Hardware Platform

* FRDM-KL46Z Development Board
* Dual DC Motors
* Motor Driver Module
* TPM2 PWM Channels
* Push Button Inputs (SW1 / SW3)

Technologies Used

* Embedded C
* ARM Cortex-M0+
* TPM (Timer/PWM Module)
* GPIO Peripheral Control
* MCUXpresso IDE

Learning Outcomes

This project demonstrates practical experience with:

* PWM signal generation
* Embedded motor control
* Timer configuration
* Real-time hardware interfacing
* Register-level programming
* Differential drive robotics
