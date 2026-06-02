/*
 * Copyright 2016-2026 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "MKL46Z4.h"

#define CircleTiming 9000000 // how long one circle takes (by time)


/* ============================================================
   TRIM TUNING VALUES (YOU ADJUST THESE)
   ============================================================ */

// This delay runs every time the while(1) loop repeats.
// Bigger = loop runs slower (robot decisions happen slower).
#define  LOOP_DELAY_TICKS     2000u

// How often we do a trim action.
// Here: after 60 loop cycles, we do a trim (stop right motor briefly).
// Smaller number = trim happens more often.
#define  TRIM_PERIOD_TICKS    60u

// How long we pause the motor during a trim.
// Bigger number = pause lasts longer (stronger correction).
#define  TRIM_OFF_TICKS       60000u


/* ============================================================
   MOTOR HELPER FUNCTIONS
   These functions just set GPIO pins to control motor direction.
   ============================================================ */

/* Left motor moves forward */
static inline void Left_Forward(void) {
    // AI1 and AI2 are the two control pins for left motor driver channel.
    // For this driver: AI1=0 and AI2=1 means "forward".
    GPIOB->PCOR = (1u << 1);   // Clear PTB1 -> AI1 = 0
    GPIOB->PSOR = (1u << 0);   // Set   PTB0 -> AI2 = 1

}

/* Left motor moves backward */
// PORTB controls Motor A (left wheel) through the motor driver.
 //  pins :      Name        Purpose
//    PTB1       AI1          Leftmotor
static inline void Left_Reverse(void) {
    // AI1=1 and AI2=0 means "reverse".
    GPIOB->PSOR = (1u << 1);   // Set   PTB1 -> AI1 = 1
    GPIOB->PCOR = (1u << 0);   // Clear PTB0 -> AI2 = 0

}

/* Right motor moves forward */
static inline void Right_Forward(void) {
    // BI1 and BI2 are the right motor driver pins.
    // BI1=0 and BI2=1 means "forward".
    GPIOC->PCOR = (1u << 1);   // Clear PTC1 -> BI1 = 0
    GPIOC->PSOR = (1u << 2);   // Set   PTC2 -> BI2 = 1

 }

/* Right motor moves backward */
static inline void Right_Reverse(void) {
    // BI1=1 and BI2=0 means "reverse".
    GPIOC->PSOR = (1u << 1);   // Set   PTC1 -> BI1 = 1
    GPIOC->PCOR = (1u << 2);   // Clear PTC2 -> BI2 = 0

 }

/* STOP motors (IMPORTANT for trim) */
static inline void Left_Stop(void) {
    // Stop means both pins are 0 (no direction command).
    GPIOB->PCOR = (1u << 0);   // Clear PTB0 -> AI2 = 0
    GPIOB->PCOR = (1u << 1);   // Clear PTB1 -> AI1 = 0
}

static inline void Right_Stop (void) {
    // Stop right motor: BI1=0 and BI2=0
    GPIOC->PCOR = (1u << 1);   // Clear PTC1 -> BI1 = 0
    GPIOC->PCOR = (1u << 2);   // Clear PTC2 -> BI2 = 0
}

/* Simple busy-wait delay */
static inline void Delay(volatile unsigned int t) {
    // This is a “do nothing” loop.
    // It just burns CPU cycles so time passes.
    while (t--) {
        __asm volatile ("nop");  // nop = "no operation" (wastes 1 CPU cycle)
    }
  }

/* ============================================================
   PWM SPEED CONTROL HELPERS (SIMPLE VERSION)
   ============================================================ */

/*
 * Set LEFT motor speed using PWM
 * speed range: 0 .. MOD
 * 0   = motor OFF
 * MOD = full speed
 */
 static inline void LeftMotorSpeed (int speed){
	 TPM2->CONTROLS[0].CnV = speed; // left motor speed
	 }

/*
 * Set RIGHT motor speed using PWM
 * speed range: 0 .. MOD
 * 0   = motor OFF
 * MOD = full speed
 */
 static inline void RightMotorSpeed(int speed) {
	 TPM2->CONTROLS[1].CnV = speed; // right motor speed
  }

 /*
  * Stop BOTH motors completely
  * - direction pins LOW
  * - PWM speed = 0
  */
 static inline void StopBothMotors(void) {

	 // Set left Motor pins (AI1/AI2) and right motor (BI1/BI2) to LOW( STOP !!)
     Left_Stop();
     Right_Stop();

     LeftMotorSpeed(0);  // set left motor to zero for a full stop
     RightMotorSpeed(0); // set right motor to zero for a full stop
   }

  // This is a function that sets up TPM2 so it can generate PWM.
    static void TPM2GenPWM(void) {

     SIM->SCGC6    |= (1 << 26);   // Enable clock for TPM2 (TPM2 must be clocked to work)
     // added the bottom
     SIM->SCGC5 |= (1u << 10);      // Enable PORTB clock (for PTB2/PTB3 mux)




    // 3) Pin mux: turn PTB2 and PTB3 to becomes TPM PWM outputs (GPIO)
      PORTB->PCR[2] &= ~0x700; // Force only bits 10–8 (MUX) of PTB2 to 0, keep other bits unchanged
      PORTB->PCR[3] &= ~0x700; // Force only bits 10–8 (MUX) of PTB3 to 0, keep other bits unchanged

     //   Set MUX to TPM function (ALT = 3 on pin table)
     //   Example ONLY — replace ALT_VALUE with the correct one from the KL46Z table
     //  •<< 8 puts that value into MUX bits [10:8]
     //	 •|= sets those bits without touching any others
     //  Pin PTB2 & PTB3 is being connected to the TPM hardware so it outputs a PWM signal.
      PORTB->PCR[2] |= (0X300);    // PTB2 -> TPM2_CH0 PWM
      PORTB->PCR[3] |= (0X300);     // PTB3 -> TPM2_CH1 PWM


      // Select TPM clock source = OSCERCLK (8 MHz crystal)
     // SIM_SOPT2[25:24] = 10
      SIM -> SOPT2 &= ~(3 << 24);   // clears both bits 25 and 24 bits before setting them
      SIM -> SOPT2 |=  (2 << 24);   // Set TPMSRC = OSCERCLK

// added 3 line to test
      // 2) stop TPM2 before config


       // SC = is like ON/OFF switch + settings knob for the timer.
      TPM2  ->  SC  |=  (1 << 3); // SC Turns TPM2 OFF(Stopping the internal counter) because we much stop the timer before changing its settings
      TPM2  ->  MOD  =     7999;   // PWM period (frequency setting). choosed 62 to control PWM frequency

// PAGE 569
      /* Set the PWM mode for each channel
      // CONTROLS[] ==  hardware-defined from MKL46Z4.h
       Ex:
     //  TPM2
      // ├── CONTROLS[0]   ← channel 0
      // ├── CONTROLS[1]   ← channel 1
      // ├── CONTROLS[2]
      // └── ...
       CnSC: tells the channel how to behave:Ex: active-high or active-low
       (1 << 5) | (1 << 3); 	turn ON bit 5 and bit 3 and the rest off
       */
      // EACH LEFT SHIFT 2 = REPRESENT BOTH REGISTER
      TPM2->CONTROLS[0].CnSC = (2 << 4) | (2 << 2); // CONTROLS[0] means: TPM2 channel 0”
      TPM2->CONTROLS[1].CnSC = (2<<  4) | (2 << 2); // CONTROLS[0] means: TPM2 channel 1”

      // Initial duty cycles (speed)
         TPM2->CONTROLS[0].CnV = 5600;   // left starts at some speed
         TPM2->CONTROLS[1].CnV = 7200;   // right starts at some speed





         // ISSUE
         // Start TPM2: PS=1 and CMOD=1
           // replaced: TPM2->SC |= (1 << 3); // shey: keep this
         // to:
         //TPM2->SC = (1u << 3);  //chat  CMOD=01 (run), PS=000 (/1)
     }


/* ============================================================
   MAIN:
   ============================================================ */

int main (void) {

    /* These are generated by MCUXpresso to set up the board */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

 #ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
 #endif

    /* Print to the debug console to prove the program runs */
    PRINTF("Hello World\r\n");

    /* ========================================================
       LED SETUP (PTD5)
       We'll use PTD5 as an output pin for the LED
       ======================================================== */

    SIM->SCGC5    |=  (1u << 12);       // Turn ON clock for PORTD (needed before using PTD pins)
    PORTD->PCR[5] &=   ~0x700;       // Clear MUX bits for PTD5
    PORTD->PCR[5] |= (1u << 8);     // Set PTD5 MUX = GPIO
    GPIOD->PDDR   |= (1u << 5);       // Set PTD5 direction = OUTPUT

    /* ========================================================
       SWITCH SETUP (PORTC)
       SW1 and SW3 are active-low buttons.
       Active-low means:
         - not pressed = 1 (HIGH)
         - pressed     = 0 (LOW)
       ======================================================== */

    SIM->SCGC5 |= (1 << 11);       // Turn ON clock for PORTC


    // SW1 = PTC3
    PORTC->PCR[3]   &= ~0x700;     // Clear MUX
    PORTC->PCR[3] |= (1 << 8);     // GPIO mode
    PORTC->PCR[3] |= (1 << 1);     // Enable pull resistor
    PORTC->PCR[3] |= (1 << 0);     // Choose pull-up (so default is HIGH)
    GPIOC->PDDR   &= ~(1 << 3);      // Set PTC3 direction = INPUT

    // SW3 = PTC12
    PORTC->PCR[12] &=  ~0x700u;
    PORTC->PCR[12] |= (1 << 8);
    PORTC->PCR[12] |= (1 << 1);
    PORTC->PCR[12] |= (1 << 0);
    GPIOC->PDDR &=   ~(1 << 12);     // INPUT

    /* ========================================================
       MOTOR GPIO SETUP
       Configure motor control pins as GPIO outputs.
       ======================================================== */

    SIM->SCGC5 |= (1 << 10);       // Turn ON clock for PORTB
    SIM->SCGC6 |= (1 << 26);       // TPM2 clock gate (TPM0/1/2 live in SCGC6)


    // ------------------------------------------------------------
    // Configure motor control pins to act as GPIO outputs
    // ------------------------------------------------------------
    // Each pin on the KL46Z can do many different jobs (UART, SPI,
    // PWM, GPIO, etc.). This is called "pin multiplexing".
    //
    // The PCR (Pin Control Register) selects WHAT FUNCTION the pin uses.
    //
    // For the KL46Z:
    //   - PCR MUX bits are bits [10:8]
    //   - MUX = 001 (binary) means GPIO mode
    //
    // Steps we do for EACH motor pin:
    //   1) Clear the old MUX setting (wipe bits [10:8])
    //   2) Set MUX = 001 → GPIO mode
    // ------------------------------------------------------------

    // PTB1 → Left motor control pin AI1
    // Clear bits 10:8 (old function), then set MUX = GPIO
    PORTB->PCR[1] &=    ~0x700;     // Clear MUX bits [10:8]
    PORTB->PCR[1] |= (1u << 8);  // Set MUX=001 → GPIO mode (AI1)

    // PTB0 → Left motor control pin AI2
    PORTB->PCR[0] &=    ~0x700;     // Clear MUX bits [10:8]
    PORTB->PCR[0] |= (1u << 8);  // Set MUX=001 → GPIO mode (AI2)

    // PTC1 → Right motor control pin BI1
    PORTC->PCR[1] &=   ~0x700;     // Clear MUX bits [10:8]
    PORTC->PCR[1] |= (1 << 8);  // Set MUX=001 → GPIO mode (BI1)

    // PTC2 → Right motor control pin BI2
    PORTC->PCR[2] &=   ~0x700;     // Clear MUX bits [10:8]
    PORTC->PCR[2] |= (1u << 8);  // Set MUX=001 → GPIO mode (BI2)

    // Set motor pins direction = OUTPUT
    GPIOB->PDDR |= (1 << 1) | (1 << 0);
    GPIOC->PDDR |= (1 << 1) | (1 << 2);
    GPIOB->PDDR |= (1 << 2) | (1 << 3);   // PTB2=TPM2_CH0 (PWMA), PTB3=TPM2_CH1 (PWMB)

     /* ========================================================
        IDLE state: motors stopped at power-up
        ======================================================== */


      /* ========================================================
         Enable PWM generation for motors
         ======================================================== */
    // Make sure motors are stopped first
    TPM2GenPWM();
    StopBothMotors();
    Delay(500000);



    /* i is just a counter (not really used for logic now, but fine to keep) */
     volatile unsigned int i = 0;

    /* These store the previous button state so we can detect “new press” */
     static int sw1_last = 0;
     static int sw3_last = 0;

    /* This counts how many loops have happened since the last trim */
      static unsigned int trimTick = 0;



      // make it drive forward.
             Left_Forward();
             Right_Forward();
             LeftMotorSpeed(5600);
             RightMotorSpeed(7200);

    /* ========================================================
       MAIN LOOP (RUNS FOREVER)
       ======================================================== */

    while (1) {

        i++;

        // Slow down the loop so behavior is stable
        Delay(LOOP_DELAY_TICKS);

        /* -------- READ BUTTON PINS -------- */

        // Read raw pin state (bit is either 0 or nonzero)
        int sw1_pin = GPIOC->PDIR & (1u << 3);      // SW1 raw read
        int sw3_pin = GPIOC->PDIR & (1u << 12);     // SW3 raw read

        // Convert active-low to pressed=1 / not-pressed=0
        int sw1_pressed_now = (sw1_pin == 0) ? 1 : 0;
        int sw3_pressed_now = (sw3_pin == 0) ? 1 : 0;

        /* DEBUG LED: ON when SW1 pressed */
        // NOTE: On FRDM-KL46Z, PTD5 LED is active-low:
        //   PCOR makes it ON, PSOR makes it OFF.
        if (sw1_pressed_now) {
            GPIOD->PCOR = (1 << 5);   // LED ON
        } else {
            GPIOD->PSOR = (1 << 5);   // LED OFF
        }




        /* -------- EDGE DETECTION -------- */
        // press_event becomes 1 only at the moment you FIRST press the button.
        // It will NOT stay 1 while holding.
        int sw1_press_event = (sw1_pressed_now == 1 && sw1_last == 0) ? 1 : 0;
        int sw3_press_event = (sw3_pressed_now == 1 && sw3_last == 0) ? 1 : 0;

        // Save current states for next loop
        sw1_last = sw1_pressed_now;
        sw3_last = sw3_pressed_now;

        /* turning flag prevents trim while we are doing a turn */
        int turning = 0;


        /* -------- LEFT TURN (SW1) -------- */
        if (sw1_press_event) {
            turning = 1;

            // debounce: wait a bit so one press doesn't bounce into multiple presses
            Delay(80000);

            // Turning left: left wheel reverse + right wheel forward (spin)
            Left_Reverse();
            Right_Forward();

            // Hold that turn for a fixed time
            Delay(1700000);

            // Go back to forward
            Left_Forward();
            Right_Forward();

            // Reset trim counter so trim doesn't trigger right away after turning
            trimTick = 0;
        }

        /* -------- RIGHT TURN (SW3) -------- */
        if (sw3_press_event) {
            turning = 1;
            Delay(80000);

            // Turning right: left wheel forward + right wheel reverse
            Left_Forward();
            Right_Reverse();

            Delay(1700000);

           // Left_Forward();
           // Right_Forward();

            trimTick = 0;
        }

        /* -------- TRIM (ONLY WHEN NOT TURNING) --------
           robot drifts LEFT.
           So we are trying to fix it by slowing the RIGHT motor sometimes.
           Idea:we briefly stop RIGHT motor so the robot nudges back toward center.
        */
        if (!turning && !sw1_press_event && !sw3_press_event) {
            // not turning  AND no new left turn press && no new right turn press
            trimTick++;  // count loops

            // When we hit the trim period, do one trim correction
            if (trimTick >= TRIM_PERIOD_TICKS) {
                trimTick = 0;

                // Stop right motor briefly
                Right_Stop();

                // Hold it stopped (this is the strength of correction)
                Delay(TRIM_OFF_TICKS);

                // Resume right motor forward
                Right_Forward();
            }
        }
    }

}
