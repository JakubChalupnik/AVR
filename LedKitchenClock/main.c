/******************************************************************************
  Copyright (C) 2012 Jakub Chalupnik <jakubchalupnik@gmail.com>.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
******************************************************************************/

//*******************************************************************************
//*
//* ATmega8 based kitchen clock / timer, using four 7seg LED digits 
//* controlled through TLC5917.
//*
//*******************************************************************************
//* FileName:   main.c
//* Processor:  ATmega8
//*
//* Author      Date      Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik       3.6.2012  First release, HW functional (CPU + display)
//*
//*******************************************************************************

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
//
// See led_seg.h for details about pin definitions
//

//*******************************************************************************
//*                           Includes and defines                              *
//*******************************************************************************

#include "defs.h"
#include "hw.h"
#include "global.h"

#include <avr/io.h>                     // include I/O definitions (port names, pin names, etc)
#include <avr/signal.h>                 // include "signal" names (interrupt names)
#include <avr/interrupt.h>              // include interrupt support
#include <avr/pgmspace.h>
#include <util/delay_basic.h>
#include <util/delay.h>
#include <string.h>

#include "led_seg.h"

#define nop() {__asm__("nop");}         // nop is cca 50ns

//*******************************************************************************
//*                               HW dependant macros                           *
//*******************************************************************************


//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************

const byte Ciel8[] PROGMEM = {
    0, 1, 2, 4, 8, 13, 21, 31
};
#define Ciel8Value(Index) ((Index >= 8) ? 31 : pgm_read_byte (((PGM_P) Ciel8) + Index))

byte LedScreen [4];
byte Brightness;

//*******************************************************************************
//*                              Interrupt handler                              *
//*******************************************************************************

//
// This interrupt is activated when timer 1 matches OCR0A.
//

SIGNAL(TIMER1_COMPA_vect) {
    static byte LedDigitActive = 0;
    static byte BrightnessCounter = 0;

    //
    // If BrightnessCounter is 31, it's time to move to another digit
    //

    if (BrightnessCounter == 31) {
        BrightnessCounter = 0;

        //
        // Disable all outputs, shift the new digit value into the LED driver
        //

        LedBitSet (OE);

        LedShiftByte (LedScreen [LedDigitActive]);

        //
        // Enable corresponding digit
        //

        if (LedDigitActive == 0) {
            LedBitSet (D1);
        } else {
            LedBitClear (D1);
        }

        if (LedDigitActive == 1) {
            LedBitSet (D2);
        } else {
            LedBitClear (D2);
        }

        if (LedDigitActive == 2) {
            LedBitSet (D3);
        } else {
            LedBitClear (D3);
        }

        if (LedDigitActive == 3) {
            LedBitSet (D4);
        } else {
            LedBitClear (D4);
        }

        //
        // Enable LED driver, but only if brightness is not set to 0
        // (if it's 0, we shall not enable the output drivers at all)
        //

        if (Brightness > 0 ) {
            LedBitClear (OE);
        }

        //
        // Increment active digit number
        //

        if (LedDigitActive >= 3) {
            LedDigitActive = 0;
        } else {
            LedDigitActive++;
        }

    //
    // If BrightnessCounter is bigger than requested brightness, disable LED drivers
    //

    } else if ((BrightnessCounter > Brightness) || (Brightness == 0)) {
        LedBitSet (OE);
        BrightnessCounter++;

    //
    // Otherwise (BC <= B) enable LED drivers
    //

    } else {
        LedBitClear (OE);
        BrightnessCounter++;
    }
}

//*******************************************************************************
//*                                LED support                                  *
//*******************************************************************************

//
// Sends one byte to the LED driver
//

void LedShiftByte (byte Byte) {
    byte i;

    for (i = 0; i < 8; i++) {
        if (Byte & 0x01) {
            LedBitSet (SDI);
        } else {
            LedBitClear (SDI);
        }

        LedBitSet (CLK);
        Byte >>= 1;
        LedBitClear (CLK);
    }

    LedBitSet (LE);
    LedBitClear (LE);
}

//
// Displays one two-digit decimal number on the left, the second on the right
//

void LedDisplayTime (byte Left, byte Right) {

    LedScreen [0] = seg_hex_table [Left / 10];
    LedScreen [1] = seg_hex_table [Left % 10];
    LedScreen [2] = seg_hex_table [Right / 10];
    LedScreen [3] = seg_hex_table [Right % 10];
}

//
// Displays / hides the dot between left and right side of the display
//

void LedDisplayDot (byte Enable) {

    if (Enable) {
        LedScreen [1] |= 1 << _P_SEG;
    } else {
        LedScreen [1] &= ~(1 << _P_SEG);
    }
}

//
// Displays / hides the colon between left and right side of the display
//

void LedDisplayColon (byte Enable) {

    if (Enable) {
        LedScreen [2] |= 1 << _P_SEG;
    } else {
        LedScreen [2] &= ~(1 << _P_SEG);
    }
}

//*******************************************************************************
//*                                 HW init                                     *
//*******************************************************************************

void HwInit(void) {

    //
    // Initialize ports, set directions.
    //

    LedBitClear (CLK);
    LedBitSet (OE);
    LedBitClear (LE);

    LedBitOutput (SDI);
    LedBitOutput (CLK);
    LedBitOutput (LE);
    LedBitOutput (OE);

    LedBitClear (D1);
    LedBitClear (D2);
    LedBitClear (D3);
    LedBitClear (D4);

    LedBitOutput (D1);
    LedBitOutput (D2);
    LedBitOutput (D3);
    LedBitOutput (D4);

    //
    // Initialize timer to tick every 0.5ms
    //

    TCCR1A = 0x00;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);  // Prescaler 1/64
    OCR1AH = 0x00;
    OCR1AL = 16;                        // Timer1 output compare A - triggers "Output Compare A" at rate of cca 2kHz
    TIMSK |= _BV(OCIE1A);               // Enable Timer 1 Output Compare A interrupt
    TIFR = _BV(OCF1A);                  // Clear any pending interrupts
}

//*******************************************************************************
//*                                     MAIN                                    *
//*******************************************************************************

int main(void) {
    int i;

    //
    // Initialize all global variables
    //

    Brightness = Ciel8Value (1);       // Medium brightness
    memset (LedScreen, 0, sizeof (LedScreen));

    //
    // Initialize all the hardware, that is pins, I2C, RTC
    //

    HwInit();
    sei();                              // Enable interrupts

    i = 0;
    while (1) {
//         LedScreen [0] = seg_hex_table [(i % 10000) / 1000];
//         LedScreen [1] = seg_hex_table [(i % 1000) / 100];
//         LedScreen [2] = seg_hex_table [(i % 100) / 10];
//         LedScreen [3] = seg_hex_table [i % 10];
//         LedScreen [0] = seg_hex_table [0];
//         LedScreen [1] = seg_hex_table [1];
//         LedScreen [2] = seg_hex_table [2];
//         LedScreen [3] = seg_hex_table [3];

        LedDisplayTime (i / 100, i % 100);
        LedDisplayColon (1);
        _delay_ms(100);
        i++;
    }



//     LedBitSet (D4);
//     LedBitClear (OE);
//     for (i = 0; i < 8; i++) {
//         LedShiftByte (1 << i);
//         _delay_ms(2000);
//     }
//
//     for (i = 0; i < 8; i++) {
//         LedShiftByte (seg_hex_table [i]);
//         _delay_ms(2000);
//     }

    //
    // Main loop just repeatedly does nothing at all
    //

    while(1) {
    }
}


