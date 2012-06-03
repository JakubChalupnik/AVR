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
//* Kubik       3.6.2012  Added RTC and keyboard handling
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
#include "i2cmaster.h"

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

byte Hour, Min, Sec;
byte Day, Month;
byte Dow;
byte AmPm;

byte key = 0x00;

//*******************************************************************************
//*                              Function headers                               *
//*******************************************************************************

void LedShiftByte (byte Byte);
void KeyIntHandler (void);

//*******************************************************************************
//*                              Interrupt handler                              *
//*******************************************************************************

//
// This interrupt is activated when timer 2 matches OCR2.
//

SIGNAL(TIMER2_COMP_vect) {
    static byte LedDigitActive = 0;
    static byte BrightnessCounter = 0;

    //
    // If BrightnessCounter is 31, it's time to move to another digit and scan the keys
    //

    if (BrightnessCounter == 31) {

        BrightnessCounter = 0;

        //
        // Before doing anything else, process keys
        //

        KeyIntHandler ();

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
// Uses BCD encoding!
//

void LedDisplayTime (byte Left, byte Right) {

    LedScreen [0] = seg_hex_table [Left >> 4];
    LedScreen [1] = seg_hex_table [Left & 0x0F];
    LedScreen [2] = seg_hex_table [Right >> 4];
    LedScreen [3] = seg_hex_table [Right & 0x0F];
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
//*                                  Key processing                             *
//*******************************************************************************

byte KeyRead (void) {
    byte Keys = 0;

    if (Key1Pressed ()) {
        Keys |= KEY_1;
    }

    if (Key2Pressed ()) {
        Keys |= KEY_2;
    }

    return Keys;
}

void KeyIntHandler (void) {
    static uint8_t id = 0x00;
    static uint8_t key_counter = 0;
    uint8_t c;

// Process keyboard - based on algorithm by guy KimmoHop from avrfreaks.org:
// In key detection, there is a counter, and identification of the pressed key. If no key is pressed, mark
// identification as "no key" and counter to 0. When a key is pressed, counter is increased, and identification
// is set to the key pressed. If the key changes, counter is set back to 0 and id updated.
// When the counter reaches first mark value (A, debounce delay), key event is sent. If the key is yet pressed,
// counter keeps increasing.
// When the counter reaches second mark value (B, repeat start delay), new key event is sent, and third value (C,
// repeat delay) is subtracted from counter.
// So, A/interrupt period is the debounce time, (B-A)/interrupt period is the time before repeat starts, and C/
// interrupt period is the time between repeats.
// The counter goes 0, 1,..,A-1, A, A+1,...,B-1, B, B-C, B-C+1,...,B-1, B and key event is sent at every A and B.

    c = KeyRead();
    if(c == 0) {                    // No key pressed
        id = 0;
        key_counter = 0;
    } else if(c != id) {            // New key differs from the previous one
        id = c;
        key_counter = 0;
    } else {                        // New key is the same as previous one
        key_counter++;
    }
    if(key_counter == DELAY_DEBOUNCE) {     // Debouncing complete - set key pressed
        key = id;
    } else if(key_counter == DELAY_REPEAT_START) {  // Repeated key
        key = id;
        key_counter -= DELAY_REPEAT;
    }
}

//*******************************************************************************
//*                                  RTC related                                *
//*******************************************************************************

void RtcUpdateTime(void) {

    if(i2c_start(CONFIG_DS1337_ADDRESS + I2C_WRITE)) {  // Failed to issue start condition, possibly no device found
        i2c_stop();
        return;
    }
    //
    // Issuing start condition ok, device accessible
    //

    i2c_write(0x00);                    // Write register address - 0 stands for 'seconds' register
    i2c_stop();                         // Set stop condition = release bus

    //
    // Write ok, read value back from registers 0..2
    //

    i2c_start_wait(CONFIG_DS1337_ADDRESS + I2C_READ);   // Set device address and read mode
    Sec = i2c_readAck();
    Min = i2c_readAck();
    Hour = i2c_readAck();
    Hour &= 0x1F;                       // Mask out AM/PM and 12/24 bits
    Dow = i2c_readAck();
    Dow--;
    Day = i2c_readAck();
    Month = i2c_readNak();
    i2c_stop();                         // set stop condition = release bus
}

void RtcInit(void) {

    //
    // Issue Start and if all works fine, configure Alarm1 to trigger every second
    //

    if(i2c_start(CONFIG_DS1337_ADDRESS + I2C_WRITE)) {  // Failed to issue start condition, possibly no device found
        i2c_stop();
        return;
    }

    i2c_write(0x07);                    // Write register address - 7 stands for Alarm1 registers
    i2c_write(0x80);                    // Setting A1M1
    i2c_write(0x80);                    // Setting A1M2
    i2c_write(0x80);                    // Setting A1M3
    i2c_write(0x80);                    // Setting A1M4 - interrupt A every second
    i2c_stop();                         // Set stop condition = release bus

    //
    // Issue Start and if all works fine, configure the RTC to OSC and Alarm1 enabled
    //

    if(i2c_start(CONFIG_DS1337_ADDRESS + I2C_WRITE)) {  // Failed to issue start condition, possibly no device found
        i2c_stop();
        return;
    }

    i2c_write(0x0E);                    // Write register address - E stands for control register
    i2c_write(0x01);                    // OSC enabled, SquareWave 1Hz, SQW on pin 7, Alarm2 dis, Alarm1 en
    i2c_stop();                         // Set stop condition = release bus
}

void RtcClearFlags(void) {

    //
    // Issue Start and if all works fine, configure Alarm1 to trigger every second
    //

    if(i2c_start(CONFIG_DS1337_ADDRESS + I2C_WRITE)) {  // Failed to issue start condition, possibly no device found
        i2c_stop();
        return;
    }

    i2c_write(0x0F);                    // Write register address - F stands for status register
    i2c_write(0x00);                    // Clear all flags by writing 0 into them
    i2c_stop();                         // Set stop condition = release bus
}

void RtcWriteTime(void) {

    if(i2c_start(CONFIG_DS1337_ADDRESS + I2C_WRITE)) {  // Failed to issue start condition, possibly no device found
        i2c_stop();
        return;
    }
    //
    // Issuing start condition ok, device accessible - write register address first (0), then date/time
    //

    i2c_write(0x00);                    // Write register address - 0 stands for 'seconds' register
    i2c_write(Sec);
    i2c_write(Min);
    i2c_write(Hour | (1 << 6) | (AmPm << 5));   // Configure clock to 12Hr format and set AM/PM accordingly
    i2c_write(Dow + 1);
    i2c_write(Day);
    i2c_write(Month);
    i2c_stop();                         // set stop condition = release bus
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
    // Initialize timer to cca 16kHz
    // Prescaler 1/64, CTC mode
    //

    TCCR2 = (1 << WGM21) | (0 << WGM20) | (0 << COM21) | (0 << COM20) | (1 << CS22) | (0 << CS21) | (0 << CS20);
    OCR2 = 16;                          // Timer2 output compare
    TIMSK |= _BV(OCIE2);                // Enable Timer 2 Output Compare interrupt
    TIFR = _BV(OCF2);                   // Clear any pending interrupts

    //
    // Initialize ADC to AVcc, left adjusted, MUX0 input, 1/128 divider, free running.
    // Then enable it and start the conversion.
    //

    ADMUX = (1 << REFS0) | (1 << ADLAR);
    ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADFR);
    ADCSRA |= (1 << ADEN);
    ADCSRA |= (1 << ADSC);

    //
    // Keypad init - set as input, activate corresponding pull-up
    // Disable globall "PullUp Disable" in SFIOR
    //

    KEY_K1_DDR &= ~(1 << KEY_K1_BIT);
    KEY_K1_PORT |= (1 << KEY_K1_BIT);

    KEY_K2_DDR &= ~(1 << KEY_K2_BIT);
    KEY_K2_PORT |= (1 << KEY_K2_BIT);

    SFIOR &= ~(1 << PUD);

    //
    // Other HW init
    //

    i2c_init();
    RtcInit();
    RtcClearFlags();
    RtcUpdateTime();
}

//*******************************************************************************
//*                                     MAIN                                    *
//*******************************************************************************

int main(void) {

    //
    // Initialize all global variables
    //

    Brightness = Ciel8Value (1);       // Medium brightness
    memset (LedScreen, 0, sizeof (LedScreen));

    //
    // Initialize all the hardware, that is pins, I2C, RTC, and enable interrupts
    //

    HwInit();
    sei();

    //
    // Main loop
    //

    while(1) {

        //
        // Do the following every second:
        // - if the minute or hour changed, roll the changed digits
        // - otherwise, just blink the second colon
        // - unless the second is right to display date or DOW
        //

        if(RtcAlarmIsSet()) {
            RtcUpdateTime();
            RtcClearFlags();
            LedDisplayTime (Hour, Min);
            if (Sec & 0x01) {
                LedDisplayColon (1);
            } else {
                LedDisplayColon (0);
            }
        }
    }
}


