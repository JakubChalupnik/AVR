//---------------------------------------------------------------------------
//
// FILENAME.
//      main.c
//
// FUNCTIONAL DESCRIPTION.
//      Test program for the BMP085 barometric pressure sensor.
//      Uses AVRLIB, compiled with WinAVR
//
// PROCESSOR.
//      ATMega32
//
// MODIFICATION HISTORY.
//      Kubik   24.6.2012 Initial version
//
// LICENSE.
//      Copyright 2012 Jakub Chalupnik (JakubChalupnik@gmail.com). All rights reserved.
//      (see license.txt)
//
//---------------------------------------------------------------------------

//
// Include files, first common includes, then project specific includes
//

#include "global.h"                     // include our global settings

#include <avr/io.h>                     // include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>              // include interrupt support
#include <util/delay.h>

#include "global.h"                     // include our global settings
#include "uart.h"                       // include uart function library
#include "rprintf.h"                    // include printf function library
#include "timer.h"                      // include timer function library (timing, PWM, etc)
#include "vt100.h"                      // include VT100 terminal support
#include "i2csw.h"                      // include I2C SW bitbang support
#include "bmp085.h"

//
// Local constant defines
//

//
// Macro definitions
//

//
// Local typedefs
//

//
// Static variables
//

//
// External function declarations
//

//
// Local functions
//

int main (void) {
    uint16_t TempRaw;
    uint32_t PressRaw;
    short Temperature;
    long Pressure;
    float PressureSeaLevel;

    //
    // Initiaze AVR libraries used in the project: UART timer I2C
    //

    uartInit ();
    uartSetBaudRate (9600);
    timerInit ();
    i2cInit ();

    bmp085Init ();

    //
    // Initialize rprintf system
    // - use uartSendByte as the output for all rprintf statements
    //

    rprintfInit (uartSendByte);

    vt100Init ();                       // Initialize vt100 library

    vt100ClearScreen ();                // Clear the terminal screen

    rprintf ("Temperature / pressure measurement\n");

    bmp085Init ();

    while (1) {
        TempRaw = bmp085ReadUT ();
        PressRaw = bmp085ReadUP ();

        Temperature = bmp085ConvertTemperature (TempRaw);
        Pressure = bmp085ConvertPressure (PressRaw);
        PressureSeaLevel = bmp085GetSeaLevelPressure ((float) Pressure, 519);   // 519 is an average Munich altitude

        rprintf ("Temp %d, pressure ", Temperature);
        rprintfNum (10, 6, FALSE, ' ', Pressure);
        rprintf ("Pa, sea level %dhPa\n", (short) (PressureSeaLevel / 100));

        _delay_ms (1000);
    }

    return 0;
}
