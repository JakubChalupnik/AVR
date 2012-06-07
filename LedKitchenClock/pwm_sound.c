//---------------------------------------------------------------------------
//
// FILENAME.
//      pwm_sound.c
//
// FUNCTIONAL DESCRIPTION.
//      Basic sound support for AVR, using timer and pwm, playing from program flash memory
//
// PROCESSOR.
//      AVR ATMega8
//
// LICENSE.
//      The code comes from http://avrpcm.blogspot.de/2010/11/playing-8-bit-pcm-using-any-avr.html
//      author calls himself Rejith
//
//---------------------------------------------------------------------------

#include "defs.h"
#include "hw.h"
#include "global.h"

#include <avr/io.h>                     // include I/O definitions (port names, pin names, etc)
#include <avr/signal.h>                 // include "signal" names (interrupt names)
#include <avr/interrupt.h>              // include interrupt support
#include <avr/pgmspace.h>

static const byte Chord[] PROGMEM = {
#include "chord.h"
};

volatile uint16_t sample;
volatile uint16_t sample_pause;
int sample_count;
const int pcm_length = sizeof(Chord);


/* initialise the PWM */
void pwm_init(void) {

    /* use OC1A pin as output */
    DDRB |= _BV(PB1);

    /*
     * clear OC1A on compare match
     * set OC1A at BOTTOM, non-inverting mode
     * Fast PWM, 8bit
     */
    TCCR1A = _BV(COM1A1) | _BV(WGM10);

    /*
     * Fast PWM, 8bit
     * Prescaler: clk/1 = 8MHz
     * PWM frequency = 8MHz / (255 + 1) = 31.25kHz
     */
    TCCR1B = _BV(WGM12) | _BV(CS10);

    /* set initial duty cycle to zero */
    OCR1A = 0;

    /* Setup Timer0 */

    TCCR0 |= (1 << CS00);
    TCNT0 = 0;
//     TIMSK |= (1 << TOIE0);
    sample_count = 8;
    sample_pause = SAMPLE_PAUSE;
}

SIGNAL(TIMER0_OVF_vect) {

    sample_count--;
    if(sample_count == 0) {
        sample_count = 8;
        if (sample <= pcm_length) {
            OCR1A = pgm_read_byte(&Chord[sample++]);
        } else {
            OCR1A = 0;
            if (sample_pause > 0) {
                sample_pause--;
            } else {
                sample = 0;
                sample_pause = SAMPLE_PAUSE;
            }
        }
    }
}
