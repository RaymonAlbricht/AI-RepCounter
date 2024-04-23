/*! ***************************************************************************
 *
 * \brief     Demo application
 * \file      main.c
 * \author    Hugo Arends
 * \date      October 2023
 *
 * \copyright 2023 HAN University of Applied Sciences. All Rights Reserved.
 *            \n\n
 *            Permission is hereby granted, free of charge, to any person
 *            obtaining a copy of this software and associated documentation
 *            files (the "Software"), to deal in the Software without
 *            restriction, including without limitation the rights to use,
 *            copy, modify, merge, publish, distribute, sublicense, and/or sell
 *            copies of the Software, and to permit persons to whom the
 *            Software is furnished to do so, subject to the following
 *            conditions:
 *            \n\n
 *            The above copyright notice and this permission notice shall be
 *            included in all copies or substantial portions of the Software.
 *            \n\n
 *            THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *            EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *            OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *            NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *            HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *            WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *            FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *            OTHER DEALINGS IN THE SOFTWARE.
 *
 *****************************************************************************/
#include "millis.h"

#define F_CPU (16000000UL)

#include <avr/io.h>
#include <avr/interrupt.h>

static volatile uint32_t ms = 0;

void millis_init(void)
{
    ms = 0;

    // Configure Timer/counter 0 to generate an interrupt every millisecond
    //
    // - WGM0[2:0] = 010 : waveform generation mode is CTC with TOP in OCR0A
    // - CS0[2:0] = 011 : 64 prescaler
    // - TOP : 249
    //
    // T_events = T_CPU * N * (TOP + 1)
    //          = 1/16 MHz * 64 * (249 + 1)
    //          = 1 ms
    TCCR0A |= (1<<WGM01);
    TCCR0B |= (1<<CS01) | (1<<CS00);
    OCR0A = 249;
    
    // Timer/Counter0 Output Compare Match A Interrupt Enable
    TIMSK0 |= (1<<OCIE0A);
}

// Interrupt Service Routine that is automatically executed as soon as
// timer/counter 0 has reached its compare value
ISR(TIMER0_COMPA_vect)
{
    ms++;
}

inline uint32_t millis(void)
{
    // ms is a 32-bit variable (uint32_t). This means that multiple accesses
    // are needed to read or write the value of ms. There is a chance that
    // in the middle of these multiple accesses, the ms value is written due to
    // the ISR being triggered. In order to make sure the value of ms is not
    // updated when reading it, disable interrupts while reading the value.
    cli();
    uint32_t ret = ms;
    sei();

    return ret;
}
