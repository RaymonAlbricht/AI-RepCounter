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

#define F_CPU (16000000UL)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdio.h>

#include "millis.h"
#include "sw0.h"
#include "usart0.h"

float angle = 0.0f;
const float intervalAngle = (2 * 3.14159265359f / 100);

// For redirecting stdout to USART0
int usart0_putchar(char c, FILE *stream)
{
    usart0_transmit(c);
    return 0;
}

static FILE usart0_stdout = FDEV_SETUP_STREAM(usart0_putchar, NULL, _FDEV_SETUP_WRITE);

// Main application
int main(void)
{
    // Redirect stdout to USART0
    stdout = &usart0_stdout;

    // PB5 output
    DDRB |= (1<<DDB5);

    // Initialise
    sw0_init();
    usart0_init();

    // Initialize the millisecond counter
    millis_init();

    // Enable interrupts
    sei();

    uint32_t previousmillis = 0;
    uint32_t ms1 = 0;
    uint32_t ms2 = 0;
    uint32_t interval = 10;

    printf("Application started\n");
    
    while(1) 
    {
        // Set initial timestamp
        ms1 = millis();

        // Has 'interval' time passed since the last time we checked?
        if((ms1 - previousmillis) >= interval)
        {
            previousmillis = ms1;

            // Generate artificial example data
            float acc_x_mg = 1000 * sinf(angle);
            float acc_y_mg = 500 * sinf(angle);
            float acc_z_mg = 100 * cosf(angle);

            angle += intervalAngle;

            // Set final timestamp
            ms2 = millis();

            // Send the data
            printf("%lu,%lu,%.1f,%.1f,%.1f\n",
                ms1,
                ms2,
                (double)acc_x_mg,
                (double)acc_y_mg,
                (double)acc_z_mg);

            // TODO Implement filter functions as required by the application.

            // TODO Implement normalization functions as required by the
            //      application.

            // TODO Finish this example by designing an ML model and implement
            //      the generated C code.
        }
    }
}
