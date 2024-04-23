/*! ***************************************************************************
 *
 * \brief     Low level driver for the onboard RGB LED
 * \file      rgb.c
 * \author    Hugo Arends
 * \date      April 2021
 *
 * \copyright 2021 HAN University of Applied Sciences. All Rights Reserved.
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
#include "rgb.h"

/*!
 * \brief Initialises the onboard RGB LED
 *
 * This functions initializes the onboard RGB LED. The LEDs are connected as
 * follows:
 * - Red: PTB18
 * - Green: PTB19
 */
void rgb_init(void)
{
    // Enable clock to PORTs
    SIM->SCGC5 |= SIM_SCGC5_PORTB(1);

    // Set pin function GPIO
    PORTB->PCR[18] = PORT_PCR_MUX(1);
    PORTB->PCR[19] = PORT_PCR_MUX(1);

    // Output pins
    GPIOB->PDDR |= (1<<18);
    GPIOB->PDDR |= (1<<19);
    
    // LEDs off
    GPIOB->PSOR = (1<<18);
    GPIOB->PSOR = (1<<19);
}

inline void rgb_red(const bool on)
{
    if(on == false)
    {
        GPIOB->PSOR = (1<<18);
    }
    else
    {
        GPIOB->PCOR = (1<<18);
    }
}

inline void rgb_green(const bool on)
{
    if(on == false)
    {
        GPIOB->PSOR = (1<<19);
    }
    else
    {
        GPIOB->PCOR = (1<<19);
    }
}
