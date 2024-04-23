/*! ***************************************************************************
 *
 * \brief     Demo application
 * \file      main.c
 * \author    Hugo Arends
 * \date      July 2023
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
#include <MKL25Z4.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "bsp.h"
#include "features.h"
#include "filters.h"
#include "normalizations.h"
#include "temp.h"

#define N_BUFFER (100)

static volatile uint32_t ms = 0;
//static volatile uint32_t prev_ms = 0;

static uint32_t n = 0;
static float buffer_x_out[N_BUFFER];
static float buffer_y_out[N_BUFFER];
static float buffer_z_out[N_BUFFER];

#define N_FIR (8)

static const float fir_coefs[N_FIR] =
{
    0.020179930612355165f,
    0.06489483637184779f,
    0.16638970522168856f,
    0.24853552779410845f,
    0.24853552779410845f,
    0.16638970522168856f,
    0.06489483637184779f,
    0.020179930612355165f,
};

static float fir_x[N_FIR] = {0};
static float fir_y[N_FIR] = {0};
static float fir_z[N_FIR] = {0};

// Functions for redirectiing standard output to UART0
int stdout_putchar(int ch)
{
    uart0_put_char((char)ch);
    return ch;
}

int stderr_putchar(int ch)
{
    return stdout_putchar(ch);
}

int stdin_getchar(void)
{
    return uart0_get_char();
}


typedef enum
{
    left_right = 0,
    stationary = 1,
    up_down = 2,
}dtc_t;

/*
 * \brief Decision tree classifier
 * 
 * Decision tree classifier based on the following input characteristics:
 *   BLOCK_SIZE: 100
 *   BLOCK_TYPE: BLOCK
 * 
 * \return dtc_t
 *   0  left_right
 *   1  stationary
 *   2  up_down
 */
dtc_t dtc(const float x_out_fir_rescale_variance, const float y_out_fir_rescale_variance)
{
    dtc_t ret;

    if(x_out_fir_rescale_variance <= 0.000538f)
    {
         ret = stationary;
    }
    else // x_out_fir_rescale_variance > 0.000538f
    {
        if(y_out_fir_rescale_variance <= 0.032295f)
        {
             ret = left_right;
        }
        else // y_out_fir_rescale_variance > 0.032295f
        {
             ret = up_down;
        }
    }

    return ret;
}


int main(void)
{
    rgb_init();
    uart0_init();

    // Generate SysTick interrupt every millisecond
    SysTick_Config(48000-1);

    // Blink Green LED
    rgb_green(true);
    delay_us(1000);
    rgb_green(false);

    // -------------------------------------------------------------------------
    // MMA8451 initialize
    // -------------------------------------------------------------------------

    printf("Initializing MMA8451Q");

    if(!mma8451_init())
    {
        // Set pins to GPIO function
        PORTE->PCR[24] &= ~PORT_PCR_MUX_MASK;
        PORTE->PCR[25] &= ~PORT_PCR_MUX_MASK;
        PORTE->PCR[24] |= PORT_PCR_MUX(1);
        PORTE->PCR[25] |= PORT_PCR_MUX(1);

        // Set pins to output
        PTE->PDDR |= (1<<24) | (1<<25);

        // Set pins high
        PTE->PSOR = (1<<24) | (1<<25);

        printf(" failed, please reset the device\n");

        while(1)
        {
            rgb_red(true);
            delay_us(1000);
            rgb_red(false);
            delay_us(1000000);
        }
    }
    // Calibrate
    if(!mma8451_calibrate())
    {
        // Set pins to GPIO function
        PORTE->PCR[24] &= ~PORT_PCR_MUX_MASK;
        PORTE->PCR[25] &= ~PORT_PCR_MUX_MASK;
        PORTE->PCR[24] |= PORT_PCR_MUX(1);
        PORTE->PCR[25] |= PORT_PCR_MUX(1);

        // Set pins to output
        PTE->PDDR |= (1<<24) | (1<<25);

        // Set pins high
        PTE->PSOR = (1<<24) | (1<<25);

        printf(" failed, please reset the device\n");

        while(1)
        {
            rgb_red(true);
            delay_us(1000);
            rgb_red(false);
            delay_us(1000000);
        }
    }

    printf(" done\n");
    printf("ODR = 100Hz\n");
    printf("+/-2g output scale\n");

    // -------------------------------------------------------------------------
    // Internal temperature sensor initialize
    // -------------------------------------------------------------------------

    printf("Initializing internal temperature sensor");

    temp_init();

    printf(" done\n");

    // For debugging
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB->PCR[0] |= PORT_PCR_MUX(1);
    PTB->PDDR |= (1<<0);

    uint32_t ms1 = 0;
    uint32_t ms2 = 0;

    while(1)
    {
        // Check if user pressed a key
        if(uart0_num_rx_chars_available() > 0)
        {
            char c = uart0_get_char();

            if(c == ' ')
            {
                // Blink Green LED
                rgb_green(true);
                delay_us(1000);
                rgb_green(false);
            }
        }

#define RAW
//#define BLOCK
//#define SLIDING

#ifdef RAW

        // Handle the data as soon as new data is available
        // mma8451 accelerometer Output Data Rate (ODR) is set to 100 Hz
        if(mma8451_ready_flag)
        {
            // Set initial timestamp
            ms1 = ms;
            
            // Clear the flag
            mma8451_ready_flag = false;

            // Reads the data in three global variables: x_out_mg, y_out_mg and
            // z_out_mg
            mma8451_read();
          //float t = temp_get();

            // Set final timestamp
            ms2 = ms;

            // Send the data
            // Send the raw data
            printf("%d,%d,%.3f,%.3f,%.3f\n",
                ms1,
                ms2,
                (double)(x_out_mg),
                (double)(y_out_mg),
                (double)(z_out_mg));

            // TODO Implement filter function as required by the application.

            // TODO Implement normalization function as required by the
            //      application.

            // TODO Finish this example by designing an ML model and implement the
            //      generated C code.
        }

#endif

#ifdef BLOCK

        // Handle the data as soon as new data is available
        // mma8451 accelerometer Output Data Rate (ODR) is set to 100 Hz
        if(mma8451_ready_flag)
        {
            // Set initial timestamp
            ms1 = ms;
            
            // Clear the flag
            mma8451_ready_flag = false;

            // Reads the data in three global variables: x_out_mg, y_out_mg and
            // z_out_mg
            mma8451_read();
          //float t = temp_get();
          
            // TODO Implement filter function as required by the application.

            // Filter accelerometer data
            x_out_mg = fir(x_out_mg, fir_coefs, fir_x, N_FIR);
            y_out_mg = fir(y_out_mg, fir_coefs, fir_y, N_FIR);
            z_out_mg = fir(z_out_mg, fir_coefs, fir_z, N_FIR);
            
            // TODO Implement normalization function as required by the
            //      application.

            // Scale accelerometer data
            const float from[2] = {-1000.0f, 1000.0f};
            const float to[2] = {-1.0f, 1.0f};

            x_out_mg = rescale(x_out_mg, from, to);
            y_out_mg = rescale(y_out_mg, from, to);
            z_out_mg = rescale(z_out_mg, from, to);
            
            // TODO Finish this example by designing an ML model and implement
            //      the generated C code.

            // Add accelerometer data to the buffer
            buffer_x_out[n] = x_out_mg;
            buffer_y_out[n] = y_out_mg;
            buffer_z_out[n] = z_out_mg;
            
            n++;

            // Buffer full?
            if(n >= N_BUFFER)
            {
                // Reset buffer counter for next block
                n = 0;

                // Calculate features by using feature functions
                float x_out_var = variance(buffer_x_out, N_BUFFER);
                float y_out_var = variance(buffer_y_out, N_BUFFER);

                // Calculate label by using the generated Decision Tree
                // Classifier
                dtc_t label = dtc(x_out_var, y_out_var);
                
                char *label_str = "";

                // Use the calculated label for further processing
                if(label == stationary)
                {
                     label_str = "stationary";
                     rgb_green(false);
                     rgb_red(false);
                }
                else if(label == up_down)
                {
                     label_str = "up_down";
                     rgb_green(true);
                     rgb_red(false);
                }
                else if(label == left_right)
                {
                     label_str = "left_right";
                     rgb_green(false);
                     rgb_red(true);
                }

                // Set final timestamp
                ms2 = ms;
                
                // Print duration and label
                printf("%d,%d,%s\n",
                    ms1,
                    ms2,
                    label_str);
            }
        }

#endif

#ifdef SLIDING

    // Handle the data as soon as new data is available
    // mma8451 accelerometer Output Data Rate (ODR) is set to 100 Hz
    if(mma8451_ready_flag)
    {
        // Set initial timestamp
        ms1 = ms;
        
        // Clear the flag
        mma8451_ready_flag = false;

        // Reads the data in three global variables: x_out_mg, y_out_mg and
        // z_out_mg
        mma8451_read();
        //float t = temp_get();
        
        // TODO Implement filter function as required by the application.

        // Filter accelerometer data
        x_out_mg = fir(x_out_mg, fir_coefs, fir_x, N_FIR);
        y_out_mg = fir(y_out_mg, fir_coefs, fir_y, N_FIR);
        z_out_mg = fir(z_out_mg, fir_coefs, fir_z, N_FIR);
        
        // TODO Implement normalization function as required by the
        //      application.

        // Scale accelerometer data
        const float from[2] = {-1000.0f, 1000.0f};
        const float to[2] = {-1.0f, 1.0f};

        x_out_mg = rescale(x_out_mg, from, to);
        y_out_mg = rescale(y_out_mg, from, to);
        z_out_mg = rescale(z_out_mg, from, to);
        
        // TODO Finish this example by designing an ML model and implement
        //      the generated C code.

        // Buffer full?
        if(n >= (N_BUFFER-1))
        {
            // Remove first data item in the buffer and move all others one
            // position.
            memmove(&buffer_x_out[0], &buffer_x_out[1], sizeof(float) * (N_BUFFER-1));
            memmove(&buffer_y_out[0], &buffer_y_out[1], sizeof(float) * (N_BUFFER-1));
            memmove(&buffer_z_out[0], &buffer_z_out[1], sizeof(float) * (N_BUFFER-1));
        }

        // Add accelerometer data to the buffer
        buffer_x_out[n] = x_out_mg;
        buffer_y_out[n] = y_out_mg;
        buffer_z_out[n] = z_out_mg;

        n++;

        // Buffer full?
        if(n >= N_BUFFER)
        {
            // Set counter at the end of the buffer
            n = N_BUFFER-1;

            // Calculate features by using feature functions
            float y_out_var = variance(buffer_y_out, N_BUFFER);

            // Calculate label by using the generated Decision Tree
            // Classifier
            dtc_t label = dtc(y_out_var);
            
            char *label_str = "";

            // Use the calculated label for further processing
            if(label == stationary)
            {
                    label_str = "stationary";
                    rgb_green(false);
                    rgb_red(false);
            }
            else if(label == up_down)
            {
                    label_str = "up_down";
                    rgb_green(true);
                    rgb_red(false);
            }
            else if(label == left_right)
            {
                    label_str = "left_right";
                    rgb_green(false);
                    rgb_red(true);
            }

            // Set final timestamp
            ms2 = ms;
            
            // Print duration and label
            printf("%d,%d,%s\n",
                ms1,
                ms2,
                label_str);
        }
    }
        
#endif

    }
}

void SysTick_Handler(void)
{
    ms++;
}
