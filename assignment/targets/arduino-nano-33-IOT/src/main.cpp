/*! ***************************************************************************
 *
 * \brief     Demo application
 * \file      main.cpp
 * \author    Hugo Arends
 *            Jeroen Veen
 * \date      September 2023
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
#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include "features.h"

#define BLOCK_SIZE 175
#define NUMBER_OF_COEFS 8

int n = 0;

float x, y, z;
float acceleration_mg[3] = {0};
float x_buffer[BLOCK_SIZE] = {0};
float y_buffer[BLOCK_SIZE] = {0};
float z_buffer[BLOCK_SIZE] = {0};
float fir_x[8] = {0};
float fir_y[8] = {0};
float fir_z[8] = {0};

float coefs[8] = {0.020179930612355165, 0.06489483637184779, 0.16638970522168856, 0.24853552779410845, 0.24853552779410845, 0.16638970522168856, 0.06489483637184779, 0.020179930612355165};

uint32_t ms1 = 0;
uint32_t ms2 = 0;

typedef enum
{
    BicepCurl = 0,
    ChestPress = 1,
    LatPulldown = 2,
    Stationary = 3,
}dtc_t;

/*
 * \brief Decision tree classifier
 * 
 * Decision tree classifier based on the following input characteristics:
 *   BLOCK_SIZE: 175
 *   BLOCK_TYPE: SLIDING
 * 
 * \return dtc_t
 *   0  BicepCurl
 *   1  ChestPress
 *   2  LatPulldown
 *   3  Stationary
 */
dtc_t dtc(const float y_out_fir_rescale_mean, const float z_out_fir_rescale_min, const float z_out_fir_rescale_max)
{
    dtc_t ret;

    if(y_out_fir_rescale_mean <= 0.754241f)
    {
        if(z_out_fir_rescale_min <= -0.606537f)
        {
             ret = BicepCurl;
        }
        else // z_out_fir_rescale_min > -0.606537f
        {
            if(z_out_fir_rescale_max <= 0.843368f)
            {
                 ret = Stationary;
            }
            else // z_out_fir_rescale_max > 0.843368f
            {
                 ret = ChestPress;
            }
        }
    }
    else // y_out_fir_rescale_mean > 0.754241f
    {
         ret = LatPulldown;
    }

    return ret;
}




float fir(const float data, const float *coefs, float *x, const uint32_t n)
{
    float y = 0.0f;

    // Shift the data samples and calculate its contribution
    for (int i = (n - 1); i > 0; --i)
    {
        x[i] = x[i - 1];
        y = y + (coefs[i] * x[i]);
    }

    // Add new data sample and calculate its contribution
    x[0] = data;
    y = y + (coefs[0] * x[0]);

    return y;
}

float normalize(float value, float min_val, float max_val)
{
    return (value - min_val) / (max_val - min_val);
}

float max(float *data, const uint32_t n)
{
    float max = data[0];

    for(uint32_t i=1; i<n; ++i)
    {
        max = (data[i] > max) ? data[i] : max;
    }

    return max;
}

float min(float *data, const uint32_t n)
{
    float min = data[0];

    for(uint32_t i=1; i<n; ++i)
    {
        min = (data[i] < min) ? data[i] : min;
    }

    return min;
}


void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("Initializing sensors\n");

    if (!IMU.begin())
    {
        Serial.println("Failed to initialize IMU!\n");
        while (1)
            ;
    }
    Serial.print("Accelerometer sample rate ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
    if (IMU.accelerationAvailable())
    {
        // Set initial timestamp
        IMU.readAcceleration(x, y, z);

    // Buffer full?
    if (n >= BLOCK_SIZE)
    {
      
      memmove(&x_buffer[0], &x_buffer[1], sizeof(float) * (BLOCK_SIZE - 1));
      memmove(&y_buffer[0], &y_buffer[1], sizeof(float) * (BLOCK_SIZE - 1));
      memmove(&z_buffer[0], &z_buffer[1], sizeof(float) * (BLOCK_SIZE - 1));
 
      n = BLOCK_SIZE - 1; // Reset counter at the end of the buffer
    }

    y_buffer[n]= y;
    z_buffer[n]= z;

    n++;

    if (n >= BLOCK_SIZE){

    for(int i = 0; i <= BLOCK_SIZE; i++){
          //x_buffer[i] = fir(x_buffer[i], coefs, fir_x, NUMBER_OF_COEFS);
          y_buffer[i] = fir(y_buffer[i], coefs, fir_y, NUMBER_OF_COEFS);
          z_buffer[i] = fir(z_buffer[i], coefs, fir_z, NUMBER_OF_COEFS);

          //x_buffer[i] = normalize(x_buffer[i],-1,1);
          y_buffer[i] = normalize(y_buffer[i],-1,1);
          z_buffer[i] = normalize(z_buffer[i],-1,1);
    }

          float y_result = mean(y_buffer,BLOCK_SIZE);
          float z_min_result = min(z_buffer,BLOCK_SIZE);
          float z_max_result = max(z_buffer,BLOCK_SIZE);

           int label = dtc(y_result,z_min_result, z_max_result);

        char* label_str;
        // Use the calculated label forfurther processing
        if(label == 0)
        {
          label_str="BicepCurl";
        }
        else if(label == 1)
        {
          label_str="ChestPress";
        }
        else if(label == 2)
        {
          label_str="LatPulldown";
        }
        else if(label == 3)
        {
          label_str="Stationary";
        }

    Serial.println(label_str);
    }
    }
}