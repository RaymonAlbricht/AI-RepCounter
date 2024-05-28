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
#include <dtc_model.h>

float x, y, z;
float acceleration_mg[3] = {0};

uint32_t ms1 = 0;
uint32_t ms2 = 0;

typedef enum
{
    Bicepcurl = 0,
    Chestpress = 1,
    Latpulldown = 2,
    Stationary = 3,
}dtc_t;

/*
 * \brief Decision tree classifier
 * 
 * Decision tree classifier based on the following input characteristics:
 *   BLOCK_SIZE: 100
 *   BLOCK_TYPE: BLOCK
 * 
 * \return dtc_t
 *   0  Bicepcurl
 *   1  Chestpress
 *   2  Latpulldown
 *   3  Stationary
 */
dtc_t dtc(const float y_out_fir_rescale_max, const float z_out_fir_rescale_max)
{
    dtc_t ret;

    if(y_out_fir_rescale_max <= 0.014531f)
    {
         ret = Stationary;
    }
    else // y_out_fir_rescale_max > 0.014531f
    {
        if(z_out_fir_rescale_max <= 0.788158f)
        {
            if(y_out_fir_rescale_max <= 0.945533f)
            {
                 ret = Bicepcurl;
            }
            else // y_out_fir_rescale_max > 0.945533f
            {
                 ret = Latpulldown;
            }
        }
        else // z_out_fir_rescale_max > 0.788158f
        {
             ret = Chestpress;
        }
    }

    return ret;
}



void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  // delay(1000);
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
    ms1 = millis();
    
    IMU.readAcceleration(x, y, z);
    acceleration_mg[0] = x * 1000.0;
    acceleration_mg[1] = y * 1000.0;
    acceleration_mg[2] = z * 1000.0;

    // Set final timestamp
    ms2 = millis();

    // // Send the data
    // Serial.print(ms1);
    // Serial.print(',');
    // Serial.print(ms2);
    // Serial.print(',');
    // Serial.print(acceleration_mg[0]);
    // Serial.print(',');
    // Serial.print(acceleration_mg[1]);
    // Serial.print(',');
    // Serial.println(acceleration_mg[2]);

    // TODO Implement filter functions as required by the application.
      
    // TODO Implement normalization functions as required by the
    //      application.

    // TODO Finish this example by designing an ML model and implement the
    //      generated C code.
  }
}
