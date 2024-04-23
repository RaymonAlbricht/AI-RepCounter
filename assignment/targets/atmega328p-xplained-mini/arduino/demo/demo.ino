/*! ***************************************************************************
 *
 * \brief     Demo application
 * \file      demo.ino
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

unsigned long previousMillis = 0;
const long intervalMillis = 10;

float angle = 0.0f;
const float intervalAngle = 2*PI / 100;

void setup(){
  Serial.begin(9600);
}

void loop(){

  // Set initial timestamp
  unsigned long ms1 = millis();

  if (ms1 - previousMillis >= intervalMillis) {
    previousMillis = ms1;

    // Generate artificial example data
    float acc_x_mg = 1000 * sinf(angle);
    float acc_y_mg = 500 * sinf(angle);
    float acc_z_mg = 100 * cosf(angle);

    angle += intervalAngle;

    // Set final timestamp
    unsigned long ms2 = millis();

    // Send the data
    Serial.print(ms1);
    Serial.print(',');
    Serial.print(ms2);
    Serial.print(',');
    Serial.print(acc_x_mg);
    Serial.print(',');
    Serial.print(acc_y_mg);
    Serial.print(',');
    Serial.println(acc_z_mg);

    // TODO Implement filter functions as required by the application.

    // TODO Implement normalization functions as required by the
    //      application.

    // TODO Finish this example by designing an ML model and implement the
    //      generated C code.

  }
}