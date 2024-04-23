/*! ***************************************************************************
 *
 * \brief     Low level driver for the internal temperature sensor
 * \file      temp.h
 * \author    Hugo Arends
 * \date      June 2021
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
#include "temp.h"

/*!
 * \brief Initialises the internal temperature sensor
 *
 * This functions initializes the microcontroller's internal temperature sensor.
 */
void temp_init(void)
{
    // Enable clock to ADC0
    SIM->SCGC6 |= SIM_SCGC6_ADC0(1);
    
    // Enable clock to PORTs
    SIM->SCGC5 |= SIM_SCGC5_PORTB(1);
    
    // ADC input pin (channel 13).
    PORTB->PCR[3] &= ~0x7FF;
    
    // Configure ADC
    // - ADLPC = 1 : Low-power configuration. The power is reduced at the 
    //               expense of maximum clock speed.
    // - ADIV[1:0] = 00 : The divide ratio is 1 and the clock rate is input 
    //                    clock.
    // - ADLSMP = 1 : Long sample time.
    // - MODE[1:0] = 11 : Single-ended 16-bit conversion
    // - ADICLK[1:0] = 01 : (Bus clock)/2
    ADC0->CFG1 = 0x9D;
    
    ADC0->SC2 = 0;
}

/*!
 * \brief Takes a sample from the internal temperature sensor
 *
 * This functions takes a sample from the internal temperature sensor.
 */
float temp_get(void)
{   
   float temp = -1.0f;
   float v_temp25;
   float v_temp;
   float m;
    
    // - Start conversion by writing to SC1n register.
    // - Select Temp Sensor as input channel.
    ADC0->SC1[0] = 26;
    
    // Wait for conversion to complete
    while(!(ADC0->SC1[0] & ADC_SC1_COCO_MASK))
    {}
    
    // - Read the conversion result and calculate the measured voltage.
    //   Assign the result to v_temp:
    // - The conversion result can be read from register ADC0->R[0].
    // - The reference voltage is 3.3V.
    // - The voltage was measured with 16-bit resolution.  
    v_temp = 3.3f * (float)(ADC0->R[0]) / 65535.0f;
        
    // From KL25 Sub-Family Reference Manual, Rev. 3, September 2012
    // Section 28.4.8:
    //
    //  "For temperature calculations, use the V_TEMP25 and temperature sensor
    //   slope values from the ADC Electricals table."
    //
    // The ADC Electrics table is found in the datasheet:
    // Kinetis KL25 Sub-Family, Rev5 08/2014. Table 26.
    v_temp25 = 0.716f;

    // From KL25 Sub-Family Reference Manual, Rev. 3, September 2012
    // Section 28.4.8:
    //
    //  "In application code, the user reads the temperature sensor channel, 
    //   calculates V_TEMP , and compares to V_TEMP25. If V_TEMP is greater than 
    //   V_TEMP25 the cold slope value is applied in the preceding equation. If 
    //   V_TEMP is less than V_TEMP25, the hot slope value is applied in the 
    //   preceding equation. ADC Electricals table may only specify one 
    //   temperature sensor slope value. In that case, the user could use the
    //   same slope for the calculation across the operational temperature 
    //   range."
    //
    // Notice that there is only a single slope value mentioned in 
    // Kinetis KL25 Sub-Family, Rev5 08/2014. Table 26.
    m = 0.00162f;
        
    // - Implement the approximate transfer function of the temperature
    //   Use the variables v_temp, v_temp25 and m.
    temp = 25 - ((v_temp - v_temp25) / m);

    return temp;
}
