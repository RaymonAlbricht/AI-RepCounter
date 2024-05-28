"""
fir_coefs_calculator.py

Helper script to calculate the coefficients for a FIR filter

Authors:    Jeroen Veen
            Hugo Arends
Date:       March 2024

Copyright:  2024 HAN University of Applied Sciences. All Rights Reserved.
"""

from scipy import signal

# Set FIR filter parameters
numtaps = 8
cutoff = 2
fs = 100

# Calculate coefficients
coefs = signal.firwin(numtaps=numtaps, cutoff=cutoff, fs=fs)

print('['+', '.join(str(x) for x in coefs)+']')

# Format output so it can be easily copy-pasted in a C source file
print('#define N_FIR ({})'.format(numtaps))
print()
print('static const float fir_coefs[N_FIR] = ')
print('{')
for coef in coefs:
    print('    {}f,'.format(coef))
print('};')
print()

# TODO Set the number of temporary arrays needed for your application
print('static float fir_x[N_FIR] = {0};')
print('static float fir_y[N_FIR] = {0};')
print('static float fir_z[N_FIR] = {0};')
