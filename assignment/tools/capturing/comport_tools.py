"""
comport_tools.py

Tools related to serial ports.

Authors:    Jeroen Veen
            Hugo Arends
Date:       October 2023

Copyright:  2023 HAN University of Applied Sciences. All Rights Reserved.
"""
import serial

def available_comports() -> list:
    """
    Returns the available comports in a list
    """
    result = []
    for port in ['COM%s' % (p + 1) for p in range(255)]:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        
        except (OSError, serial.SerialException):
            pass

    return result
