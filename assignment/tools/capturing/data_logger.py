"""
data_logger.py

Logs data from a serial port to the terminal. Can be used at the beginning of a
project to test datacommunication between a Python script and the
microcontroller.

Strings must be termintated with a '\n' character.

Authors:    Jeroen Veen
            Hugo Arends
Date:       October 2023

Copyright:  2023 HAN University of Applied Sciences. All Rights Reserved.
"""
import sys
from os.path import join, dirname, realpath
sys.path.append(join(dirname(realpath(__file__)), '..'))

import config as cfg
import comport_tools as cpt
import serial

def main():
    print("Press CTRL+C to quit")

    # Open COM port
    ser = serial.Serial()
    ser.port = cfg.COMPORT
    ser.baudrate = cfg.BAUDRATE
    ser.timeout = 3

    try:
        ser.open()
    except:
        print("Could not open %s @ %sbps" % (ser.port, ser.baudrate))
        print("Available COM ports: " + str(cpt.available_comports()))
        exit()

    print("Opened %s @ %sbps" % (ser.port, ser.baudrate))

    # Reset serial
    ser.reset_output_buffer()
    ser.reset_input_buffer()

    # Discard the first line, because logging might be started in the middle of
    # the transmission of a string
    ser.readline()

    while True:
        try:
            # Read a line
            line = ser.readline()

            if(len(line) == 0):
                print("Data timeout")
            else:
                print(line.rstrip().decode("ascii"))

        except KeyboardInterrupt:
            print("Stop app")
            break
        
        except Exception as e:
            print("Abort app: %s" % e)
            break

    ser.flush()
    ser.close()
    print("Closed %s" % ser.port)


if __name__ == "__main__":
    main()
