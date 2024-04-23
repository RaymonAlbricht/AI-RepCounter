"""
timestamp_inspector.py

Logs data from a serial port to the terminal. Can be used at the beginning of a
project to test datacommunication between a Python script and the
microcontroller.

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
import numpy as np

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

    N_SAMPLES = 1000
    samples = []
    cnt = 0

    while True:
        try:
            # Read a line
            line = ser.readline()

            if(len(line) == 0):
                print("Data timeout")
            else:
               # Decode the line and split the comma seperated values
                line = line.rstrip().decode("ascii")
                timestamp1, timestamp2, *data = line.split(',')
                
                samples.append(int(timestamp2) - int(timestamp1))

                if len(samples) > N_SAMPLES:
                    samples.pop(0)

                avg_ms = np.average(samples)

                cnt += 1

                print("[{:>8}] average: {:5.1f}ms | min: {:5.1f}ms | " \
                    "max: {:5.1f}ms".format(cnt,avg_ms,np.min(samples),
                    np.max(samples)),end ='\r')
                
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
