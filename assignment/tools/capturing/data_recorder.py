"""
data_recorder.py

Records data from a serial port and saves the data in a file as a CustomBunch.
The commandline parameter -l must be used to set the label. The file is saved
with name 'raw_<label>.csv'.

This script assumes that the microcontroller data is formatted as follows:
<timestamp 1>,<timestamp 2>,<attribute 1>,<attribute 2>,etc

IMPORTANT. The attributes are application depended and must be set manually in
this file!. Refer to the list called 'attributes'.

Authors:    Jeroen Veen
            Hugo Arends
Date:       October 2023

Copyright:  2023 HAN University of Applied Sciences. All Rights Reserved.
"""
import sys
from os.path import join, dirname, realpath
sys.path.append(join(dirname(realpath(__file__)), '..'))

import config as cfg
from custom_bunch import CustomBunch
import comport_tools as cpt
from os.path import join
import argparse
from numpy import array
import serial

# TODO Set the label for the recording here. Alternatively, use the command
#      line -l option
LABEL_NAME = 'stationary'

# TODO Set the attribute names from the microcontroller data here.
#      These attributes are application dependent! The attributes correspond
#      to the comma seperated sensor values transmitted by the microcontroller.
ATTRIBUTE_NAMES = ['x_out','y_out','z_out']

# TODO Set number of samples (or press CTRL+C to abort)
N_SAMPLES = 1000

def main():
    print("Press CTRL+C to quit")

    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--label', default=LABEL_NAME, 
        help="label for the captured data")
    args = parser.parse_args()

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

    cnt = 0
    labels = []
    timestamps = []
    data = []

    # Reset serial
    ser.reset_output_buffer()
    ser.reset_input_buffer()

    # Discard the first line, because logging might be started in the middle of
    # the transmission of a string
    ser.readline()

    while cnt < N_SAMPLES:
        try:
            # Read a line
            line = ser.readline()

            if(len(line) == 0):
                print("Data timeout")
            else:
                # Decode the line and split the comma seperated values
                line = line.rstrip().decode("ascii")
                csv = line.split(',')
                
                # Parse data required by custom bunch
                timestamps.append([int(d) for d in csv[:2]])
                data.append([float(d) for d in csv[2:2+len(ATTRIBUTE_NAMES)]])
                labels.append(args.label)

                # Print the data and a counter, so the number of samples is
                # visible
                cnt += 1
                print(("[{:>8}] " + args.label + ',' + line).format(cnt))
                
        except KeyboardInterrupt:
            print("Stop app")
            break
        
        except Exception as e:
            print("Abort app: %s" % e)
            break

    ser.flush()
    ser.close()
    print("Closed %s" % ser.port)

    # Create bunch from the recorded data
    name = args.label
    bunch = CustomBunch(data, timestamps=timestamps, attributes=ATTRIBUTE_NAMES,
        labels=labels, name=name)
    print(bunch.print_summary())
    
    # Save the bunch
    filename = join(cfg.CAPTURED_DIR_PATH, name+'.csv')
    bunch.save_csv(filename)

    print('Files written:')
    print(filename)


if __name__ == "__main__":
    main()
