"""
feature_calculator.py

Calculates features

Authors:    Jeroen Veen
            Hugo Arends
Date:       October 2023

Copyright:  2023 HAN University of Applied Sciences. All Rights Reserved.
"""
import sys
from os.path import join, dirname, realpath
sys.path.append(join(dirname(realpath(__file__)), '..', '..'))

from custom_bunch import CustomBunch
import config as cfg
from glob import glob
import matplotlib.pyplot as plt
from os.path import join
import numpy as np
import feature_functions as ff


# TODO Set feature functions here
FEATURE_FUNCTIONS = [ff.variance]
# FEATURE_FUNCTIONS = [ff.raw,ff.min,ff.max,ff.mean,ff.variance,ff.energy,ff.peak_to_peak]
# FEATURE_FUNCTIONS = [np.mean,np.var,np.median,np.ptp,np.std]

# TODO Set input directory path for feature calculation
#INPUT_DIR_PATH = cfg.CAPTURED_DIR_PATH
#INPUT_DIR_PATH = cfg.PREPROCESSING_FILTERS_DIR_PATH
INPUT_DIR_PATH = cfg.PREPROCESSING_NORMALIZATIONS_DIR_PATH

def main():

    print('BLOCK_SIZE: '+str(cfg.BLOCK_SIZE))
    print('BLOCK_TYPE: '+str(cfg.BLOCK_TYPE))

    if __name__ == "__main__":
        print('FEATURE_FUNCTIONS: ' +
            str([f.__name__ for f in FEATURE_FUNCTIONS]))

    # Get paths of all captured data files
    filenames = glob(join(INPUT_DIR_PATH, '*.csv'))

    assert len(filenames) != 0, 'No CSV files'

    # Load captured data in a list of bunches
    bunches = [CustomBunch.load_csv(f) for f in filenames]
    new_bunches = []

    # Loop all bunches
    for bunch in bunches:

        # Featurs can only be calculated of data consisting of a single label
        assert len(bunch.unique_labels) == 1, \
            'Expected a bunch with a unique label'

        data = []
        labels = []
        timestamps = []
        attributes = []

        # Loop all attributes of this bunch
        for attr in bunch.attributes:
            # Get a slice containing this attributes data
            d = bunch.data[:, bunch.attributes.index(attr)]

            # BLOCK type?
            if cfg.BLOCK_TYPE == 'BLOCK':
                # Make the length of the data a multiple of the BLOCK_SIZE
                d = d[0:(len(d) - (len(d) % int(cfg.BLOCK_SIZE)))]
                # Reshape the data to blocks of size BLOCK_SIZE
                d = np.reshape(d, (-1, int(cfg.BLOCK_SIZE)))

                # Loop all feature functions
                for f in FEATURE_FUNCTIONS:
                    r = []
                    # Loop all blocks
                    for block in d:
                        # Append the calculated feature of this block to the
                        # result
                        r.append(f(block))        
                    # Append the result to the data
                    data.append(r)
                    # Combine this attribute and feature name to a new
                    # attribute
                    attributes.append(attr+'_'+f.__name__)
        
            # Sliding type?
            if cfg.BLOCK_TYPE == 'SLIDING':
                # Loop all feature functions
                for f in FEATURE_FUNCTIONS:
                    # Copy the original data
                    tmp = d
                    r = []
                    # Loop the data while its length is greater or equal to the
                    # BLOCK_SIZE
                    while(len(tmp) >= int(cfg.BLOCK_SIZE)):
                        # Get BLOCK_SIZE elements from the start of data
                        block = tmp[0:int(cfg.BLOCK_SIZE)]
                        # Append the calculated feature of this block to the
                        # result
                        r.append(f(block))      
                        # Remove the first element from the data  
                        tmp = np.delete(tmp,0)
                    # Append the result to the data
                    data.append(r)
                    # Combine this attribute and function name to a new
                    # attribute
                    attributes.append(attr+'_'+f.__name__)

        # Swap the data axes
        data = np.swapaxes(data,0,1)
        # Create an array of the same length with the label of this bunch
        labels = [bunch.labels[0] for _ in range(len(data))]
        # Empty timestamps
        timestamps = [[None,None] for _ in range(len(data))]

        # Create a new bunch
        new_bunches.append(CustomBunch(data,timestamps=timestamps,
            attributes=attributes,labels=labels,name=bunch.name))
    
    # Save the new created bunches
    print('Files written:')
    for bunch in new_bunches:
        filename = join(cfg.PREPROCESSING_FEATURES_DIR_PATH,bunch.name+'.csv')
        bunch.save_csv(filename)
        print(filename)


if __name__ == "__main__":
    main()
