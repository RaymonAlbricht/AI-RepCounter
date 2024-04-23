"""
normalization_calculator.py

Calculates normalizations

Authors:    Jeroen Veen
            Hugo Arends
Date:       March 2024

Copyright:  2024 HAN University of Applied Sciences. All Rights Reserved.
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
import normalization_functions as nf


# TODO Set normalization function here
NORMALIZATION_FUNCTIONS = [nf.rescale]
#NORMALIZATION_FUNCTIONS = nf.rescale,nf.clip

ARGS = [
    [
        # Rescale
        [-1000, 1000], # From
        [-1, 1]        # To
    ],
    # [
    #     # Clip
    #     [0],    # Min
    #     [1000]  # Max
    # ],
]

# TODO Set input file path for calculation normalizations
#INPUT_DIR_PATH = cfg.CAPTURED_DIR_PATH
INPUT_DIR_PATH = cfg.PREPROCESSING_FILTERS_DIR_PATH


assert len(NORMALIZATION_FUNCTIONS) == len(ARGS), 'Number of NORMALIZATION_FUNCTIONS and ARGS must be equal'

def main():

    if __name__ == "__main__":
        print('NORMALIZATION_FUNCTIONS: ' +
            str([f.__name__ for f in NORMALIZATION_FUNCTIONS]))

    # Get paths of all captured data files
    filenames = glob(join(INPUT_DIR_PATH, '*.csv'))

    assert len(filenames) != 0, 'No CSV files'

    # Load captured data in a list of bunches
    bunches = [CustomBunch.load_csv(f) for f in filenames]
    new_bunches = []

    # Loop all bunches
    for bunch in bunches:

        # Filters can only be calculated of data consisting of a single label
        assert len(bunch.unique_labels) == 1, \
            'Expected a bunch with a unique label'

        data = []
        labels = []
        timestamps = [] # bunch.timestamps for preserving timestamps
        attributes = []

        # Loop all attributes of this bunch
        for attr in bunch.attributes:
            # Get a slice containing this attributes data
            d = bunch.data[:, bunch.attributes.index(attr)]

            # Loop all filter functions
            for f, arg in zip(NORMALIZATION_FUNCTIONS, ARGS): 
                r = []
                for val in d:
                    r.append(f(val, arg[0], arg[1]))    
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
        filename = join(cfg.PREPROCESSING_NORMALIZATIONS_DIR_PATH,bunch.name+'.csv')
        bunch.save_csv(filename)
        print(filename)


if __name__ == "__main__":
    main()
