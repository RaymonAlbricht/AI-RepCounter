"""
config.py

Configuration file

Authors:    Jeroen Veen
            Hugo Arends
Date:       October 2023

Copyright:  2023 HAN University of Applied Sciences. All Rights Reserved.
"""
from os.path import join, dirname, realpath, exists
from os import makedirs

# TODO Run this config script to generate the required data folders.

# TODO Set serial communication settings.
COMPORT = 'COM3'
BAUDRATE = 115200

# TODO Set feature calculation parameters
#      Valid BLOCK_TYPE values are: BLOCK or SLIDING
BLOCK_SIZE = 100
BLOCK_TYPE = 'BLOCK'


# Directory paths. The data directory is located relative to this file
# config.py.
DATA_DIR_PATH = join(dirname(realpath(__file__)), 'data')
CAPTURED_DIR_PATH = join(DATA_DIR_PATH, 'captured')
PREPROCESSING_DIR_PATH = join(DATA_DIR_PATH, 'preprocessed')
PREPROCESSING_FEATURES_DIR_PATH = join(PREPROCESSING_DIR_PATH, 'features')
PREPROCESSING_FILTERS_DIR_PATH = join(PREPROCESSING_DIR_PATH, 'filters')
PREPROCESSING_NORMALIZATIONS_DIR_PATH = join(PREPROCESSING_DIR_PATH, 'normalizations')
MODEL_DIR_PATH = join(DATA_DIR_PATH, 'model')
MODEL_EMBEDDING_DIR_PATH = join(DATA_DIR_PATH, 'model_embedding')

# Create a list of all paths
paths = [
    DATA_DIR_PATH, 
    CAPTURED_DIR_PATH,
    PREPROCESSING_DIR_PATH,
    PREPROCESSING_FEATURES_DIR_PATH,
    PREPROCESSING_FILTERS_DIR_PATH,
    PREPROCESSING_NORMALIZATIONS_DIR_PATH,
    MODEL_DIR_PATH,
    MODEL_EMBEDDING_DIR_PATH,
]

def create_data_folders():
    """
    Creates the data folders for the project if they do not exist.
    """
    for path in paths:
        if not exists(path):
            makedirs(path)
            print('Created folder '+str(path))

if __name__ == "__main__":
    create_data_folders()
