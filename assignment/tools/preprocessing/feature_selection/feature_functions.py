"""
feature_functions.py

Feature functions

Authors:    Jeroen Veen
            Hugo Arends
Date:       October 2023

Copyright:  2023 HAN University of Applied Sciences. All Rights Reserved.
"""
import sys
from os.path import join, dirname, realpath
sys.path.append(join(dirname(realpath(__file__)), '..', '..'))

import config as cfg
import ctypes
from os.path import join, isfile
import feature_functions_c2dll

FEATURES_DLL = join(cfg.PREPROCESSING_FEATURES_DIR_PATH, 'features.dll')

def check_features_dll():
    """
    Create the feature functions dll as soon as needed
    """
    if not isfile(FEATURES_DLL):
        feature_functions_c2dll.main()

def min(data):
    """
    Python wrapper for the feature calculation functions that are also used on
    the microcontroller. Refer to the C-source files for documentation.
    """
    check_features_dll()
    c_lib = ctypes.CDLL(FEATURES_DLL)

    c_lib.min.restype = ctypes.c_float
    n = len(data)
    x = (ctypes.c_float * n)(*data)
    return c_lib.min(ctypes.byref(x), n)

def max(data):
    """
    Python wrapper for the feature calculation functions that are also used on
    the microcontroller. Refer to the C-source files for documentation.
    """
    check_features_dll()
    c_lib = ctypes.CDLL(FEATURES_DLL)

    c_lib.max.restype = ctypes.c_float
    n = len(data)
    x = (ctypes.c_float * n)(*data)
    return c_lib.max(ctypes.byref(x), n)

def mean(data):
    """
    Python wrapper for the feature calculation functions that are also used on
    the microcontroller. Refer to the C-source files for documentation.
    """
    check_features_dll()
    c_lib = ctypes.CDLL(FEATURES_DLL)

    c_lib.mean.restype = ctypes.c_float
    n = len(data)
    x = (ctypes.c_float * n)(*data)
    return c_lib.mean(ctypes.byref(x), n)

def variance(data):
    """
    Python wrapper for the feature calculation functions that are also used on
    the microcontroller. Refer to the C-source files for documentation.
    """
    check_features_dll()
    c_lib = ctypes.CDLL(FEATURES_DLL)

    c_lib.variance.restype = ctypes.c_float
    n = len(data)
    x = (ctypes.c_float * n)(*data)
    return c_lib.variance(ctypes.byref(x), n)

def energy(data):
    """
    Python wrapper for the feature calculation functions that are also used on
    the microcontroller. Refer to the C-source files for documentation.
    """
    check_features_dll()
    c_lib = ctypes.CDLL(FEATURES_DLL)

    c_lib.energy.restype = ctypes.c_float
    n = len(data)
    x = (ctypes.c_float * n)(*data)
    return c_lib.energy(ctypes.byref(x), n)

def peak_to_peak(data):
    """
    Python wrapper for the feature calculation functions that are also used on
    the microcontroller. Refer to the C-source files for documentation.
    """
    check_features_dll()
    c_lib = ctypes.CDLL(FEATURES_DLL)

    c_lib.peak_to_peak.restype = ctypes.c_float
    n = len(data)
    x = (ctypes.c_float * n)(*data)
    return c_lib.peak_to_peak(ctypes.byref(x), n)

def raw(data, n=None):
    """
    Returns the first raw sample in the array

    Mainly used for testing purpose. With a block_size of 1, this feature should
    return the same raw input data.
    """
    return data[0] if (len(data) > 0) else None
