"""
filter_functions.py

Filter functions

Authors:    Jeroen Veen
            Hugo Arends
Date:       March 2024

Copyright:  2024 HAN University of Applied Sciences. All Rights Reserved.
"""
import sys
from os.path import join, dirname, realpath
sys.path.append(join(dirname(realpath(__file__)), '..', '..'))

import config as cfg
import ctypes
from os.path import join, isfile
import filter_functions_c2dll

FILTERS_DLL = join(cfg.PREPROCESSING_FILTERS_DIR_PATH, 'filters.dll')

def check_filters_dll():
    """
    Create the feature functions dll as soon as needed
    """
    if not isfile(FILTERS_DLL):
        filter_functions_c2dll.main()

def fir(data, coefs, x):
    """
    Python wrapper for the filter calculation functions that are also used on
    the microcontroller. Refer to the C-source files for documentation.
    """
    check_filters_dll()
    c_lib = ctypes.CDLL(FILTERS_DLL)

    c_lib.fir.restype = ctypes.c_float
    n = len(coefs)
    c = (ctypes.c_float * n)(*coefs)
    return c_lib.fir(ctypes.c_float(data), ctypes.byref(c), ctypes.byref(x), n)

def raw(data, coefs, x):
    """
    Returns the data

    Mainly used for comparing the filtered data to the raw data.
    """
    return data
