"""
normalization_functions.py

Normalization functions

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
import normalization_functions_c2dll

NORMALIZATIONS_DLL = join(cfg.PREPROCESSING_NORMALIZATIONS_DIR_PATH, 'normalizations.dll')

def check_normalizations_dll():
    """
    Create the feature functions dll as soon as needed
    """
    if not isfile(NORMALIZATIONS_DLL):
        check_normalizations_dll.main()

def rescale(data, from_, to_):
    """
    Python wrapper for the normalization calculation functions that are also
    used on the microcontroller. Refer to the C-source files for documentation.
    """
    check_normalizations_dll()
    c_lib = ctypes.CDLL(NORMALIZATIONS_DLL)

    c_lib.rescale.restype = ctypes.c_float
    f_ = (ctypes.c_float * 2)(*from_)
    t_ = (ctypes.c_float * 2)(*to_)
    return c_lib.rescale((ctypes.c_float)(data), ctypes.byref(f_), ctypes.byref(t_))

def clip(data, min, max):
    """
    Python wrapper for the normalization calculation functions that are also
    used on the microcontroller. Refer to the C-source files for documentation.
    """
    check_normalizations_dll()
    c_lib = ctypes.CDLL(NORMALIZATIONS_DLL)

    c_lib.clip.restype = ctypes.c_float
    min_ = (ctypes.c_float * 1)(*min)
    max_ = (ctypes.c_float * 1)(*max)
    return c_lib.clip((ctypes.c_float)(data), ctypes.byref(min_), ctypes.byref(max_))
