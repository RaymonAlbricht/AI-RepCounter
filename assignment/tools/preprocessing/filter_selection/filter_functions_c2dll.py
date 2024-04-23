"""
filter_functions_c2dll.py

Compiles the filter functions source files to a dll.

Authors:    Jeroen Veen
            Hugo Arends
Date:       March 2024

Copyright:  2024 HAN University of Applied Sciences. All Rights Reserved.
"""
import sys
from os.path import join, dirname, realpath
sys.path.append(join(dirname(realpath(__file__)), '..', '..'))

import config as cfg

from sys import platform
from os.path import join, exists
from os import makedirs
from distutils.ccompiler import new_compiler
from shutil import copyfile, rmtree

# TODO The list of filter functions that are implemented in filters.c.
FUNCTIONS_IN_C_FILE = ['fir']

# Set to False if you would like to examine the temporary files that are
# created.
DELETE_TEMPORARY_FILES = True

MAIN_FILE_STR = \
'''
#include "filters.h"

int main(void)
{
    return 0;
}
'''

def main():

    PROJECT_DIR = join(cfg.PREPROCESSING_FILTERS_DIR_PATH, 'c2dll_project')
    if not exists(PROJECT_DIR):
        makedirs(PROJECT_DIR)

    print('Creating project for buidling in ' + PROJECT_DIR)

    # Create a simple main application, because a main entry point is required
    # by the linker.
    main_file = open(join(PROJECT_DIR,'main.c'), 'w')
    main_file.write(MAIN_FILE_STR)
    main_file.close()
    
    # Copy the source files
    FILTERS_SOURCE_DIR_PATH = join(cfg.DATA_DIR_PATH, '..', '..', 'lib')
    copyfile(join(FILTERS_SOURCE_DIR_PATH,'filters.h'), 
        join(PROJECT_DIR,'filters.h'))
    copyfile(join(FILTERS_SOURCE_DIR_PATH,'filters.c'),
        join(PROJECT_DIR,'filters.c'))

    # Compile and link the project
    cc = new_compiler(force=1)

    output_libname = cc.library_filename(
        'filters', lib_type='shared', 
        output_dir=cfg.PREPROCESSING_FILTERS_DIR_PATH)

    if platform.startswith('win'):
        libraries = None
        cc_args = None
    else:
        # Math library
        libraries = ["m"]
        cc_args = ["-std=c99"]

    objects = cc.compile(
        sources=[join(PROJECT_DIR,'main.c'),join(PROJECT_DIR,'filters.c')],
        extra_preargs=cc_args,
        output_dir=join(PROJECT_DIR,'build'))

    cc.link("shared_library", objects,
        output_filename=output_libname, 
        libraries=libraries,
        export_symbols=FUNCTIONS_IN_C_FILE)  

    if DELETE_TEMPORARY_FILES:
        print('Removing temporary files ' + PROJECT_DIR)
        rmtree(PROJECT_DIR)
    else:
        print('Temporary files have been preserved in ' + PROJECT_DIR)

    print('Functions compiled:')
    print('\n'.join(str("  "+x+"()") for x in FUNCTIONS_IN_C_FILE))
    print('Generated ' + output_libname)


if __name__ == "__main__":
    main()
