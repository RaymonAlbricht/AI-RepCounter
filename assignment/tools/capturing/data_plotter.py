"""
data_plotter.py

Data plotting functions.

Authors:    Jeroen Veen
            Hugo Arends
Date:       October 2023

Copyright:  2023 HAN University of Applied Sciences. All Rights Reserved.
"""
import sys
from os.path import join, dirname, realpath
sys.path.append(join(dirname(realpath(__file__)), '..'))

from custom_bunch import CustomBunch
import config as cfg
from glob import glob
import matplotlib.pyplot as plt
from os.path import join

def plot_single(filename=''):
    """
    Load a single bunch from a CSV file and plot it.
    """

    if filename == '':
        filenames = glob(join(cfg.CAPTURED_DIR_PATH, '*.csv'))
        assert len(filenames) != 0, 'No CSV files'

        # Get first CSV file from the folder
        filename = filenames[0]
    else:
        filename = join(cfg.CAPTURED_DIR_PATH, filename)

    print(filename)
    bunch = CustomBunch.load_csv(filename)
    bunch.plot()
    plt.show()


def plot_all():
    """
    Load several bunches from all CSV files in a folder. Add all these bunches
    into a new bunch and plot this new bunch.
    """
    filenames = glob(join(cfg.CAPTURED_DIR_PATH, '*.csv'))
    assert(len(filenames) != 0), 'No CSV files'

    # Add all bunches into one new bunch
    bunch = CustomBunch.load_csv(filenames[0])
    for filename in filenames[1:]:
        bunch = bunch + CustomBunch.load_csv(filename)
    
    # Give the new bunch a more meaningful name
    bunch.name = 'all_raw'

    # Plot the bunch
    bunch.plot()

    # Save plot
    filename = join(cfg.CAPTURED_DIR_PATH, bunch.name+'_time_series_data.png')
    plt.savefig(filename, dpi=300)

    print('Files written:')
    print(filename)

    if __name__ == "__main__":
        plt.show()


if __name__ == "__main__":
    # plot_single('stationary.csv')
    plot_all()
