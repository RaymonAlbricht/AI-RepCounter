"""
visualize_clf.py

Set of functions to visualize supervised machine learning classifiers. 

Authors:    Jeroen Veen
Date:       October 2023

Copyright:  2023 HAN University of Applied Sciences. All Rights Reserved.
"""
import sys
from os.path import join, dirname, realpath
sys.path.append(join(dirname(realpath(__file__)), '..'))

import matplotlib.pyplot as plt
from numpy import array, arange, meshgrid, ceil, sqrt, zeros
from custom_bunch import CustomBunch, check_custom_bunch
from itertools import combinations
from sklearn.tree import DecisionTreeClassifier
from sklearn.metrics import ConfusionMatrixDisplay


cmap = plt.cm.viridis


# @check_custom_bunch
def plot_tree_decision_surfaces(bunch: CustomBunch, clf: DecisionTreeClassifier):
    """
    Plot decision surfaces for all pairs of features in the CustomBunch object and return the figure and axes objects.

    Parameters:
    - bunch: CustomBunch object
    - classifier: sklearn-like classifier with `fit` and `predict` methods.

    Returns
    -------
    fig : matplotlib.figure.Figure
        The main figure object containing the plot.
    axes : array-like of matplotlib.axes._subplots.AxesSubplot
        The axes or collection of axes that the data is plotted on. The exact shape and size 
        depends on the specific plotting routine and parameters used.         
    """

    n_features = bunch.data.shape[1]

    if n_features < 2:
        raise ValueError("Decision tree surfaces can only be computed for multiple attributes!" )
        
    # Create a colormap based on the number of unique labels
    colors = plt.cm.get_cmap(cmap, len(bunch.unique_labels))
    
    pair_idx = list(combinations(range(n_features), 2))
    n_combinations = len(pair_idx)
    nrows = int(sqrt(n_combinations))
    ncols = int(ceil(n_combinations / nrows))
    
    fig, axes = plt.subplots(nrows, ncols, figsize=(4*ncols, 4*nrows))
    
    for (f1, f2), ax in zip(pair_idx, axes.ravel()):
        # Prepare data for current pair of features
        X = bunch.data[:, [f1, f2]]
        
        # Create a mesh grid
        x_min, x_max = X[:, 0].min() - 1, X[:, 0].max() + 1
        y_min, y_max = X[:, 1].min() - 1, X[:, 1].max() + 1
        x_step = (x_max - x_min) / 1000
        y_step = (y_max - y_min) / 1000        
        xx, yy = meshgrid(arange(x_min, x_max, x_step), arange(y_min, y_max, y_step))
        
        # Create a full feature set with dummy values, then replace the columns we're interested in
        Z = zeros((xx.ravel().shape[0], n_features))
        Z[:, f1] = xx.ravel()
        Z[:, f2] = yy.ravel()
        
        # Predict the class labels
        Z_pred = clf.predict(Z)
        Z_pred = bunch.encode_labels(Z_pred)
        Z_pred = Z_pred.reshape(xx.shape)
        
        # Plot the decision boundary
        ax.contourf(xx, yy, Z_pred, alpha=0.8, cmap=cmap)
        # scatter = ax.scatter(X[:, 0], X[:, 1], c=bunch.coded_labels, edgecolors='k', marker='o', cmap=cmap)
        # Plot the data points and create a legend
        handles = []
        for idx, label in enumerate(bunch.unique_labels):
            subset = X[bunch.labels == label]
            handle = ax.scatter(subset[:, 0], subset[:, 1], color=colors(idx), edgecolors='k', marker='o', label=label)
            handles.append(handle)
        ax.set_xlabel(bunch.attributes[f1])
        ax.set_ylabel(bunch.attributes[f2])
        
    # Delete any remaining unused subplots
    for i in range(n_combinations, nrows*ncols):
        fig.delaxes(axes.flatten()[i])
    
    plt.tight_layout()

    return fig, axes     


@check_custom_bunch
def plot_confusion_matrix(bunch: CustomBunch, clf: DecisionTreeClassifier):
    """
    Plot confusion matrix and return the figure and axes objects.

    Parameters:
    - bunch: CustomBunch object
    - classifier: sklearn-like classifier with `fit` and `predict` methods.

    Returns
    -------
    fig : matplotlib.figure.Figure
        The main figure object containing the plot.
    axes : array-like of matplotlib.axes._subplots.AxesSubplot
        The axes or collection of axes that the data is plotted on. The exact shape and size 
        depends on the specific plotting routine and parameters used.         
    """
    fig, axes = plt.subplots()

    y_test = bunch.labels
    y_pred = clf.predict(bunch.data)

    ConfusionMatrixDisplay.from_predictions(y_pred, y_test, display_labels=bunch.unique_labels, cmap=cmap, ax=axes)
    axes.set_title('Confusion matrix')

    plt.tight_layout()

    return fig, axes     
