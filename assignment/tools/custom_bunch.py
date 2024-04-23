"""
custom_bunch.py

Custom supervised machine learning data type and support functions.

Authors:    Jeroen Veen
            Hugo Arends
Date:       October 2023

Copyright:  2023 HAN University of Applied Sciences. All Rights Reserved.
"""
from sklearn.utils import Bunch
from sklearn.model_selection import train_test_split
from numpy import array, unique, bincount, concatenate, swapaxes, corrcoef
from pandas import DataFrame
from functools import wraps
import unittest
from joblib import dump, load
import matplotlib.pyplot as plt
from os.path import split
import seaborn as sns


class CustomBunch(Bunch):
    """
    CustomBunch is an extension of sklearn's Bunch object.
    
    This object is intended to hold data arrays alongside meta-data attributes 
    like attribute names, class names, labels, unique targets, and relation.
    
    Parameters
    ----------     
    data : array, shape (n_samples, n_attributes)
        Data array.
    timestamps : array, shape (n_samples, 2)
        A list of two timestamps corresponding to the data.
    attributes : list of str
        Names of the attributes.
    labels : array, shape (n_samples, )
        Labels corresponding to the data.
    name : str
        Descriptive string or identifier.
    
    Attributes
    ----------
    data : array, shape (n_samples, n_attributes)
    timestamps : array, shape (n_samples, 2)
    attributes : list of str
    labels : array, shape (n_samples, )
    unique_labels : list of str
        Names of the labels that are in the data.
    coded_labels : array, shape (n_samples, )
        Labels encoded with value between 0 and number of unique labels - 1.
    name : str
    """    
    def __init__(self, data: array, timestamps: array, attributes: list, labels: array, name: str = ""):

        data = array(data)
        timestamps = array(timestamps)
        unique_labels = sorted(list(unique(labels)))        
        labels_dict = {class_:i for i, class_ in enumerate(unique_labels)}
        coded_labels = [labels_dict[label] for label in labels]       

        super().__init__(data=data, timestamps=timestamps, labels=labels, attributes=attributes,
                         unique_labels=unique_labels, labels_dict=labels_dict, coded_labels=coded_labels, name=name)
        
    def encode_labels(self, labels:list):
        """
        Encode labels according to the custombunch codebook, ie labels_dict
        
        Parameters
        ----------
        labels : array, shape (n_samples, )
        Labels array

        Returns:
        --------
        array, shape (n_samples, )
            coded labels
        """
        return array([self.labels_dict[label] for label in labels])

    def select_attributes(self, selected_attrs):
        """Selects parts of the CustomBunch according to given attribute names."""
        if type(selected_attrs) != list or len(selected_attrs) < 1 or type(selected_attrs[0]) != str:
            raise ValueError("Selected attribute names should be a list of strings!")
        
        indices = [self.attributes.index(attr) for attr in selected_attrs]
        sliced_data = self.data[:, indices]
        
        # Create a new CustomBunch with the sliced data and updated attributes
        return CustomBunch(data=sliced_data, 
                           timestamps=self.timestamps,
                           attributes=selected_attrs, 
                           labels=self.labels)    
    
    def to_dataframe(self):
        """
        Convert the CustomBunch object to a Pandas DataFrame.
        
        Returns:
        --------
        DataFrame
            A DataFrame representation of the CustomBunch.
        """
        df = DataFrame(self.data, columns=self.attributes)
        df['labels'] = self.labels
        df['timestamp1'] = self.timestamps[:, 0]
        df['timestamp2'] = self.timestamps[:, 1]
        return df
    
    def __add__(self, o):      
        """
        Add two CustomBunches.
        Requires that both bunches have the same attributes.
        """
        assert self.attributes == o.attributes, 'attributes must be the same'

        data = []
        
        for attr in self.attributes:
            sliced_data_self = self.data[:, self.attributes.index(attr)]
            sliced_data_o = o.data[:, o.attributes.index(attr)]
            data.append(concatenate((sliced_data_self, sliced_data_o)))
        
        data = swapaxes(data,0,1)
        labels = self.labels + o.labels
        timestamps = concatenate((self.timestamps, o.timestamps))
        name=self.name + '_' + o.name

        return CustomBunch(data,
                           timestamps=timestamps,
                           attributes=self.attributes,
                           labels=labels,
                           name=name)

    def plot(self, fmt=None):
        """
        Plots the bunch. A subplot is created for each label. Within each subplot the
        attribute data is plotted.

        Returns
        -------
        fig : matplotlib.figure.Figure
            The main figure object containing the plot.
        axes : array-like of matplotlib.axes._subplots.AxesSubplot
            The axes or collection of axes that the data is plotted on. The exact shape and size 
            depends on the specific plotting routine and parameters used.          
        """
        # Create as many subplots as there are labels in this bunch
        fig, axes = plt.subplots(len(self.unique_labels), sharey=True, sharex=True)
        fig.suptitle(self.name)

        # Loop all labels
        for x, label in enumerate(self.unique_labels):
            ax = axes
            if len(self.unique_labels) > 1:
                ax = axes[x]

            for attr in self.attributes:
                index = self.attributes.index(attr)
                sliced_data = self.data[:, index]
                current_label_data = []
                for i, l in enumerate(self.labels):
                    if l == label:
                        current_label_data.append(sliced_data[i])
                sns.lineplot(data=current_label_data, linestyle=fmt,ax=ax,
                             label=self.attributes[index])
                
                # Legacy code to produce line plot using matplotlib
                # In the arument list, fmt should be set to fmt="" instead of fmt=None
                # ax.plot(current_label_data, fmt, label=self.attributes[index])

            ax.set_title(label)
            ax.legend()

        plt.tight_layout()
        return fig, axes
    
    def plot_label_distribution(self):
        """
        Show label distribution and return the figure and axes objects.

        Returns
        -------
        fig : matplotlib.figure.Figure
            The main figure object containing the plot.
        axes : array-like of matplotlib.axes._subplots.AxesSubplot
            The axes or collection of axes that the data is plotted on. The exact shape and size 
            depends on the specific plotting routine and parameters used.          
        """
        df = DataFrame(self.data, columns=self.attributes)
        df['labels'] = self.labels
       
        fig, axes = plt.subplots()
        sns.countplot(data=df,  x='labels', ax=axes, )
        axes.set_title(self.name + " label distribution")
        plt.tight_layout()

        # Legacy code to produce countplot using matplotlib
        # nr_of_labels = len(bunch.unique_labels)
        # color = cmap(0.5)
        # fig, axes = plt.subplots(1, 1)
        # axes.hist(bunch.coded_labels, bins=range(0, nr_of_labels + 1), align='left', rwidth=0.8, color=color, alpha=0.7)
        # axes.set_xticks(range(0, nr_of_labels))
        # axes.set_xticklabels(bunch.unique_labels)
        # axes.set_ylabel('count')

        return fig, axes   

    def plot_pairs(self):
        """
        Show scatterplots of paired attributes and returns a PairGrid object.

        Returns
        -------
        grid : :class: `PairGrid`
            Returns the underlying :class:`PairGrid` instance for further tweaking.
        """
        df = DataFrame(self.data, columns=self.attributes)
        df['labels'] = self.labels
        
        pair_grid = sns.pairplot(df, hue='labels')
        pair_grid.fig.suptitle(self.name + " pair plot")

        # Legacy code to produce pairplots using matplotlib
        # nr_of_labels = len(bunch.unique_labels)
        # fig, axes = plt.subplots(nr_of_selected_attributes, nr_of_selected_attributes)
        # for i, sel_a in enumerate(selected_attributes):
        #     index_a = bunch.attributes.index(sel_a)
        #     for j, sel_b in enumerate(selected_attributes):
        #         index_b = bunch.attributes.index(sel_b)
        #         if i == j:
        #             # Diagonal - create histogram
        #             axes[i, j].hist(bunch.data[:, index_a], bins=30, color=cmap(0.5), alpha=0.7)
        #         else:
        #             # Off-diagonal - create scatter plot
        #             sc = axes[i, j].scatter(bunch.data[:, index_b], bunch.data[:, index_a], c=bunch.coded_labels, cmap=cmap, s=10)
                
        #         # If on the edge, label the axes
        #         if i == nr_of_selected_attributes - 1:
        #             axes[i, j].set_xlabel(sel_b)
        #         if j == 0:
        #             axes[i, j].set_ylabel(sel_a)
        #         if j > i:
        #             axes[i, j].set_visible(False)

        # # Add colorbar
        # cbar = plt.colorbar(sc, ax=axes.ravel().tolist(), boundaries=arange(-.5, nr_of_labels +.5), ticks=range(0, nr_of_labels))
        # cbar.set_ticklabels(bunch.unique_labels, rotation=90, verticalalignment='center')

        return pair_grid
    
    def plot_correlation_heatmap(self):
        """
        Show heatmap of attribute correlations and return the figure and axes objects.

        Returns
        -------
        fig : matplotlib.figure.Figure
            The main figure object containing the plot.
        axes : array-like of matplotlib.axes._subplots.AxesSubplot
            The axes or collection of axes that the data is plotted on. The exact shape and size 
            depends on the specific plotting routine and parameters used.          
        """
        if len(self.attributes) == 1:
            raise ValueError("Correlations can only be computed for multiple attributes!" )

        corr = corrcoef(self.data, rowvar=False)

        fig, axes = plt.subplots()
        sns.heatmap(corr, ax=axes, annot=True, xticklabels=self.attributes, yticklabels=self.attributes)
        axes.set_title(self.name + " correlation heatmap")

        plt.tight_layout()

        return fig, axes 
    
    def plot_boxplots(self):
        """
        Show boxplots of attributes and return the figure and axes objects.

        Returns
        -------
        fig : matplotlib.figure.Figure
            The main figure object containing the plot.
        axes : array-like of matplotlib.axes._subplots.AxesSubplot
            The axes or collection of axes that the data is plotted on. The exact shape and size 
            depends on the specific plotting routine and parameters used.          
        """
        df = DataFrame(self.data, columns=self.attributes)
        df['labels'] = self.labels
        n_attributes = len(self.attributes)

        figsize_per_plot=(4, 4)
        fig, axes = plt.subplots(1, n_attributes, figsize=(figsize_per_plot[0] * n_attributes, figsize_per_plot[1]))

        if n_attributes == 1:
            axes = [axes]

        for ax, attribute in zip(axes, self.attributes):
            sns.boxplot(x='labels', y=attribute, data=df, ax=ax)
            ax.set_title(f'Boxplot for {attribute}')

        plt.tight_layout()
        
        return fig, axes     



    def save_csv(self, file=None):
        if file == None:
            file = self.name + '.csv'
        assert str(file).endswith('csv'), 'file should end with .csv'
        f = open(file, 'w')
        f.write(self.export_csv())
        f.close()

    @staticmethod
    def load_csv(file):
        assert str(file).endswith('csv'), 'file should end with .csv'
        
        f = open(file, 'r')
        lines = f.readlines()
        f.close()

        # Get the name for this bunch from the filename
        name = split(file)[-1][:-4]

        # First line contains, amongst others, the attributes
        label, ts1, ts2, *attrs = lines[0].rstrip().split(',')

        labels = []
        timestamps = []
        data = []
        attributes = attrs

        # Rest of the lines
        for line in lines[1:]:
            label, ts1, ts2, *attrs = line.rstrip().split(',')
            
            labels.append(label)
            timestamps.append([None if ts1=='' else int(ts1),None if ts2=='' else int(ts2)])
            data.append([float(d) for d in attrs])

        return CustomBunch(data=data,
                        timestamps=timestamps,
                        attributes=attributes,
                        labels=labels,
                        name=name)

    def save_blob(self, file=None):
        if file == None:
            file = self.name + '.gz'
        assert str(file).endswith('.gz'), 'file should end with .gz'
        dump(self, str(file))        

    @staticmethod
    def load_blob(file):
        assert str(file).endswith('.gz'), 'file should end with .gz'
        return load(str(file))
    
    def print_summary(self):
        print(f"\nDataset: {self.name}, {len(self.unique_labels)} unique labels, {len(self.attributes)} attributes, {len(self.data)} samples")
        print(f"\nUnique labels: \n{self.unique_labels}")
        print(f"\nAttributes: \n{self.attributes}")
        print("\nHead:")
        print(self.to_dataframe().head())

    def export_csv(self):

        sliced_timestamps_1 = self.timestamps[:, 0]
        sliced_timestamps_2 = self.timestamps[:, 1]

        df = DataFrame(self.data, columns=self.attributes)
        df.insert(0, 'label', self.labels)
        df.insert(1, 'timestamp1', sliced_timestamps_1)
        df.insert(2, 'timestamp2', sliced_timestamps_2)
        return df.to_csv(index=False, lineterminator='\n')


def check_custom_bunch(func):
    @wraps(func)
    def wrapper(bunch, *args, **kwargs):
        if not isinstance(bunch, CustomBunch):
            raise TypeError(f"Expected CustomBunch, got {type(bunch)}")

        # Get attributes of CustomBunch class, excluding inherited ones from Bunch
        expected_attrs = [k for k, v in vars(CustomBunch).items() if not callable(v) and not k.startswith("__")]

        for attr in expected_attrs:
            if not hasattr(bunch, attr):
                raise AttributeError(f"Expected attribute {attr} not found in CustomBunch")

        return func(bunch, *args, **kwargs)
    return wrapper

def stratified_train_test_split(bunch: CustomBunch, test_size: float = 0.25, random_state: int = None) -> (CustomBunch, CustomBunch):
    """
    Splits the data in a CustomBunch into a training and a test set using stratified sampling.
    
    Parameters
    ----------
    bunch : CustomBunch
        The input dataset to split.
    test_size : float, optional (default=0.25)
        Proportion of the dataset to include in the test split.
    random_state : int, optional
        Random state seed for reproducibility.

    Returns
    -------
    (train_bunch, test_bunch) : tuple of CustomBunch
        The training and test datasets as CustomBunch objects.
    """

    # Split data using stratified sampling
    data_train, data_test, labels_train, labels_test = train_test_split(
        bunch.data, bunch.labels, test_size=test_size, stratify=bunch.labels, random_state=random_state)

    # Create CustomBunch objects for train and test sets
    timestamps = [[None,None] for _ in range(len(data_train))]
    train_bunch = CustomBunch(data=data_train, attributes=bunch.attributes, labels=labels_train, timestamps=timestamps)

    timestamps = [[None,None] for _ in range(len(data_test))]
    test_bunch = CustomBunch(data=data_test, attributes=bunch.attributes, labels=labels_test, timestamps=timestamps)

    return train_bunch, test_bunch

class TestCustomBunchDecorator(unittest.TestCase):

    def setUp(self):
        # Create a sample CustomBunch for testing
        data = array([[1, 2], [3, 4], [5, 6], [7, 8], [9, 10], [11, 12]])
        attributes = ["attr1", "attr2"]
        labels = array([0, 0, 1, 1, 2, 2])
        self.bunch = CustomBunch(data=data, attributes=attributes, timestamps=None, labels=labels, name='complete')
        self.incomplete_bunch = CustomBunch(data=data, attributes=attributes, timestamps=None, labels=labels, name='incomplete')
        del self.incomplete_bunch.relation  # intentionally remove an attribute

    @check_custom_bunch
    def dummy_function(self, bunch):
        return True  # A dummy function to simulate usage with the decorator

    def test_decorator_with_wrong_type(self):
        with self.assertRaises(TypeError):
            self.dummy_function([1, 2, 3])  # Pass a list instead of a CustomBunch

    def test_decorator_with_missing_attribute(self):
        with self.assertRaises(AttributeError):
            self.dummy_function(self.incomplete_bunch)  # Pass the incomplete CustomBunch

class TestStratifiedTrainTestSplit(unittest.TestCase):

    def setUp(self):
        # Create a sample CustomBunch for testing
        data = array([[1, 2], [3, 4], [5, 6], [7, 8], [9, 10], [11, 12]])
        labels = array([0, 0, 1, 1, 2, 2])
        attributes = ["attr1", "attr2"]
        self.bunch = CustomBunch(data=data, attributes=attributes, timestamps=None, labels=labels)

    def test_split_shape(self):
        train_bunch, test_bunch = stratified_train_test_split(self.bunch, test_size=0.5, random_state=42)

        # Check if data splits are of correct shape
        self.assertEqual(train_bunch.data.shape, (3, 2))
        self.assertEqual(test_bunch.data.shape, (3, 2))

    def test_stratification(self):
        train_bunch, test_bunch = stratified_train_test_split(self.bunch, test_size=0.5, random_state=42)
        
        # Check if labels are stratified
        train_counts = bincount(train_bunch.labels)
        test_counts = bincount(test_bunch.labels)
        
        for count in train_counts:
            self.assertEqual(count, 1)
        
        for count in test_counts:
            self.assertEqual(count, 1)


if __name__ == '__main__':
    # unittest.main()
    import test
    test.main()