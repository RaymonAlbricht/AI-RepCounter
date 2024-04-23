"""
build_dtc.py

Decision tree classifier

Authors:    Jeroen Veen
            Hugo Arends
Date:       October 2023

Copyright:  2023 HAN University of Applied Sciences. All Rights Reserved.
"""
import sys
from os.path import join, dirname, realpath
sys.path.append(join(dirname(realpath(__file__)), '..'))

from custom_bunch import CustomBunch, stratified_train_test_split
import config as cfg
from os.path import join
from joblib import dump
from glob import glob
from sklearn.tree import DecisionTreeClassifier
from sklearn import tree
from sklearn.metrics import confusion_matrix
from sklearn.metrics import classification_report
from sklearn.model_selection import cross_val_score
from visualize_clf import plot_confusion_matrix, plot_tree_decision_surfaces
import matplotlib.pyplot as plt

def main():

    filenames = glob(join(cfg.PREPROCESSING_FEATURES_DIR_PATH, '*.csv'))
    assert(len(filenames) != 0), 'No CSV files'

    # Add all bunches into one new bunch
    bunch = CustomBunch.load_csv(filenames[0])
    for filename in filenames[1:]:
        bunch = bunch + CustomBunch.load_csv(filename)
    
    # Give the new bunch a more meaningful name
    bunch.name = 'features'

    (train_bunch, test_bunch) = stratified_train_test_split(bunch)

    # Give the bunches a more meaningful name
    train_bunch.name = 'train'
    test_bunch.name = 'test'

    # import inspect
    # print(inspect.getmembers(train_bunch, lambda a:not(inspect.isroutine(a))))

    # Create and train the decision tree classifier
    dtc = DecisionTreeClassifier()
    dtc.fit(train_bunch.data, train_bunch.labels)
    plot_tree_fig, axes = plt.subplots()    
    tree.plot_tree(dtc, feature_names=train_bunch.data, 
                   class_names=train_bunch.labels, rounded=True, 
                   filled=True, ax=axes, fontsize=10)
    tree_decision_surfaces_fig, _ = plot_tree_decision_surfaces(train_bunch, dtc)
    
    # For advanced users pruning could be interesting
    # https://scikit-learn.org/stable/auto_examples/tree/plot_cost_complexity_pruning.html#sphx-glr-auto-examples-tree-plot-cost-complexity-pruning-py

    # Perform cross-validation, and train classifier multiple times to check 
    # overfitting
    n_splits = 5
    scores = cross_val_score(dtc, train_bunch.data, train_bunch.labels, cv=n_splits)

    # Predict the labels for training data
    train_pred = dtc.predict(train_bunch.data)

    # Predict the labels for test data
    test_pred = dtc.predict(test_bunch.data)
    test_accuracy = dtc.score(test_bunch.data, test_bunch.labels)
    confusion_matrix_fig, _ = plot_confusion_matrix(test_bunch, dtc)

    # Print info
    if __name__ == "__main__":
        print('Decision tree plot:\n')
        print(tree.export_text(dtc,feature_names=train_bunch.attributes))

        print('Training report:\n')
        print(classification_report(train_bunch.labels, train_pred))
        
        print(f'Training accuracy score (cross-validated over {n_splits} splits): ')
        print(scores)
        print(f'Average training accuracy: {scores.mean():.4f} +/- {scores.std():.4f}\n')        

        print('\nTest report:\n')
        print(classification_report(test_bunch.labels, test_pred))

        print(f'Test accuracy score: {test_accuracy:.4f}\n')

        print('Confusion matrix:\n')
        print(confusion_matrix(test_bunch.labels, test_pred))
        
        print()

        plt.show()        
    
    # Create output files
    filename_dump = join(cfg.MODEL_DIR_PATH,"dtc_model.gz")
    filename_txt = join(cfg.MODEL_DIR_PATH,"dtc_model.txt")
    filename_train_bunch = join(cfg.MODEL_DIR_PATH,"dtc_train_bunch.csv")
    filename_test_bunch = join(cfg.MODEL_DIR_PATH,"dtc_test_bunch.csv")

    # Save the bunches
    train_bunch.save_csv(filename_train_bunch)
    test_bunch.save_csv(filename_test_bunch)
    
    # Save the model
    dump(dtc, filename_dump)

    # Save model results in plain text
    textfile = open(filename_txt, 'w')
    textfile.write('Decision tree plot: ')
    textfile.write('\n'.ljust(80, '-') + '\n')
    textfile.write(tree.export_text(dtc,feature_names=train_bunch.attributes))
    textfile.write('\n\n')
    textfile.write('Training report: ')
    textfile.write('\n'.ljust(80, '-') + '\n')
    textfile.write(str(classification_report(train_bunch.labels, train_pred)))
    textfile.write('\n')
    textfile.write(f"Training accuracy scores (cross-validated over {n_splits} splits): ")
    textfile.write(" ".join([str(score) for score in scores]))
    textfile.write('\n')
    textfile.write(f"Average training accuracy: {scores.mean():.4f} +/- {scores.std():.4f}\n")
    textfile.write('\n')
    textfile.write('Test report: ')
    textfile.write('\n'.ljust(80, '-') + '\n')
    textfile.write(str(classification_report(test_bunch.labels, test_pred)))
    textfile.write('\n')
    textfile.write(f'Test accuracy score: {test_accuracy:.4f}\n')
    textfile.write('\n')
    textfile.write('Confusion matrix: ')
    textfile.write('\n'.ljust(80, '-') + '\n')
    textfile.write(str(confusion_matrix(test_bunch.labels, test_pred)))
    textfile.close()


    filename_plot_tree_fig = join(cfg.MODEL_DIR_PATH, bunch.name + '_tree.png')
    plot_tree_fig.savefig(filename_plot_tree_fig, dpi=600)
     
    filename_confusion_matrix = join(cfg.MODEL_DIR_PATH, bunch.name + '_confusion_matrix.png')
    confusion_matrix_fig.savefig(filename_confusion_matrix, dpi=300)

    filename_decision_surfaces = join(cfg.MODEL_DIR_PATH, bunch.name + '_decision_surfaces.png')
    tree_decision_surfaces_fig.savefig(filename_decision_surfaces, dpi=300)

    print('Files written:')
    print(filename_dump)
    print(filename_txt)
    print(filename_train_bunch)
    print(filename_test_bunch)
    print(filename_plot_tree_fig)
    print(filename_decision_surfaces)
    print(filename_confusion_matrix)


if __name__ == "__main__":
    main()
