"""
build_svm.py

Support Vector Machine classifier

Authors:    Jeroen Veen
            Hugo Arends
Date:       Februari 2024

Copyright:  2024 HAN University of Applied Sciences. All Rights Reserved.
"""
import sys
from os.path import join, dirname, realpath
sys.path.append(join(dirname(realpath(__file__)), '..'))

from custom_bunch import CustomBunch, stratified_train_test_split
import config as cfg
from os.path import join
from joblib import dump
from glob import glob
from sklearn import svm
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

    # Create and train the support vector classifier
    clf = svm.SVC(kernel='linear', gamma=0.001)
    clf.fit(train_bunch.data, train_bunch.labels)
    
    # perform cross-validation, and train classifier multiple times to check overfitting
    n_splits = 5
    scores = cross_val_score(clf, train_bunch.data, train_bunch.labels, cv=n_splits)

    # Predict the labels for training data
    train_pred = clf.predict(train_bunch.data)

    # Predict the labels for test data
    test_pred = clf.predict(test_bunch.data)
    test_accuracy = clf.score(test_bunch.data, test_bunch.labels)
    confusion_matrix_fig, _ = plot_confusion_matrix(test_bunch, clf)

    from micromlgen import port
    print(port(clf))
    exit()

    # Print info
    if __name__ == "__main__":

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
    filename_dump = join(cfg.MODEL_DIR_PATH,"clf_model.gz")
    filename_txt = join(cfg.MODEL_DIR_PATH,"clf_model.txt")
    filename_train_bunch = join(cfg.MODEL_DIR_PATH,"clf_train_bunch.csv")
    filename_test_bunch = join(cfg.MODEL_DIR_PATH,"clf_test_bunch.csv")

    # Save the bunches
    train_bunch.save_csv(filename_train_bunch)
    test_bunch.save_csv(filename_test_bunch)
    
    # Save the model
    dump(clf, filename_dump)

    # Save model results in text
    textfile = open(filename_txt, 'w')
    textfile.write('CLF function: \n'.ljust(80, '-'))
    textfile.write('\n')
    textfile.write(export_text(clf,feature_names=train_bunch.attributes))
    textfile.write('\n\n')
    textfile.write('Training report: \n'.ljust(80, '-'))
    textfile.write('\n')
    textfile.write(str(classification_report(train_bunch.labels, train_pred)))
    textfile.write('\n')
    textfile.write(f"Training accuracy scores (cross-validated over {n_splits} splits): ")
    textfile.write(" ".join([str(score) for score in scores]))
    textfile.write('\n')
    textfile.write(f"Average training accuracy: {scores.mean():.4f} +/- {scores.std():.4f}\n")
    textfile.write('\n')
    textfile.write('Test report: '.ljust(80, '-'))
    textfile.write('\n')
    textfile.write(str(classification_report(test_bunch.labels, test_pred)))
    textfile.write('\n')
    textfile.write(f'Test accuracy score: {test_accuracy:.4f}\n')
    textfile.write('\n')
    textfile.write('Confusion matrix: \n'.ljust(80, '-'))
    textfile.write('\n')
    textfile.write(str(confusion_matrix(test_bunch.labels, test_pred)))
    textfile.close()

    filename_confusion_matrix = join(cfg.MODEL_DIR_PATH, bunch.name + '_confusion_matrix.png')
    confusion_matrix_fig.savefig(filename_confusion_matrix, dpi=300)

    print('Files written:')
    print(filename_dump)
    print(filename_txt)
    print(filename_train_bunch)
    print(filename_test_bunch)
    print(filename_confusion_matrix)


if __name__ == "__main__":
    main()
