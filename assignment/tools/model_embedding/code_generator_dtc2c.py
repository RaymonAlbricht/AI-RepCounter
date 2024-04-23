"""
code_generator_dtc2c.py

Generate C code from a decision tree model

Authors:    Hugo Arends
            Jeroen Veen
Date:       October 2023

Copyright:  2023 HAN University of Applied Sciences. All Rights Reserved.
"""
import sys
from os.path import join, dirname, realpath
sys.path.append(join(dirname(realpath(__file__)), '..'))

import config as cfg
from custom_bunch import CustomBunch
import joblib
import numpy as np
from os.path import join, exists
from os import makedirs
from sklearn.tree import _tree
from sklearn.tree import DecisionTreeClassifier

export_str = ""

def main():

    filename_train_bunch = join(cfg.MODEL_DIR_PATH,"dtc_train_bunch.csv")
    filename_dtc = join(cfg.MODEL_DIR_PATH,"dtc_model.gz")

    dtc = joblib.load(filename_dtc)
    bunch = CustomBunch.load_csv(filename_train_bunch)
    
    # Adapted from the original tree export_text function
    # https://github.com/scikit-learn/scikit-learn/blob/main/sklearn/tree/_export.py

    tree_ = dtc.tree_
    spacing = 4
    decimals = 6
    max_depth = 10
    show_weights = False

    feature_names_ = [bunch.attributes[i] if i != _tree.TREE_UNDEFINED else
        None for i in tree_.feature]

    right_child_fmt = "{}if({} <= {}f)\n"
    left_child_fmt = "{}else // {} > {}f\n"
    truncation_fmt = "{} {}\n"
    value_fmt = "{}{}{}\n"

    global export_str

    export_str = ""

    def _compute_depth(tree, node):
        """
        Returns the depth of the subtree rooted in node.
        """

        def compute_depth_(
            current_node, current_depth, children_left, children_right, depths
        ):
            depths += [current_depth]
            left = children_left[current_node]
            right = children_right[current_node]
            
            if left != -1 and right != -1:
                compute_depth_(left, current_depth + 1, children_left, 
                    children_right, depths)
                compute_depth_(right, current_depth + 1, children_left,
                    children_right, depths)

        depths = []
        compute_depth_(node, 1, tree.children_left, tree.children_right, depths)
        return max(depths)

    def _add_leaf(value, class_name, indent):
        global export_str
        val = ""
        is_classification = isinstance(dtc, DecisionTreeClassifier)
        if show_weights or not is_classification:
            val = ["{1:.{0}f}, ".format(decimals, v) for v in value]
            val = "[" + "".join(val)[:-2] + "]"
        if is_classification:
            # val += " ret = " + str(np.argmax(value)) + "; // " + str(class_name)
            val += " ret = " + str(class_name) + ";"
        export_str += value_fmt.format(indent, "", val)

    def print_tree_recurse(node, depth):
        global export_str
        indent = (" " * spacing) * depth

        value = None
        if tree_.n_outputs == 1:
            value = tree_.value[node][0]
        else:
            value = tree_.value[node].T[0]
        class_name = np.argmax(value)

        if tree_.n_classes[0] != 1 and tree_.n_outputs == 1:
            class_name = dtc.classes_[class_name]

        if depth <= max_depth + 1:
            if tree_.feature[node] != _tree.TREE_UNDEFINED:
                name = feature_names_[node]
                threshold = tree_.threshold[node]
                threshold = "{1:.{0}f}".format(decimals, threshold)
                export_str += (right_child_fmt.format(indent, name, threshold))
                export_str += '{}{{\n'.format(indent)
                print_tree_recurse(tree_.children_left[node], depth + 1)
                export_str += '{}}}\n'.format(indent)

                export_str += (left_child_fmt.format(indent, name, threshold))
                export_str += '{}{{\n'.format(indent)
                print_tree_recurse(tree_.children_right[node], depth + 1)
                export_str += '{}}}\n'.format(indent)
            else:  # leaf
                _add_leaf(value, class_name, indent)
        else:
            subtree_depth = _compute_depth(tree_, node)
            if subtree_depth == 1:
                _add_leaf(value, class_name, indent)
            else:
                trunc_report = "#warning truncated branch of depth %d" % \
                    subtree_depth
                export_str += (truncation_fmt.format(indent, trunc_report))

        return export_str

    # Build source file

    # Create enumerated type of all labels
    typedef_str = \
        'typedef enum\n' \
        '{\n'
    for x, label in enumerate(dtc.classes_):
        typedef_str += '{}{} = {},\n'.format(" " * spacing, label, x)
    typedef_str += \
        '}dtc_t;\n\n'

    # Create function documentation
    comment_str = \
        '/*\n' \
        ' * \\brief Decision tree classifier\n' \
        ' * \n' \
        ' * Decision tree classifier based on the following input characteristics:\n' \
        ' *   BLOCK_SIZE: ' + str(cfg.BLOCK_SIZE) + '\n' \
        ' *   BLOCK_TYPE: ' + str(cfg.BLOCK_TYPE) + '\n' \
        ' * \n' \
        ' * \\return dtc_t\n'
    for x, label in enumerate(dtc.classes_):
        comment_str += ' *   ' + str(x) + '  ' + label + '\n'
    comment_str += \
        ' */\n'
               
    # Create an argument for each feature
    args = ', '.join(['const float ' + str(f) for f in [i for i in set(feature_names_) if i is not None]])

    # Create the function start
    function_open_str = \
        'dtc_t dtc(' +  args + ')\n' \
        '{\n'
    function_open_str += \
        '{}dtc_t ret;\n\n'.format(" " * spacing)

    # Create the function body
    function_body_str = print_tree_recurse(0, 1)

    # Create the function end
    function_close_str = \
        '\n{}return ret;\n}}\n'.format(" " * spacing)

    # Show all parts
    if __name__ == "__main__":
        print(typedef_str[:-1])
        print(comment_str[:-1])
        print(function_open_str[:-1])
        print(function_body_str[:-1])
        print(function_close_str[:-1])

    # Save all parts in a file
    code_filepath = join(cfg.MODEL_EMBEDDING_DIR_PATH, 'dtc')
    code_filename = join(code_filepath, 'dtc_model.c')

    if not exists(code_filepath):
        makedirs(code_filepath)

    codefile = open(code_filename, 'w')
    codefile.write(typedef_str)
    codefile.write(comment_str)
    codefile.write(function_open_str)
    codefile.write(function_body_str)
    codefile.write(function_close_str)
    codefile.close()

    print('File written:')
    print(code_filename)


if __name__ == "__main__":
    main()
