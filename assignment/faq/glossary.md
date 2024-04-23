# Glossary
Based on  [ref](https://developers.google.com/machine-learning/glossary/).

**accuracy**	The number of correct classification predictions divided by the total number of predictions.

**attribute**	A particular type of data in your sample
attribute 

**sampling**	A tactic for training a decision forest in which each decision tree considers only a random subset of possible features when learning the condition.
Generally, a different subset of features is sampled for each node. In contrast, when training a decision tree without attribute sampling, all possible features are considered for each node.

**bagging** Or bootstrap aggregating.
A method to train an ensemble where each constituent model trains on a random subset of training examples sampled with replacement. 
For example, a random forest is a collection of decision trees trained with bagging.


**bias** 
1. Stereotyping, prejudice or favoritism towards some things, people, or groups over others. These biases can affect collection and interpretation of data, the design of a system, and how users interact with a system. 
2. Systematic error introduced by a sampling or reporting procedure.
3. As used in math: An intercept or offset from an origin.

**binary classification**	A type of classification task that predicts one of two mutually exclusive classes: the positive class, the negative class.

**binary condition**	In a decision tree, a condition that has only two possible outcomes, typically yes or no.

**boosting**	A tactic that iteratively combines a set of simple and not very accurate classifiers (referred to as weak classifiers) into a classifier with high accuracy (a strong classifier) by upweighting the examples that the model is currently misclassifying.

**bucketing**	Converting a single feature into multiple binary features called buckets or bins, typically based on a value range.
The chopped feature is typically a continuous feature.

**centroid**	The center of a cluster as determined by a k-means or k-median algorithm.

**class**	A category that a label can belong to.

**lassification model**	Or classifier.
A model whose prediction is a class.

**clustering**	Grouping related examples, particularly during unsupervised learning. 

**condition** 	In a decision tree, any node that evaluates an expression. 
A condition is also called a split or a test.

**confirmation bias**	The tendency to search for, interpret, favor, and recall information in a way that confirms one's preexisting beliefs or hypotheses.

**confusion matrix**	A table that summarizes the number of correct and incorrect predictions that a classification model made.

**continuous**	real number

**cross-validation**	A mechanism for estimating how well a model would generalize to new data by testing the model against one or more non-overlapping data subsets withheld from the training set.

**decision boundary**	The separator between classes learned by a model in a binary class or multi-class classification problems.

**decision forest**	A model created from multiple decision trees. 
A decision forest makes a prediction by aggregating the predictions of its decision trees. Popular types of decision forests include random forests and gradient boosted trees.

**decision tree**	A supervised learning model composed of a set of conditions and leaves organized hierarchically.
discrete	Nominal, natura number, i.e. from a countable set f values
example	The values of one row of features and possibly a label. 

**false negative (FN)**	An example in which the model mistakenly predicts the negative class.

**false positive (FP)**	An example in which the model mistakenly predicts the positive class. 

**feature**	An input variable to a machine learning model. This can be an attribute or a combination of attributes

**feature engineering**	A process that involves the following steps:
1. Determining which features might be useful in training a model.
2. Converting raw data from the dataset into efficient versions of those features.
feature vector	The array of feature values comprising an example. 

**generalization**	A model's ability to make correct predictions on new, previously unseen data. 
A model that can generalize is the opposite of a model that is overfitting.

**ground truth**	Reality.
The thing that actually happened.

**implicit bias**	Automatically making an association or assumption based on one’s mental models and memories. Implicit bias can affect the following:
1. How data is collected and classified.
2. How machine learning systems are designed and developed.

**inference**	In machine learning, the process of making predictions by applying a trained model to unlabeled examples.

**inference path**	In a decision tree, during inference, the route a particular example takes from the root to other conditions, terminating with a leaf.

**information gain**	In decision forests, the difference between a node's entropy and the weighted (by number of examples) sum of the entropy of its children nodes.

**instance**	synonym of sample

**k-fold cross validation**	An algorithm for predicting a model's ability to generalize to new data. 
The k in k-fold refers to the number of equal groups you divide a dataset's examples into; that is, you train and test your model k times. 
For each round of training and testing, a different group is the test set, and all remaining groups become the training set. 
After k rounds of training and testing, you calculate the mean and standard deviation of the desired test metric(s).

**label**	In supervised machine learning, the answer or result portion of an example.

**labeled example**	An example that contains one or more features and a label.

**label leakage**	A model design flaw in which a feature is a proxy for the label.

**leaf**	Any endpoint in a decision tree. 
Unlike a condition, a leaf does not perform a test. Rather, a leaf is a possible prediction. 
A leaf is also the terminal node of an inference path.

**linear** 	A relationship between two or more variables that can be represented solely through addition and multiplication.

**machine learning**	A program or system that trains a model from input data. 

**mean**

**metric**	A statistic that you care about.
An objective is a metric that a machine learning system tries to optimize.
minority class	The less common label in a class-imbalanced dataset. 

**model**	In general, any mathematical construct that processes input data and returns output. 
Phrased differently, a model is the set of parameters and structure needed for a system to make predictions.
In supervised machine learning, a model takes an example as input and infers a prediction as output. 

**model training**	The process of determining the best model.

**multi-class classification**	In supervised learning, a classification problem in which the dataset contains more than two classes of labels.

**NaN trap**	When one number in your model becomes a Not a Number (NaN) during training, which causes many or all other numbers in your model to eventually become a NaN.

**negative class**	In binary classification, one class is termed positive and the other is termed negative. 

**node (decision tree)** 	In a decision tree, any condition or leaf.

**normalization**	Broadly speaking, the process of converting a variable's actual range of values into a standard range of values.

**online inference**	Generating predictions on demand. 
For example, an embedded system passes input to a model and issues a request for a prediction.

**out-of-bag evaluation** 	OOB evaluation
A mechanism for evaluating the quality of a decision forest by testing each decision tree against the examples not used during training of that decision tree.
outlier	Values distant from most other values.
overfitting	Creating a model that matches the training data so closely that the model fails to make correct predictions on new data.
performance	How correct is this model? That is, how good are the model's predictions?

**positive class**	The class you are testing for.

**prediction**	A model's output.

**prediction bias**	A value indicating how far apart the average of predictions is from the average of labels in the dataset.

**recall**	A metric for classification models that answers the following question :
When ground truth was the positive class, what percentage of predictions did the model correctly identify as the positive class?

**selection bias**	Or selection bias
Errors in conclusions drawn from sampled data due to a selection process that generates systematic differences between samples observed in the data and those not observed. 
**sparse feature**	A feature whose values are predominately zero or empty.

**sample**	A data point contining one or multiple attributes

**sampling with replacement**	A method of picking items from a set of candidate items in which the same item can be picked multiple times. 
The phrase with replacement means that after each selection, the selected item is returned to the pool of candidate items.

**standard deviation**

**supervised machine learning**	Training a model from features and their corresponding labels. 
Supervised machine learning is analogous to learning a subject by studying a set of questions and their corresponding answers. 
After mastering the mapping between questions and answers, a student can then provide answers to new (never-before-seen) questions on the same topic.

**synthetic feature**	A feature not present among the input features, but assembled from one or more of them.

**temporal data**	Data recorded at different points in time.

**training set**	The subset of the dataset used to train a model.

**true negative (TN)**	An example in which the model correctly predicts the negative class.

**true positive (TP)**	An example in which the model correctly predicts the positive class.

**underfitting**	a model with poor predictive ability because the model hasn't fully captured the complexity of the training data.

**validation**	The initial evaluation of a model's quality. Validation checks the quality of a model's predictions against the validation set.
validation set	The subset of the dataset that performs initial evaluation against a trained model. 
Typically, you evaluate the trained model against the validation set several times before evaluating the model against the test set.

**variance**

**test set**	A subset of the dataset reserved for testing a trained model.
