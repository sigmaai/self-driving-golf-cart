# **Traffic Sign Recognition** 

---

**Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./img/training_sample_image.png "Sign Sample"
[image2]: ./img/Sign_Percentage.png "Bar Chart"
[image3]: ./img/test_images.png "Test Images"
[image4]: ./img/predictions.png "Top 5 Predictions"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/481/view) individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf. You can use this template as a guide for writing the report. The submission includes the project code.

You're reading it! The project code and html file will also be included in the zip file.

### Data Set Summary & Exploration

#### 1. Provide a basic summary of the data set. In the code, the analysis should be done using python, numpy and/or pandas methods rather than hardcoding results manually.

I used nothing more than native python functions for the summary:

* The size of training set is 34799
* The size of the validation set is 4410
* The size of test set is 12630
* The shape of a traffic sign image is (32, 32, 3)
* The number of unique classes/labels in the data set is 43

#### 2. Include an exploratory visualization of the dataset.

Here is a figure of all 43 types of signs in the training set:

![alt text][image1]

Here is a bar figure of the percentage of each type of sign in all three data sets:

![alt text][image2]

### Design and Test a Model Architecture

#### 1. Describe how you preprocessed the image data. What techniques were chosen and why did you choose these techniques? Consider including images showing the output of each preprocessing technique. Pre-processing refers to techniques such as converting to grayscale, normalization, etc. (OPTIONAL: As described in the "Stand Out Suggestions" part of the rubric, if you generated additional data for training, describe why you decided to generate additional data, how you generated the data, and provide example images of the additional data. Then describe the characteristics of the augmented training set like number of images in the set, number of images for each class, etc.)

I converted the images to grayscale because with only the image value we are already able to distinguish between images. Besides, The color will increase the amount of processing time. 

Then, I normalized the image data because this puts all images to the same footing. In that way, the model won't make false judgement because of the overall brightness and contrast of the images.

Since the process here is standard, I did not include a visualization of each step. Also, I did not include additional steps, as my model was already performing well without them. 

#### 2. Describe what your final model architecture looks like including model type, layers, layer sizes, connectivity, etc.) Consider including a diagram and/or table describing the final model.

My final model consisted of the following layers:

| Layer         		|     Description	        					| 
|:---------------------:|:---------------------------------------------:| 
| Input         		| 32x32x3 RGB image   							| 
| Convolution 5x5     	| 1x1 stride, valid padding, outputs 28x28x16 	|
| RELU					|												|
| Max pooling	      	| 2x2 stride,  outputs 14x14x16 				|
| Dropout			 	| Dropout Rate is stored in a tensor			|
| Convolution 5x5	    | 1x1 stride, valid padding, outputs 10x10x32	|
| RELU					|												|
| Max pooling	      	| 2x2 stride,  outputs 5x5x32 					|
| Dropout			 	| Dropout Rate is stored in a tensor			|
| Flatten			 	| outputs 800									|
| Fully connected		| outputs 400									|
| Fully connected		| outputs 200									|
| Fully connected		| outputs 43 									| 


#### 3. Describe how you trained your model. The discussion can include the type of optimizer, the batch size, number of epochs and any hyperparameters such as learning rate.

To train the model, I used softmax cross entropy for loss and adam optimizer for optimization. I chose a epoch of 10, batch size of 128, learning rate of 0.001 and dropout rate of 0.7. 

#### 4. Describe the approach taken for finding a solution and getting the validation set accuracy to be at least 0.93. Include in the discussion the results on the training, validation and test sets and where in the code these were calculated. Your approach may have been an iterative process, in which case, outline the steps you took to get to the final solution and why you chose those steps. Perhaps your solution involved an already well known implementation or architecture. In this case, discuss why you think the architecture is suitable for the current problem.

My final model results were:
* training set loss of 0.0166
* validation set accuracy of 99.65%
* test set accuracy of 96.07%

If an iterative approach was chosen:
* What was the first architecture that was tried and why was it chosen?
I first chose the LeNet.
* What were some problems with the initial architecture?
I realized that distinguishing between 43 different catagories means I need to adjust the parameters.
* How was the architecture adjusted and why was it adjusted? Typical adjustments could include choosing a different model architecture, adding or taking away layers (pooling, dropout, convolution, etc), using an activation function or changing the activation function. One common justification for adjusting an architecture would be due to overfitting or underfitting. A high accuracy on the training set but low accuracy on the validation set indicates over fitting; a low accuracy on both sets indicates under fitting.
I increased the convolution layer depth and increased nodes in the fully connected layer. In that way, there is less loss of information, as the network needs to distinguishing between 43 different catagories instead of 10 in case of LeNet.
I added dropout layers so that the model is more robust with less overfitting. 
* Which parameters were tuned? How were they adjusted and why?
The batch size is adjusted. I try to find the right batch size that suits my computer memory, but is not too small for poor performance. 
* What are some of the important design choices and why were they chosen? For example, why might a convolution layer work well with this problem? How might a dropout layer help with creating a successful model?
The convolution layer extracts important features, initially several pixels, then to lines, shapes and eventually to the whole sign. By using convolution layers, we do not need to worry about centering the image as it will extract the features regardless of the position of the images.
As mentioned before, the dropout layers help prevent overfitting. They randomly shut down parts of the network so that the network does not depend only on certain features, and instead consider holistic all features.

If a well known architecture was chosen:
* What architecture was chosen?
An architecture similar to LeNet was chosen.
* Why did you believe it would be relevant to the traffic sign application?
Since the input image size is also 32 by 32, the atchitecture works well. However, the output is a vector of 43 dimensions, and thus parameters need to be adjusted.
* How does the final model's accuracy on the training, validation and test set provide evidence that the model is working well?
The high accuracy on validation and test set proves that the model is not overfitting. The low training loss shows the model is not underfitting. 

### Test a Model on New Images

#### 1. Choose five German traffic signs found on the web and provide them in the report. For each image, discuss what quality or qualities might be difficult to classify.

Here are five German traffic signs that I found on the web:

![alt text][image3]

All five are clear and should be easy to classify.

#### 2. Discuss the model's predictions on these new traffic signs and compare the results to predicting on the test set. At a minimum, discuss what the predictions were, the accuracy on these new predictions, and compare the accuracy to the accuracy on the test set (OPTIONAL: Discuss the results in more detail as described in the "Stand Out Suggestions" part of the rubric).

Here are the results of the prediction:

| Image			        |     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| 50 km/h				| 50 km/h   									| 
| 70 km/h				| 70 km/h 										|
| Turn Right			| Turn Right									|
| Road Work      		| Road Work  					 				|
| No Entry				| No Entry										|


The model was able to correctly guess 5 of the 5 traffic signs, which gives an accuracy of 100%. This compares favorably to the accuracy on the test set. However, I haven't test on more blurry or confusing images yet, which might cause lower accuracy.

#### 3. Describe how certain the model is when predicting on each of the five new images by looking at the softmax probabilities for each prediction. Provide the top 5 softmax probabilities for each image along with the sign type of each probability. (OPTIONAL: as described in the "Stand Out Suggestions" part of the rubric, visualizations can also be provided such as bar charts)

For all five images, the model is very sure about its prediction. The probabilities are all higher than 90%. This is shown in the bar chart below:

![alt text][image4]

Image 1:
| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| .97         			| 50 km/h   									| 
| .02     				| 30 km/h 										|
| .00					| Wild Animal Crossing							|
| .00	      			| 20 km/h   					 				|
| .00				    | 80 km/h      									|

Image 2:
| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| 1.00         			| 70 km/h   									| 
| .00     				| 20 km/h 										|
| .00					| 30 km/h 										|
| .00	      			| Roundabout   					 				|
| .00				    | Straight or Left								|

Image 3:
| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| 1.00         			| Turn Right   									| 
| .00     				| Straight or Left								|
| .00					| Ahead Only									|
| .00	      			| Keep Left   					 				|
| .00				    | Right at Next Intersection   					|

Image 4:
| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| 1.00         			| Road Work   									| 
| .00     				| Double Curve									|
| .00					| Bicycle Crossing								|
| .00	      			| Wild Animal Crossing			 				|
| .00				    | Bumy Road    									|

Image 5:
| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| 1.00         			| No Entry   									| 
| .00     				| Stop											|
| .00					| Turn Right									|
| .00	      			| Keep Right   					 				|
| .00				    | End of Speed Limits							|