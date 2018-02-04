# Welcome to Autonomous Steering Control Sytetm (ASCS)
One of the most important aspect of the vehicle is autonomous steering. 
![](../media/ascs.JPG)

## About this directory
## Training
`train.py` trains the ConvNet

`model.pys` has all the model architectures

## Running
`steering_predictor` is a class that `drive.py` calls. The predictor takes in an image and return a visualization and steering angle.

`mc.py` uses serial communication to send the arduino & motor controller commands.

`./motor_control` has the arduino motor control program.

## Others
`steering_visualization.py` allows you to visualize the trained results. You must have the [Udacity validation dataset](https://github.com/udacity/self-driving-car/blob/master/datasets/CH2/Ch2_001.tar.gz.torrent).

`simulation.py` allows you to run a trained model on the Udacity driving simulator

`validation.py` run inferences on all images in the Udacity validation dataset. This is a performance benchmark for the models. 