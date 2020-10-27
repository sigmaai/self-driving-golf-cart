## Autopilot & End-to-End Behavioral Cloning

The autopilot system uses deep learning to predict the steering commands and acceleration commands for the vehicle, only using data collected by the front facing camera. 

### What's Behavioral Cloning
In 2016, NVIDIA proposed a novel deep learning approach allowed their car to accurately perform real-time end-to-end steering command prediction. Around the same time, Udacity held a challenge that asked researchers to create the best end-to-end steering prediction model. Our goal is to further the work in behavioral cloning for self-driving vehicles. 

### Model

NVIDIA's paper used a convolutional neural network with a single frame input. I believe that the single-frame-input CNN doesn't provide any temporal information which is critical in self-driving. This is the motive behind choosing the i3d architecture, which is rich in spacial-temporal information.

The input of the network is a 3d convolutional block, with the shape of `n * weight * height * 3`. `n` is the length of the input sequence. A flatten layer and a dense layer are added to the back of the network for the purpose of this regression problem. 

<center>
	<img src="https://raw.githubusercontent.com/sigmaai/self-driving-golf-cart/master/media/model.png" alt="Drawing" width="640"/>
</center>

Here is a video demo of deep learning model running on the autonomous golf cart. 

[VIDEO DEMO](https://www.youtube.com/watch?v=4bZ40W4BGoE)