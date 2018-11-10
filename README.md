<center>
<img src="./media/cover.png" alt="Drawing" style="width: 700;"/>
</center>

[![Documentation Status](https://readthedocs.org/projects/self-driving-golf-cart/badge/?version=latest)](https://self-driving-golf-cart.readthedocs.io/en/latest/?badge=latest)
[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](paypal.me/YongyangNie)
[![Build Status](https://travis-ci.org/sigmaai/self-driving-golf-cart.svg?branch=master)](https://travis-ci.org/sigmaai/self-driving-golf-cart)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

# Introduction

Welcome! This is an open source self-driving development platform aimed for rapid prototyping, deep learning and robotics research. The system currently runs on a modified electric golf cart, but the code could work on a real car as well. Here are our goals:

### Goals:
1. Research and develop a deep learning-driven self-driving car.
2. The vehicle should be able to navigate from point A to point B autonomously within a geofenced area.

#### Here are the modules in this project.

1. End-to-End steering (Behavioral cloning)
2. Semantic Segmentation
3. Drive by Wire (DBW)
4. Object Detection 🚙
5. Traffic Light Detection 🚦
6. Lane Detection 🛣
7. Localization 🛰️ (currently with GPS)

Path planning is coming soon...

For the full documentation of the development process, please visit: [neilnie.com](neilnie.com)

# Running the code
1. Please download/clone the repository.
2. Make sure you have all the [dependencies](./requirements.txt) installed. 
3. Make sure that you have the [ROS](http://wiki.ros.org/ROS/Installation) installed on your computer.
4. `cd YOUR_PROJECT_DIRECTORY`
5. `catkin_make`
6. `source devel/setup.bash`
7. `roscore`
8. `roslaunch driver drive.launch`

<img src="./media/running.png" alt="image" width="480"/>

Bon Voyage 😀

# Simulation
Building a self-driving car is hard. Not everyone has access to expensive hardware. If you want to run the code inside the CARLA self-driving simulator, please refer to this [documentation](./src/simulation/README.md). The ROS system in this project can run on the CARLA simulator. 

<img src="./media/simulator.png" alt="Drawing" width="480"/>

# ROS
This project is being developed using ROS. The launch files will launch the neccesary nodes as well as rviz for visualization. For more information on ROS, nodes, topics and others please refer to the [README](./src/README.md) in the `./src` directory. 

# Deep Learning for Autonomous Steering

### Motives
In 1989, ALVINN (the self-driving truck) made by Dr. Dean Pomerleau and his team, drove around the Carnegie Mellon campus. According to Pomerleau, The vehicle was powered by a CPU slower than the Apple Watch. The car used a fully connected neural network to predict the steering angle of the car in real time. Fast forward twenty years, NVIDIA proposed a novel method that combines Pomerleau's idea with the modern GPU, giving NVIDIA's car the capability to accurately perform real-time end to end steering prediction. Around the same time, Udacity held a challenge that asked researchers to create the best end to end steering prediction model. This project is deeply inspired by that competition, and the goal is to further the work in behavioral cloning for self-driving vehicles. 

NVIDIA's paper used a convolutional neural network with a single frame input. I believe that, even though this architecture yielded good results, the single frame CNN doesn't provide any temporal information which is critical in self-driving. This is the motive behind choosing the i3d architecture, which is rich in spacial-temporal information.

<!--In fact, the winner of the Udacity challenge (team komanda) deployed a Convolutional LSTM architecture originally used for video classification and weather forecasting. -->

### Model
The input of the network is a 3d convolutional block, with the shape of `n * weight * height * 3`. `n` is the length of the input sequence. Furthermore, the network also uses nine inception modules. The output layers are modified to accommodate for this regression problem. A flatten layer and a dense layer are added to the back of the network. 

<img src="./media/model.png" alt="Drawing" width="480"/>

Here is a video demo of deep learning model running on the autonomous golf cart. 

[![IMAGE ALT TEXT HERE](https://i.ytimg.com/vi/4bZ40W4BGoE/hqdefault.jpg)](https://www.youtube.com/watch?v=CcUXtViFQeU&t=5s)

# Autonomous Cruise Control System (ACCS)

#### Semantic Segmentation

<img src="./media/seg.png" alt="Drawing" width="480"/>

The cart understands the surrounding  through semantic segmentation, which is a technique in computer that classifies each pixel in an image. The vehicle can also make decisions based on the segmentic segmentation results. The cart can change its speed based on the proximity to nearby obstacles.

We deployed the ENet architecture for segmentation. ENet is design to work well in realtime applications. For more information, please visit the [paper](http://arxiv.org/pdf/1606.02147.pdf). We used the CityScape dataset for training and the python code for training and inferencing are located in the `./src/segmentation/scripts` directory.

[![IMAGE ALT TEXT HERE](https://i.ytimg.com/vi/_y2RCakRrc4/hqdefault.jpg)](https://www.youtube.com/watch?v=_y2RCakRrc4)

# Localization

Currently, the localization module uses GPS (Global Positioning System) to find the precise location of the vehicle. However, GPS is far from enough. Localization using lidar and radar (sensor fusion and particle filters) are currently under development.

<img src="./media/gps-loc.png" alt="Drawing" width="480"/>

Furthermore, we are relying on OSM (Open Street Map) data for navigation. OSM maps provide detailed information about the paths, buildings and other landmarks in the surrounding. Currently, navigation is only limited to a geofenced area.

<img src="./media/osm-1.png" alt="Drawing" width="480"/>

# Path Planning

Coming soon...

# What's next?

#### Phase 1

We have completed phase 1 of the development process, which mainly includes:

- Drive-by-wire system.
- Autonomous steering system with deep learning
- Basic obstacle avoidance system using segmentation & detection

As you might have realized, all of these above are focused on computer vision and deep learning. Currently, the vehicle can navigate autonomously in a controlled outdoor environment for about 1000 feet, swiftly avoiding obstacles and stopping for pedistrians.

#### Phase 2

For the second phase of the development process, we will focus on making the system safer and more reliable. Basic plans include:

- Implement a localization system.
- Write a path planner.
- Collect more data in our geofenced enviroment. ✅
- Improve the computer hardware. ✅
- Improve the sensor system.

We are keeping track of all our progress here [CHECKLIST](./CHECKLIST.md). 

# Contact / Info
If you are interested in the detailed development process of this project, you can visit Neil's blog at [neilnie.com](neilnie.com) to find out more about it. Neil will make sure to keep you posted about all of the latest development on the club.

**Developers:**

<img src="./media/neil.png" alt="Drawing" width="80"/>

**Neil (Yongyang) Nie** | [Email](mailto:yongyang.nie@gmail.com) | [Github](https://www.github.com/NeilNie) | [Website](neilnie.com) | [Linkedin](https://www.linkedin.com/in/yongyang-neil-nie-896204118/)

<img src="./media/michael.png" alt="Drawing" width="80"/>

**Michael Meng** | [Email](mailto:xmeng18@deerfield.edu) | [Github](https://github.com/xmeng17)