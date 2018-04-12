<center><img src="./media/front_cover.png" alt="Drawing" style="width: 700;"/>
</center>

## Introduction

We wanted to build a self driving car. We didn't build an actual car, but we built a golf cart, and wrote a system that can work on a real car as well. 

This project has several different modules. 

1. Deep learning steering prediction
2. Semantic segmentation
3. Driver by wire system (DBW)
4. Object detection
5. Traffic light detection
6. Lane Detection
7. System management
8. Path planning
9. Engineering

Localization and some other features coming soon...

## Running the code
1. Please download/clone the repository.
2. Make sure that you have all the [dependencies](./requirements.txt) installed. 
3. To run the autonomous software, change your directory to the project directory 
4. `./drive`. 

<img src="./media/running-2.png" alt="Drawing" style="width: 500px;"/>

(Please use python3 instead of python2, because the ML models are saved in the python3 formate)

## Steering
The algorithm behind the steering system is inspired by work done by [Nvidia](https://arxiv.org/pdf/1604.07316.pdf). We developed our own convolutional neural network to predict the steering angle based on images captured by the front camera. The hardware system is custom designed in-house. Here is a video demo.


[![IMAGE ALT TEXT HERE](https://i.ytimg.com/vi/4bZ40W4BGoE/hqdefault.jpg)](https://www.youtube.com/watch?v=CcUXtViFQeU&t=5s)


## Autonomous Cruise Control System (ACCS)
### Semantic Segmentation

<img src="./media/seg.png" alt="Drawing" width="480"/>


The cart understands the world around through segmentation. It also makes decisions based on the segmentic segmentation results. The vehicle can change its speed based on the proximity to nearby obstacles. 

[![IMAGE ALT TEXT HERE](https://i.ytimg.com/vi/_y2RCakRrc4/hqdefault.jpg)](https://www.youtube.com/watch?v=_y2RCakRrc4)


## Path Planning

Coming soon...

## Localization

Coming soon...

## Development process
We have completed phase 1 of the development process, which mainly includes:

- Implementing control-by-wire system. (hardware)
- Implement the autonomous steering system.
- Implement the obstacle avoidance system. 

For the second phase of the development process, we will focus on making the system safer and more reliable. For details, please refer to the [CHECKLIST](./CHECKLIST.md)

## Contact / Info
If you are interested in the detailed development process of this project, you can visit Neil's blog at [neilnie.com](neilnie.com) to find out more about it. Neil will make sure to keep you posted about all of the latest development on the club. 

**Developers:**

<img src="./media/neil_profile.jpg" alt="Drawing" width="100"/>

**Neil (Yongyang) Nie** | [Email](mailto:yongyang.nie@gmail.com) | [Github](https://www.github.com/NeilNie) | [Website](neilnie.com) | [Linkedin](https://www.linkedin.com/in/yongyang-neil-nie-896204118/)

<img src="./media/michael_profile.jpg" alt="Drawing" width="100"/>

**Michael Meng** | [Email](mailto:xmeng18@deerfield.edu) | [Github](https://github.com/xmeng17)