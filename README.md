<center><img src="./media/front_cover.png" alt="Drawing" style="width: 700;"/>
</center>

## Introduction

We wanted to build a self driving car. We didn't build an actual car, but we built a golf cart, and wrote a system that can work on a real car as well. 

This project has several different modules. 

1. Deep learning steering prediction
2. Semantic segmentation
3. Driver by wire system (DBW)
4. System management
5. Path planning
6. Engineering

Localization and other features coming soon...

## Running the code
1. Please download/clone the repository.
2. Make sure that you have all the [dependencies](./requirements.txt) installed. 

To run the autonomous software, do 

`python3 drive.py`. 

(Please use python3 instead of python2, because the ML models are saved in the python3 formate)

## Steering
The software behind the steering system is largly inspired by work done by [Nvidia](https://arxiv.org/pdf/1604.07316.pdf). The hardware system is custom designed in house. Here is a video demo.


[![IMAGE ALT TEXT HERE](https://i.ytimg.com/vi/4bZ40W4BGoE/hqdefault.jpg)](https://www.youtube.com/watch?v=CcUXtViFQeU&t=5s)


## Autonomous Cruise Control System (ACCS)
### Semantic Segmentation

<img src="./media/seg.png" alt="Drawing" width="480"/>


Understanding the work around the vehicle through segmentation, and making decisions based on the segmentic segmentation results

[![IMAGE ALT TEXT HERE](https://i.ytimg.com/vi/_y2RCakRrc4/hqdefault.jpg)](https://www.youtube.com/watch?v=_y2RCakRrc4)


## Path Planning

Coming soon...

## Localization

Coming soon...

## Development process
We have completed phase 1 of the development process, which mainly includes:

- Implementing control-by-wire system. (hardware)
- Implement the autonomous steering system.
- Implement the obstacle avoidence system. 

For the second phase of the development process, we will focus on making the system safer and more reliable. For details, please refer to the [CHECKLIST](./CHECKLIST.md)

## Contact / Info
If you are interested in the detailed development process of this project, you can visit Neil's blog at [neilnie.com](neilnie.com) to find out more about it. Neil will make sure to keep you posted about all of the latest development on the club. 

**Developers:**

<img src="./media/michael_profile.jpg" alt="Drawing" width="100"/>

**Michael Meng** | [Email](mailto:xmeng18@deerfield.edu) | [Github](https://github.com/xmeng17)

<img src="./media/neil_profile.jpg" alt="Drawing" width="100"/>

**Neil (Yongyang) Nie** | [Email](mailto:yongyang.nie@gmail.com) | [Github](https://www.github.com/NeilNie) | [Website](neilnie.com) | [Linkedin](https://www.linkedin.com/in/yongyang-neil-nie-896204118/)
