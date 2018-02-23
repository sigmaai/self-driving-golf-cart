# ALVNS
Autonomous Land Vehicle Navigation System. Neil Nie & Michael Meng, 2017 
<img src="./media/IMG-7370.JPG" alt="Drawing" style="width: 480px;"/>

## Introduction
We wanted to build a self driving car. We didn't build an actual car, but we built a golf cart, and wrote a system that can work on a real car as well. 

This project has several different modules. 

1. Autonomous steering 
2. Road segmentation 🛣️ 🚗
3. Path planning
4. Engineering

## Steering
The software behind the steering system is largly inspired by work done by [Nvidia](https://arxiv.org/pdf/1604.07316.pdf). The hardware system is custom designed in house. Here is a video demo.

video

[![IMAGE ALT TEXT HERE](https://i.ytimg.com/vi/4bZ40W4BGoE/hqdefault.jpg)](https://www.youtube.com/watch?v=CcUXtViFQeU&t=5s)

## Cruise

## Semantic Segmentation
<img src="./media/seg.png" alt="Drawing" style="width: 480px;"/>

Understanding the work around the vehicle through segmentation, and making decisions based on the segmentic segmentation results

[![IMAGE ALT TEXT HERE](https://i.ytimg.com/vi/_y2RCakRrc4/hqdefault.jpg)](https://www.youtube.com/watch?v=_y2RCakRrc4)


## Path Planning