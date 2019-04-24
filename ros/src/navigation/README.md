## Table of Content

- [The Navigation Stack](#The%20Navigation%20Stack)
	* [RTABMap](#RTABMap)
	* [Path Planning](#Path%20Planning)
	* [Vehicle Motion Control](#Vehicle%20Motion%20Control)

<a name="The%20Navigation%20Stack"></a>

## The Navigation Stack

<center>
	<img src="https://github.com/sigmaai/self-driving-golf-cart/media/nav_stack.png" alt="Drawing" width="640"/>
</center>

The self-driving vehicle uses a modified version of the ROS navigation stack. The flowchart above illustrate the mapping and path planning process. First, I create a detailed map of the environment with `rtabmap_ros`. With that global map, I use the localization feature of `rtabmap_ros` and the odom feature of the zed camera system to localize and plan paths. 

<a name="RTABMap" > </a>

### RTABMap

`rtabmap` (realtime appearance based mapping) allows me to construct a global map of the environment. For more information on the mapping package, please check out this [`.launch` file](./ros/src/navigation/mapping/launch/rtab_mapping.launch). 

<center>
	<img src="https://github.com/sigmaai/self-driving-golf-cart/media/rtab-map.jpg" alt="Drawing" width="640"/>
</center> 

<a name="Path%20Planning" > </a>

### Path Planning

The project uses the [`move_base`](http://wiki.ros.org/move_base) node from the navigation stack. The image below shows the costmap (in blue and purple), and the global occupancy grid (in black and gray). `move_base` also plans the local and global path. Global paths are shown in green and yellow below. You can find the `yaml` files [here](./ros/src/navigation/path_planning/params). 

<center>
	<img src="https://github.com/sigmaai/self-driving-golf-cart/media/path_plan_1.png" alt="Drawing" width="640"/>
</center>

<a name="Vehicle%20Motion%20Control" > </a>

### Vehicle Motion Control

The move base node publishes `/cmd_vel` commands, which are processed and sent directly to the vehicle. 

# Contact / Info
If you are interested in the detailed development process of this project, you can visit Neil's blog at [neilnie.com](https://neilnie.com) to find out more about it. Neil will make sure to keep you posted about all of the latest development on the club.

**Developers:**

<img src="https://github.com/sigmaai/self-driving-golf-cart/media/neil.png" alt="Drawing" width="80"/>

**Neil (Yongyang) Nie** | [Email](mailto:yongyang.nie@gmail.com) | [Github](https://www.github.com/NeilNie) | [Website](neilnie.com) | [Linkedin](https://www.linkedin.com/in/yongyang-neil-nie-896204118/)