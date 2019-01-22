Running the Simulation
======================

.. image:: https://raw.githubusercontent.com/sigmaai/self-driving-golf-cart/97e10f05e95266acb398825fdd21075111e8ea82/ros/src/simulation/assets/simulator-2.png

Building a self-driving car is hard. Not everyone has access to expensive hardware. I am currently trying to integrate this project with the CARLA self-driving simulator. If you are interested in CARLA, please refer to the following documentation. (The current ROS system in this project can only partially run on the CARLA simulator)

===================
Build Unreal Engine
===================

Please note that Unreal Engine repositories are private. In order to gain access you need to **add your GitHub username when you sign up at www.unrealengine.com.**

Download and compile Unreal Engine 4.18. Here we will assume you install it at "~/UnrealEngine_4.18", but you can install it anywhere, just replace the path where necessary::

$ git clone --depth=1 -b 4.18 https://github.com/EpicGames/UnrealEngine.git ~/UnrealEngine_4.18
$ cd ~/UnrealEngine_4.18
$ ./Setup.sh && ./GenerateProjectFiles.sh && make

Check Unreal's documentation "Building On Linux" if any of the steps above fail.

================
Installing Carla
================

.. image:: https://carla.readthedocs.io/en/latest/img/manual_control.png

To install Carla the simulator is simple. Just head over to their releases page on Github and download the latest pre-built release. At the time of writing, the latest release can be found here: https://drive.google.com/open?id=1JprRbFf6UlvpqX98hQiUG9U4W_E-keiv


======================
Running the Simulation
======================
1. Setting Up:
--------------

1.1. Install CARLA
------------------
Download the compiled version of the CARLA simulator from here

Please refer to the CARLA documentation or open an issue if you have any questions or problems.

1.2. Install the CARLA Python API
---------------------------------
At this point, you should have downloaded the compiled version of the CARLA simulator. ::

$ sudo easy_install <path/to/carla/>/PythonAPI/<your_egg_file>

Just as an example, for me, the command is this::

$ sudo easy_install '/home/neil/carla/PythonAPI/carla-0.9.2-py2.7-linux-x86_64.egg'

Please note that you have to put in the complete path to the egg-file including the egg-file itself. Please use the one, that is supported by your Python version. Depending on the type of CARLA (pre-build, or build from source), the egg files are typically located either directly in the PythonAPI folder or in PythonAPI/dist.

Check the installation is successfull by trying to import carla from python::

$ python -c 'import carla;print("Success")'
You should see the Success message without any errors.

1.3. Install other requirements:
--------------------------------
::

$ sudo apt-get install python-protobuf
$ pip install --user simple-pid
If you have followed the instructions closely, you should be ready to use CARLA and the simulation_package

2. Running
----------

2.1. Compile the project
------------------------
Navigate to the ros directory of the self-driving golf cart project::

$ cd [PROJECT_DIRECTORY]/ros

Then enter & run the commands::

$ catkin_make
$ source devel/setup.bash

2.2. Launch CARLA
-----------------
First run the simulator. For the full and latest documentary, please always refer to the carla official website. (see carla documentation: http://carla.readthedocs.io/en/latest/). I have created a simple script. Enter the following command and run the script::

$ rosrun simulation launch_carla.sh 
Wait for the message::

$ Waiting for the client to connect...

2.3. Start the ros bridge & rviz
--------------------------------
::

$ roslaunch simulation carla_client_with_rviz.launch

You should see a new rviz window. You can setup the vehicle configuration config/settings.yaml.

This launch file also make use of the CARLA Python API script ``manual_control.py``. This spawns a vehicle with role_name='hero' which is interpreted as the ego vehicle as defined by the ``config/settings.yaml``.

.. image:: https://raw.githubusercontent.com/sigmaai/self-driving-golf-cart/master/ros/src/simulation/assets/simulator-1.png

The launch file also **further spawn 30 other vehicles** using ``spawn_npc.py`` from CARLA Python API. Then those vehicles will show up also on ROS side.

