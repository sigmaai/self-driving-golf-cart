1) Inicializace projektu
========================
cd src/
catkin_init_workspace
cd ..

2) Kompilace a spusteni
=======================
source setup.bash
catkin_make_release
roslaunch roboauto setup_bag.launch

3) Vizualizace
==============
rviz &
rosbag play --clock somefile.bag


1) Project initialization
========================
cd src /
catkin_init_workspace
cd ..

2) Compile and run
=======================
source setup.bash
catkin_make_release
roslaunch roboauto setup_bag.launch

3) Visualization
==============
rviz &
rosbag play --clock somefile.bag