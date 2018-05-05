Change history
==============

0.2.4 (2017-12-06)
------------------
* Convert to package xml format 2 and add launch file dependencies
* Possibility to compute paths between GeoPoints.
* Remove unused map file.
* Add Ixtapa testing map.
* Add default configuration for RVIZ.
* Create new launch file to start all the needed nodes to plan on a map.
* Contributors: Bence Magyar, Diego Ramos

0.2.3 (2015-10-12)
------------------

 * move static transform from viz_osm.launch to
   tests/test_viz_osm.launch (`#5`_)

0.2.2 (2015-10-08)
------------------

 * Add publisher ``queue_size`` for Indigo (`#3`_).
 * Remove references to deprecated ``geodesy.gen_uuid`` module.
 * Release to Indigo.
 * Contributors: Augusto Luis Ballardini, Jack O'Quin

0.2.1 (2013-09-24)
------------------

 * Install missing launch scripts (`#2`_). 

0.2.0 (2013-09-18)
------------------

 * Convert to catkin (`#2`_).
 * Release to Hydro.

0.1.0 (2012-07-16)
------------------

 * initial experimental release to Electric, Fuerte and Groovy, uses
   robuild.

.. _`#2`: https://github.com/ros-geographic-info/open_street_map/issues/2
.. _`#3`: https://github.com/ros-geographic-info/open_street_map/issues/3
.. _`#5`: https://github.com/ros-geographic-info/open_street_map/issues/5
