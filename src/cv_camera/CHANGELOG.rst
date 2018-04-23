^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cv_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2017-10-03)
------------------
* Update README.md
* Add device_path support (`#8 <https://github.com/OTL/cv_camera/issues/8>`_)
  * Add device_path support
* Contributors: Maurice Meedendorp, Takashi Ogura

0.2.1 (2017-05-05)
------------------
* use OpenCV3
* rostest is optional

0.1.0 (2015-10-17)
------------------
* Fix opencv2 to libopencv-dev
* Contributors: Takashi Ogura

0.0.3 (2015-10-17)
------------------
* Enable any prop code
* Fix coding style using roslint
* Support CV_CAP_PROP_* params
* Contributors: Takashi Ogura

0.0.2 (2013-11-08)
------------------
* rostest should be build_depend (for binary package build)
  see http://docs.ros.org/api/catkin/html/howto/rostest_configuration.html
  for more information.

0.0.1 (2013-11-07)
------------------
* change behavior with calibration yaml.
  overwrite image size with yaml data if yaml is specified.
  if no calib is provided, use image size.
* add offline mode and tests
* use camera_info_manager for CameraInfo.
* add parameters frame_id/image_width/image_height
* ROS Camera node by cv::VideoCapture
* Initial commit
