;; Auto-generated. Do not edit!


(when (boundp 'segmentation::SegmentationResult)
  (if (not (find-package "SEGMENTATION"))
    (make-package "SEGMENTATION"))
  (shadow 'SegmentationResult (find-package "SEGMENTATION")))
(unless (find-package "SEGMENTATION::SEGMENTATIONRESULT")
  (make-package "SEGMENTATION::SEGMENTATIONRESULT"))

(in-package "ROS")
;;//! \htmlinclude SegmentationResult.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass segmentation::SegmentationResult
  :super ros::object
  :slots (_height _width _depth _data ))

(defmethod segmentation::SegmentationResult
  (:init
   (&key
    ((:height __height) 0)
    ((:width __width) 0)
    ((:depth __depth) 0)
    ((:data __data) (let (r) (dotimes (i 0) (push (instance sensor_msgs::Image :init) r)) r))
    )
   (send-super :init)
   (setq _height (round __height))
   (setq _width (round __width))
   (setq _depth (round __depth))
   (setq _data __data)
   self)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; uint32 _height
    4
    ;; uint32 _width
    4
    ;; uint32 _depth
    4
    ;; sensor_msgs/Image[] _data
    (apply #'+ (send-all _data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _height
       (write-long _height s)
     ;; uint32 _width
       (write-long _width s)
     ;; uint32 _depth
       (write-long _depth s)
     ;; sensor_msgs/Image[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _depth
     (setq _depth (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; sensor_msgs/Image[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (let (r) (dotimes (i n) (push (instance sensor_msgs::Image :init) r)) r))
     (dolist (elem- _data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get segmentation::SegmentationResult :md5sum-) "5052522eddaa15fba41aac6bebabe984")
(setf (get segmentation::SegmentationResult :datatype-) "segmentation/SegmentationResult")
(setf (get segmentation::SegmentationResult :definition-)
      "uint32 height
uint32 width
uint32 depth
sensor_msgs/Image[] data
================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :segmentation/SegmentationResult "5052522eddaa15fba41aac6bebabe984")


