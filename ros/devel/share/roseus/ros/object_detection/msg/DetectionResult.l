;; Auto-generated. Do not edit!


(when (boundp 'object_detection::DetectionResult)
  (if (not (find-package "OBJECT_DETECTION"))
    (make-package "OBJECT_DETECTION"))
  (shadow 'DetectionResult (find-package "OBJECT_DETECTION")))
(unless (find-package "OBJECT_DETECTION::DETECTIONRESULT")
  (make-package "OBJECT_DETECTION::DETECTIONRESULT"))

(in-package "ROS")
;;//! \htmlinclude DetectionResult.msg.html


(defclass object_detection::DetectionResult
  :super ros::object
  :slots (_out_class _out_score _location ))

(defmethod object_detection::DetectionResult
  (:init
   (&key
    ((:out_class __out_class) 0)
    ((:out_score __out_score) 0.0)
    ((:location __location) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _out_class (round __out_class))
   (setq _out_score (float __out_score))
   (setq _location __location)
   self)
  (:out_class
   (&optional __out_class)
   (if __out_class (setq _out_class __out_class)) _out_class)
  (:out_score
   (&optional __out_score)
   (if __out_score (setq _out_score __out_score)) _out_score)
  (:location
   (&optional __location)
   (if __location (setq _location __location)) _location)
  (:serialization-length
   ()
   (+
    ;; uint32 _out_class
    4
    ;; float32 _out_score
    4
    ;; float32[] _location
    (* 4    (length _location)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _out_class
       (write-long _out_class s)
     ;; float32 _out_score
       (sys::poke _out_score (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _location
     (write-long (length _location) s)
     (dotimes (i (length _location))
       (sys::poke (elt _location i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _out_class
     (setq _out_class (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _out_score
     (setq _out_score (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _location
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _location (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _location i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get object_detection::DetectionResult :md5sum-) "3b7fae78ce58faba54a3eab2d7ca9e3c")
(setf (get object_detection::DetectionResult :datatype-) "object_detection/DetectionResult")
(setf (get object_detection::DetectionResult :definition-)
      "uint32 out_class
float32 out_score
float32[] location
")



(provide :object_detection/DetectionResult "3b7fae78ce58faba54a3eab2d7ca9e3c")


