;; Auto-generated. Do not edit!


(when (boundp 'object_detection::DetectionResults)
  (if (not (find-package "OBJECT_DETECTION"))
    (make-package "OBJECT_DETECTION"))
  (shadow 'DetectionResults (find-package "OBJECT_DETECTION")))
(unless (find-package "OBJECT_DETECTION::DETECTIONRESULTS")
  (make-package "OBJECT_DETECTION::DETECTIONRESULTS"))

(in-package "ROS")
;;//! \htmlinclude DetectionResults.msg.html


(defclass object_detection::DetectionResults
  :super ros::object
  :slots (_results ))

(defmethod object_detection::DetectionResults
  (:init
   (&key
    ((:results __results) (let (r) (dotimes (i 0) (push (instance object_detection::DetectionResult :init) r)) r))
    )
   (send-super :init)
   (setq _results __results)
   self)
  (:results
   (&rest __results)
   (if (keywordp (car __results))
       (send* _results __results)
     (progn
       (if __results (setq _results (car __results)))
       _results)))
  (:serialization-length
   ()
   (+
    ;; object_detection/DetectionResult[] _results
    (apply #'+ (send-all _results :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; object_detection/DetectionResult[] _results
     (write-long (length _results) s)
     (dolist (elem _results)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; object_detection/DetectionResult[] _results
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _results (let (r) (dotimes (i n) (push (instance object_detection::DetectionResult :init) r)) r))
     (dolist (elem- _results)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get object_detection::DetectionResults :md5sum-) "c9a646c35e7a13cc2d94d4de60869423")
(setf (get object_detection::DetectionResults :datatype-) "object_detection/DetectionResults")
(setf (get object_detection::DetectionResults :definition-)
      "DetectionResult[] results
================================================================================
MSG: object_detection/DetectionResult
uint32 out_class
float32 out_score
float32[] location
")



(provide :object_detection/DetectionResults "c9a646c35e7a13cc2d94d4de60869423")


