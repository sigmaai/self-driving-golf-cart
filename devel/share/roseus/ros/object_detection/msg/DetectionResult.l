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
  :slots (_out_class _out_score _upper_left _upper_right _lower_left _lower_right ))

(defmethod object_detection::DetectionResult
  (:init
   (&key
    ((:out_class __out_class) "")
    ((:out_score __out_score) 0.0)
    ((:upper_left __upper_left) 0)
    ((:upper_right __upper_right) 0)
    ((:lower_left __lower_left) 0)
    ((:lower_right __lower_right) 0)
    )
   (send-super :init)
   (setq _out_class (string __out_class))
   (setq _out_score (float __out_score))
   (setq _upper_left (round __upper_left))
   (setq _upper_right (round __upper_right))
   (setq _lower_left (round __lower_left))
   (setq _lower_right (round __lower_right))
   self)
  (:out_class
   (&optional __out_class)
   (if __out_class (setq _out_class __out_class)) _out_class)
  (:out_score
   (&optional __out_score)
   (if __out_score (setq _out_score __out_score)) _out_score)
  (:upper_left
   (&optional __upper_left)
   (if __upper_left (setq _upper_left __upper_left)) _upper_left)
  (:upper_right
   (&optional __upper_right)
   (if __upper_right (setq _upper_right __upper_right)) _upper_right)
  (:lower_left
   (&optional __lower_left)
   (if __lower_left (setq _lower_left __lower_left)) _lower_left)
  (:lower_right
   (&optional __lower_right)
   (if __lower_right (setq _lower_right __lower_right)) _lower_right)
  (:serialization-length
   ()
   (+
    ;; string _out_class
    4 (length _out_class)
    ;; float32 _out_score
    4
    ;; uint32 _upper_left
    4
    ;; uint32 _upper_right
    4
    ;; uint32 _lower_left
    4
    ;; uint32 _lower_right
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _out_class
       (write-long (length _out_class) s) (princ _out_class s)
     ;; float32 _out_score
       (sys::poke _out_score (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _upper_left
       (write-long _upper_left s)
     ;; uint32 _upper_right
       (write-long _upper_right s)
     ;; uint32 _lower_left
       (write-long _lower_left s)
     ;; uint32 _lower_right
       (write-long _lower_right s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _out_class
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _out_class (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _out_score
     (setq _out_score (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _upper_left
     (setq _upper_left (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _upper_right
     (setq _upper_right (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _lower_left
     (setq _lower_left (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _lower_right
     (setq _lower_right (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get object_detection::DetectionResult :md5sum-) "5fb792480f070cbc69af8da53d614697")
(setf (get object_detection::DetectionResult :datatype-) "object_detection/DetectionResult")
(setf (get object_detection::DetectionResult :definition-)
      "string out_class
float32 out_score
uint32 upper_left
uint32 upper_right
uint32 lower_left
uint32 lower_right
")



(provide :object_detection/DetectionResult "5fb792480f070cbc69af8da53d614697")


