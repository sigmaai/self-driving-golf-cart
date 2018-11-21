
(cl:in-package :asdf)

(defsystem "segmentation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "SegmentationResult" :depends-on ("_package_SegmentationResult"))
    (:file "_package_SegmentationResult" :depends-on ("_package"))
  ))