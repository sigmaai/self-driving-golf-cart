
(cl:in-package :asdf)

(defsystem "object_detection-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DetectionResult" :depends-on ("_package_DetectionResult"))
    (:file "_package_DetectionResult" :depends-on ("_package"))
    (:file "DetectionResults" :depends-on ("_package_DetectionResults"))
    (:file "_package_DetectionResults" :depends-on ("_package"))
  ))