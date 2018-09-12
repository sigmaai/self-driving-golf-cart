; Auto-generated. Do not edit!


(cl:in-package object_detection-msg)


;//! \htmlinclude DetectionResults.msg.html

(cl:defclass <DetectionResults> (roslisp-msg-protocol:ros-message)
  ((results
    :reader results
    :initarg :results
    :type (cl:vector object_detection-msg:DetectionResult)
   :initform (cl:make-array 0 :element-type 'object_detection-msg:DetectionResult :initial-element (cl:make-instance 'object_detection-msg:DetectionResult))))
)

(cl:defclass DetectionResults (<DetectionResults>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectionResults>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectionResults)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_detection-msg:<DetectionResults> is deprecated: use object_detection-msg:DetectionResults instead.")))

(cl:ensure-generic-function 'results-val :lambda-list '(m))
(cl:defmethod results-val ((m <DetectionResults>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection-msg:results-val is deprecated.  Use object_detection-msg:results instead.")
  (results m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectionResults>) ostream)
  "Serializes a message object of type '<DetectionResults>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'results))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'results))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectionResults>) istream)
  "Deserializes a message object of type '<DetectionResults>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'results) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'results)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'object_detection-msg:DetectionResult))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectionResults>)))
  "Returns string type for a message object of type '<DetectionResults>"
  "object_detection/DetectionResults")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectionResults)))
  "Returns string type for a message object of type 'DetectionResults"
  "object_detection/DetectionResults")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectionResults>)))
  "Returns md5sum for a message object of type '<DetectionResults>"
  "c9a646c35e7a13cc2d94d4de60869423")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectionResults)))
  "Returns md5sum for a message object of type 'DetectionResults"
  "c9a646c35e7a13cc2d94d4de60869423")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectionResults>)))
  "Returns full string definition for message of type '<DetectionResults>"
  (cl:format cl:nil "DetectionResult[] results~%================================================================================~%MSG: object_detection/DetectionResult~%uint32 out_class~%float32 out_score~%float32[] location~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectionResults)))
  "Returns full string definition for message of type 'DetectionResults"
  (cl:format cl:nil "DetectionResult[] results~%================================================================================~%MSG: object_detection/DetectionResult~%uint32 out_class~%float32 out_score~%float32[] location~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectionResults>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'results) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectionResults>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectionResults
    (cl:cons ':results (results msg))
))
