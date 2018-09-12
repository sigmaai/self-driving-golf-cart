; Auto-generated. Do not edit!


(cl:in-package object_detection-msg)


;//! \htmlinclude DetectionResult.msg.html

(cl:defclass <DetectionResult> (roslisp-msg-protocol:ros-message)
  ((out_class
    :reader out_class
    :initarg :out_class
    :type cl:integer
    :initform 0)
   (out_score
    :reader out_score
    :initarg :out_score
    :type cl:float
    :initform 0.0)
   (location
    :reader location
    :initarg :location
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass DetectionResult (<DetectionResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectionResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectionResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_detection-msg:<DetectionResult> is deprecated: use object_detection-msg:DetectionResult instead.")))

(cl:ensure-generic-function 'out_class-val :lambda-list '(m))
(cl:defmethod out_class-val ((m <DetectionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection-msg:out_class-val is deprecated.  Use object_detection-msg:out_class instead.")
  (out_class m))

(cl:ensure-generic-function 'out_score-val :lambda-list '(m))
(cl:defmethod out_score-val ((m <DetectionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection-msg:out_score-val is deprecated.  Use object_detection-msg:out_score instead.")
  (out_score m))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <DetectionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection-msg:location-val is deprecated.  Use object_detection-msg:location instead.")
  (location m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectionResult>) ostream)
  "Serializes a message object of type '<DetectionResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'out_class)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'out_class)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'out_class)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'out_class)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'out_score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'location))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'location))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectionResult>) istream)
  "Deserializes a message object of type '<DetectionResult>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'out_class)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'out_class)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'out_class)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'out_class)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'out_score) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'location) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'location)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectionResult>)))
  "Returns string type for a message object of type '<DetectionResult>"
  "object_detection/DetectionResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectionResult)))
  "Returns string type for a message object of type 'DetectionResult"
  "object_detection/DetectionResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectionResult>)))
  "Returns md5sum for a message object of type '<DetectionResult>"
  "3b7fae78ce58faba54a3eab2d7ca9e3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectionResult)))
  "Returns md5sum for a message object of type 'DetectionResult"
  "3b7fae78ce58faba54a3eab2d7ca9e3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectionResult>)))
  "Returns full string definition for message of type '<DetectionResult>"
  (cl:format cl:nil "uint32 out_class~%float32 out_score~%float32[] location~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectionResult)))
  "Returns full string definition for message of type 'DetectionResult"
  (cl:format cl:nil "uint32 out_class~%float32 out_score~%float32[] location~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectionResult>))
  (cl:+ 0
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'location) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectionResult>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectionResult
    (cl:cons ':out_class (out_class msg))
    (cl:cons ':out_score (out_score msg))
    (cl:cons ':location (location msg))
))
