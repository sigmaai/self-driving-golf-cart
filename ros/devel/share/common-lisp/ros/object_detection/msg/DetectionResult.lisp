; Auto-generated. Do not edit!


(cl:in-package object_detection-msg)


;//! \htmlinclude DetectionResult.msg.html

(cl:defclass <DetectionResult> (roslisp-msg-protocol:ros-message)
  ((out_class
    :reader out_class
    :initarg :out_class
    :type cl:string
    :initform "")
   (out_score
    :reader out_score
    :initarg :out_score
    :type cl:float
    :initform 0.0)
   (upper_left
    :reader upper_left
    :initarg :upper_left
    :type cl:integer
    :initform 0)
   (upper_right
    :reader upper_right
    :initarg :upper_right
    :type cl:integer
    :initform 0)
   (lower_left
    :reader lower_left
    :initarg :lower_left
    :type cl:integer
    :initform 0)
   (lower_right
    :reader lower_right
    :initarg :lower_right
    :type cl:integer
    :initform 0))
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

(cl:ensure-generic-function 'upper_left-val :lambda-list '(m))
(cl:defmethod upper_left-val ((m <DetectionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection-msg:upper_left-val is deprecated.  Use object_detection-msg:upper_left instead.")
  (upper_left m))

(cl:ensure-generic-function 'upper_right-val :lambda-list '(m))
(cl:defmethod upper_right-val ((m <DetectionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection-msg:upper_right-val is deprecated.  Use object_detection-msg:upper_right instead.")
  (upper_right m))

(cl:ensure-generic-function 'lower_left-val :lambda-list '(m))
(cl:defmethod lower_left-val ((m <DetectionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection-msg:lower_left-val is deprecated.  Use object_detection-msg:lower_left instead.")
  (lower_left m))

(cl:ensure-generic-function 'lower_right-val :lambda-list '(m))
(cl:defmethod lower_right-val ((m <DetectionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection-msg:lower_right-val is deprecated.  Use object_detection-msg:lower_right instead.")
  (lower_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectionResult>) ostream)
  "Serializes a message object of type '<DetectionResult>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'out_class))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'out_class))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'out_score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'upper_left)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'upper_left)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'upper_left)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'upper_left)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'upper_right)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'upper_right)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'upper_right)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'upper_right)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lower_left)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'lower_left)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'lower_left)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'lower_left)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lower_right)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'lower_right)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'lower_right)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'lower_right)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectionResult>) istream)
  "Deserializes a message object of type '<DetectionResult>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'out_class) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'out_class) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'out_score) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'upper_left)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'upper_left)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'upper_left)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'upper_left)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'upper_right)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'upper_right)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'upper_right)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'upper_right)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lower_left)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'lower_left)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'lower_left)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'lower_left)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lower_right)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'lower_right)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'lower_right)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'lower_right)) (cl:read-byte istream))
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
  "5fb792480f070cbc69af8da53d614697")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectionResult)))
  "Returns md5sum for a message object of type 'DetectionResult"
  "5fb792480f070cbc69af8da53d614697")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectionResult>)))
  "Returns full string definition for message of type '<DetectionResult>"
  (cl:format cl:nil "string out_class~%float32 out_score~%uint32 upper_left~%uint32 upper_right~%uint32 lower_left~%uint32 lower_right~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectionResult)))
  "Returns full string definition for message of type 'DetectionResult"
  (cl:format cl:nil "string out_class~%float32 out_score~%uint32 upper_left~%uint32 upper_right~%uint32 lower_left~%uint32 lower_right~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectionResult>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'out_class))
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectionResult>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectionResult
    (cl:cons ':out_class (out_class msg))
    (cl:cons ':out_score (out_score msg))
    (cl:cons ':upper_left (upper_left msg))
    (cl:cons ':upper_right (upper_right msg))
    (cl:cons ':lower_left (lower_left msg))
    (cl:cons ':lower_right (lower_right msg))
))
