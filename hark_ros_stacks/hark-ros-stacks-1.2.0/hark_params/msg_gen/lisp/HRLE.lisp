; Auto-generated. Do not edit!


(cl:in-package hark_params-msg)


;//! \htmlinclude HRLE.msg.html

(cl:defclass <HRLE> (roslisp-msg-protocol:ros-message)
  ((lx
    :reader lx
    :initarg :lx
    :type cl:float
    :initform 0.0)
   (time_constant
    :reader time_constant
    :initarg :time_constant
    :type cl:float
    :initform 0.0))
)

(cl:defclass HRLE (<HRLE>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HRLE>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HRLE)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hark_params-msg:<HRLE> is deprecated: use hark_params-msg:HRLE instead.")))

(cl:ensure-generic-function 'lx-val :lambda-list '(m))
(cl:defmethod lx-val ((m <HRLE>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_params-msg:lx-val is deprecated.  Use hark_params-msg:lx instead.")
  (lx m))

(cl:ensure-generic-function 'time_constant-val :lambda-list '(m))
(cl:defmethod time_constant-val ((m <HRLE>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_params-msg:time_constant-val is deprecated.  Use hark_params-msg:time_constant instead.")
  (time_constant m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HRLE>) ostream)
  "Serializes a message object of type '<HRLE>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'time_constant))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HRLE>) istream)
  "Deserializes a message object of type '<HRLE>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time_constant) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HRLE>)))
  "Returns string type for a message object of type '<HRLE>"
  "hark_params/HRLE")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HRLE)))
  "Returns string type for a message object of type 'HRLE"
  "hark_params/HRLE")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HRLE>)))
  "Returns md5sum for a message object of type '<HRLE>"
  "d573ce11b1610cdf85e2c98f807f43dc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HRLE)))
  "Returns md5sum for a message object of type 'HRLE"
  "d573ce11b1610cdf85e2c98f807f43dc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HRLE>)))
  "Returns full string definition for message of type '<HRLE>"
  (cl:format cl:nil "float32 lx~%float32 time_constant~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HRLE)))
  "Returns full string definition for message of type 'HRLE"
  (cl:format cl:nil "float32 lx~%float32 time_constant~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HRLE>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HRLE>))
  "Converts a ROS message object to a list"
  (cl:list 'HRLE
    (cl:cons ':lx (lx msg))
    (cl:cons ':time_constant (time_constant msg))
))
