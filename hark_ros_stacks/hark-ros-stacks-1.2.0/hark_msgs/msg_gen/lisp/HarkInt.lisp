; Auto-generated. Do not edit!


(cl:in-package hark_msgs-msg)


;//! \htmlinclude HarkInt.msg.html

(cl:defclass <HarkInt> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:integer
    :initform 0))
)

(cl:defclass HarkInt (<HarkInt>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HarkInt>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HarkInt)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hark_msgs-msg:<HarkInt> is deprecated: use hark_msgs-msg:HarkInt instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <HarkInt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:data-val is deprecated.  Use hark_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HarkInt>) ostream)
  "Serializes a message object of type '<HarkInt>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HarkInt>) istream)
  "Deserializes a message object of type '<HarkInt>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HarkInt>)))
  "Returns string type for a message object of type '<HarkInt>"
  "hark_msgs/HarkInt")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HarkInt)))
  "Returns string type for a message object of type 'HarkInt"
  "hark_msgs/HarkInt")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HarkInt>)))
  "Returns md5sum for a message object of type '<HarkInt>"
  "34add168574510e6e17f5d23ecc077ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HarkInt)))
  "Returns md5sum for a message object of type 'HarkInt"
  "34add168574510e6e17f5d23ecc077ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HarkInt>)))
  "Returns full string definition for message of type '<HarkInt>"
  (cl:format cl:nil "int64 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HarkInt)))
  "Returns full string definition for message of type 'HarkInt"
  (cl:format cl:nil "int64 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HarkInt>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HarkInt>))
  "Converts a ROS message object to a list"
  (cl:list 'HarkInt
    (cl:cons ':data (data msg))
))
