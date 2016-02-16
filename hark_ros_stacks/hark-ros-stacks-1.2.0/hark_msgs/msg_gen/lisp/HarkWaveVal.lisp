; Auto-generated. Do not edit!


(cl:in-package hark_msgs-msg)


;//! \htmlinclude HarkWaveVal.msg.html

(cl:defclass <HarkWaveVal> (roslisp-msg-protocol:ros-message)
  ((wavedata
    :reader wavedata
    :initarg :wavedata
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass HarkWaveVal (<HarkWaveVal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HarkWaveVal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HarkWaveVal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hark_msgs-msg:<HarkWaveVal> is deprecated: use hark_msgs-msg:HarkWaveVal instead.")))

(cl:ensure-generic-function 'wavedata-val :lambda-list '(m))
(cl:defmethod wavedata-val ((m <HarkWaveVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:wavedata-val is deprecated.  Use hark_msgs-msg:wavedata instead.")
  (wavedata m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HarkWaveVal>) ostream)
  "Serializes a message object of type '<HarkWaveVal>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'wavedata))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'wavedata))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HarkWaveVal>) istream)
  "Deserializes a message object of type '<HarkWaveVal>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'wavedata) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'wavedata)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HarkWaveVal>)))
  "Returns string type for a message object of type '<HarkWaveVal>"
  "hark_msgs/HarkWaveVal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HarkWaveVal)))
  "Returns string type for a message object of type 'HarkWaveVal"
  "hark_msgs/HarkWaveVal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HarkWaveVal>)))
  "Returns md5sum for a message object of type '<HarkWaveVal>"
  "6d4ffa9fffe078365bb941098c16cdfd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HarkWaveVal)))
  "Returns md5sum for a message object of type 'HarkWaveVal"
  "6d4ffa9fffe078365bb941098c16cdfd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HarkWaveVal>)))
  "Returns full string definition for message of type '<HarkWaveVal>"
  (cl:format cl:nil "float32[] wavedata~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HarkWaveVal)))
  "Returns full string definition for message of type 'HarkWaveVal"
  (cl:format cl:nil "float32[] wavedata~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HarkWaveVal>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'wavedata) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HarkWaveVal>))
  "Converts a ROS message object to a list"
  (cl:list 'HarkWaveVal
    (cl:cons ':wavedata (wavedata msg))
))
