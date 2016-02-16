; Auto-generated. Do not edit!


(cl:in-package hark_msgs-msg)


;//! \htmlinclude HarkJulius.msg.html

(cl:defclass <HarkJulius> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (src
    :reader src
    :initarg :src
    :type (cl:vector hark_msgs-msg:HarkJuliusSrc)
   :initform (cl:make-array 0 :element-type 'hark_msgs-msg:HarkJuliusSrc :initial-element (cl:make-instance 'hark_msgs-msg:HarkJuliusSrc))))
)

(cl:defclass HarkJulius (<HarkJulius>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HarkJulius>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HarkJulius)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hark_msgs-msg:<HarkJulius> is deprecated: use hark_msgs-msg:HarkJulius instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HarkJulius>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:header-val is deprecated.  Use hark_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'src-val :lambda-list '(m))
(cl:defmethod src-val ((m <HarkJulius>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:src-val is deprecated.  Use hark_msgs-msg:src instead.")
  (src m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HarkJulius>) ostream)
  "Serializes a message object of type '<HarkJulius>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'src))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'src))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HarkJulius>) istream)
  "Deserializes a message object of type '<HarkJulius>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'src) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'src)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'hark_msgs-msg:HarkJuliusSrc))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HarkJulius>)))
  "Returns string type for a message object of type '<HarkJulius>"
  "hark_msgs/HarkJulius")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HarkJulius)))
  "Returns string type for a message object of type 'HarkJulius"
  "hark_msgs/HarkJulius")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HarkJulius>)))
  "Returns md5sum for a message object of type '<HarkJulius>"
  "18ac98bafa65529bae1ef5042e324284")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HarkJulius)))
  "Returns md5sum for a message object of type 'HarkJulius"
  "18ac98bafa65529bae1ef5042e324284")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HarkJulius>)))
  "Returns full string definition for message of type '<HarkJulius>"
  (cl:format cl:nil "Header header~%HarkJuliusSrc[] src # Detail of Julius recognition result~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: hark_msgs/HarkJuliusSrc~%int32 id~%float32 azimuth~%float32 elevation~%int32 sec~%int32 usec~%int32 frames~%int32 msec~%int8 status            # status of recognition process defined below~%HarkJuliusSrcVal[] src # WHYPO results for each source~%~%int8 TERMINATED=0~%int8 STARTRECOG=1~%int8 INPUTPARAM=2~%int8 SOURCEINFO=3~%int8 RECOGOUT=4~%int8 RECOGFAIL=5~%~%================================================================================~%MSG: hark_msgs/HarkJuliusSrcVal~%string word~%int32 classid~%string phone~%float32 cm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HarkJulius)))
  "Returns full string definition for message of type 'HarkJulius"
  (cl:format cl:nil "Header header~%HarkJuliusSrc[] src # Detail of Julius recognition result~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: hark_msgs/HarkJuliusSrc~%int32 id~%float32 azimuth~%float32 elevation~%int32 sec~%int32 usec~%int32 frames~%int32 msec~%int8 status            # status of recognition process defined below~%HarkJuliusSrcVal[] src # WHYPO results for each source~%~%int8 TERMINATED=0~%int8 STARTRECOG=1~%int8 INPUTPARAM=2~%int8 SOURCEINFO=3~%int8 RECOGOUT=4~%int8 RECOGFAIL=5~%~%================================================================================~%MSG: hark_msgs/HarkJuliusSrcVal~%string word~%int32 classid~%string phone~%float32 cm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HarkJulius>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'src) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HarkJulius>))
  "Converts a ROS message object to a list"
  (cl:list 'HarkJulius
    (cl:cons ':header (header msg))
    (cl:cons ':src (src msg))
))
