; Auto-generated. Do not edit!


(cl:in-package hark_msgs-msg)


;//! \htmlinclude HarkFeature.msg.html

(cl:defclass <HarkFeature> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (count
    :reader count
    :initarg :count
    :type cl:integer
    :initform 0)
   (exist_src_num
    :reader exist_src_num
    :initarg :exist_src_num
    :type cl:integer
    :initform 0)
   (src
    :reader src
    :initarg :src
    :type (cl:vector hark_msgs-msg:HarkFeatureVal)
   :initform (cl:make-array 0 :element-type 'hark_msgs-msg:HarkFeatureVal :initial-element (cl:make-instance 'hark_msgs-msg:HarkFeatureVal))))
)

(cl:defclass HarkFeature (<HarkFeature>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HarkFeature>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HarkFeature)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hark_msgs-msg:<HarkFeature> is deprecated: use hark_msgs-msg:HarkFeature instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HarkFeature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:header-val is deprecated.  Use hark_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <HarkFeature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:count-val is deprecated.  Use hark_msgs-msg:count instead.")
  (count m))

(cl:ensure-generic-function 'exist_src_num-val :lambda-list '(m))
(cl:defmethod exist_src_num-val ((m <HarkFeature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:exist_src_num-val is deprecated.  Use hark_msgs-msg:exist_src_num instead.")
  (exist_src_num m))

(cl:ensure-generic-function 'src-val :lambda-list '(m))
(cl:defmethod src-val ((m <HarkFeature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:src-val is deprecated.  Use hark_msgs-msg:src instead.")
  (src m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HarkFeature>) ostream)
  "Serializes a message object of type '<HarkFeature>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'count)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'exist_src_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'src))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'src))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HarkFeature>) istream)
  "Deserializes a message object of type '<HarkFeature>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'count) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'exist_src_num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'src) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'src)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'hark_msgs-msg:HarkFeatureVal))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HarkFeature>)))
  "Returns string type for a message object of type '<HarkFeature>"
  "hark_msgs/HarkFeature")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HarkFeature)))
  "Returns string type for a message object of type 'HarkFeature"
  "hark_msgs/HarkFeature")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HarkFeature>)))
  "Returns md5sum for a message object of type '<HarkFeature>"
  "e61d4af0b817ab09bd341b909e7c1ed9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HarkFeature)))
  "Returns md5sum for a message object of type 'HarkFeature"
  "e61d4af0b817ab09bd341b909e7c1ed9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HarkFeature>)))
  "Returns full string definition for message of type '<HarkFeature>"
  (cl:format cl:nil "Header header~%int32 count~%int32 exist_src_num~%HarkFeatureVal[] src  # Feature of each source~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: hark_msgs/HarkFeatureVal~%int32 id~%int32 length~%int32 data_bytes~%float32[] featuredata~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HarkFeature)))
  "Returns full string definition for message of type 'HarkFeature"
  (cl:format cl:nil "Header header~%int32 count~%int32 exist_src_num~%HarkFeatureVal[] src  # Feature of each source~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: hark_msgs/HarkFeatureVal~%int32 id~%int32 length~%int32 data_bytes~%float32[] featuredata~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HarkFeature>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'src) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HarkFeature>))
  "Converts a ROS message object to a list"
  (cl:list 'HarkFeature
    (cl:cons ':header (header msg))
    (cl:cons ':count (count msg))
    (cl:cons ':exist_src_num (exist_src_num msg))
    (cl:cons ':src (src msg))
))
