; Auto-generated. Do not edit!


(cl:in-package hark_msgs-msg)


;//! \htmlinclude HarkFFT.msg.html

(cl:defclass <HarkFFT> (roslisp-msg-protocol:ros-message)
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
   (nch
    :reader nch
    :initarg :nch
    :type cl:integer
    :initform 0)
   (length
    :reader length
    :initarg :length
    :type cl:integer
    :initform 0)
   (src
    :reader src
    :initarg :src
    :type (cl:vector hark_msgs-msg:HarkFFTVal)
   :initform (cl:make-array 0 :element-type 'hark_msgs-msg:HarkFFTVal :initial-element (cl:make-instance 'hark_msgs-msg:HarkFFTVal))))
)

(cl:defclass HarkFFT (<HarkFFT>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HarkFFT>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HarkFFT)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hark_msgs-msg:<HarkFFT> is deprecated: use hark_msgs-msg:HarkFFT instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HarkFFT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:header-val is deprecated.  Use hark_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <HarkFFT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:count-val is deprecated.  Use hark_msgs-msg:count instead.")
  (count m))

(cl:ensure-generic-function 'nch-val :lambda-list '(m))
(cl:defmethod nch-val ((m <HarkFFT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:nch-val is deprecated.  Use hark_msgs-msg:nch instead.")
  (nch m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <HarkFFT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:length-val is deprecated.  Use hark_msgs-msg:length instead.")
  (length m))

(cl:ensure-generic-function 'src-val :lambda-list '(m))
(cl:defmethod src-val ((m <HarkFFT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:src-val is deprecated.  Use hark_msgs-msg:src instead.")
  (src m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HarkFFT>) ostream)
  "Serializes a message object of type '<HarkFFT>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'count)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'nch)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'length)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HarkFFT>) istream)
  "Deserializes a message object of type '<HarkFFT>"
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
      (cl:setf (cl:slot-value msg 'nch) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'length) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'src) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'src)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'hark_msgs-msg:HarkFFTVal))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HarkFFT>)))
  "Returns string type for a message object of type '<HarkFFT>"
  "hark_msgs/HarkFFT")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HarkFFT)))
  "Returns string type for a message object of type 'HarkFFT"
  "hark_msgs/HarkFFT")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HarkFFT>)))
  "Returns md5sum for a message object of type '<HarkFFT>"
  "8aab5c2ab0dcabc23cdf16e2742a6c6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HarkFFT)))
  "Returns md5sum for a message object of type 'HarkFFT"
  "8aab5c2ab0dcabc23cdf16e2742a6c6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HarkFFT>)))
  "Returns full string definition for message of type '<HarkFFT>"
  (cl:format cl:nil "Header header~%int32 count~%int32 nch~%int32 length~%HarkFFTVal[] src # FFT signal of each channel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: hark_msgs/HarkFFTVal~%float32[] fftdata_real~%float32[] fftdata_imag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HarkFFT)))
  "Returns full string definition for message of type 'HarkFFT"
  (cl:format cl:nil "Header header~%int32 count~%int32 nch~%int32 length~%HarkFFTVal[] src # FFT signal of each channel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: hark_msgs/HarkFFTVal~%float32[] fftdata_real~%float32[] fftdata_imag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HarkFFT>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'src) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HarkFFT>))
  "Converts a ROS message object to a list"
  (cl:list 'HarkFFT
    (cl:cons ':header (header msg))
    (cl:cons ':count (count msg))
    (cl:cons ':nch (nch msg))
    (cl:cons ':length (length msg))
    (cl:cons ':src (src msg))
))
