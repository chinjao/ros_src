; Auto-generated. Do not edit!


(cl:in-package hark_msgs-msg)


;//! \htmlinclude HarkJuliusSrc.msg.html

(cl:defclass <HarkJuliusSrc> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (azimuth
    :reader azimuth
    :initarg :azimuth
    :type cl:float
    :initform 0.0)
   (elevation
    :reader elevation
    :initarg :elevation
    :type cl:float
    :initform 0.0)
   (sec
    :reader sec
    :initarg :sec
    :type cl:integer
    :initform 0)
   (usec
    :reader usec
    :initarg :usec
    :type cl:integer
    :initform 0)
   (frames
    :reader frames
    :initarg :frames
    :type cl:integer
    :initform 0)
   (msec
    :reader msec
    :initarg :msec
    :type cl:integer
    :initform 0)
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (src
    :reader src
    :initarg :src
    :type (cl:vector hark_msgs-msg:HarkJuliusSrcVal)
   :initform (cl:make-array 0 :element-type 'hark_msgs-msg:HarkJuliusSrcVal :initial-element (cl:make-instance 'hark_msgs-msg:HarkJuliusSrcVal))))
)

(cl:defclass HarkJuliusSrc (<HarkJuliusSrc>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HarkJuliusSrc>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HarkJuliusSrc)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hark_msgs-msg:<HarkJuliusSrc> is deprecated: use hark_msgs-msg:HarkJuliusSrc instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <HarkJuliusSrc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:id-val is deprecated.  Use hark_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'azimuth-val :lambda-list '(m))
(cl:defmethod azimuth-val ((m <HarkJuliusSrc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:azimuth-val is deprecated.  Use hark_msgs-msg:azimuth instead.")
  (azimuth m))

(cl:ensure-generic-function 'elevation-val :lambda-list '(m))
(cl:defmethod elevation-val ((m <HarkJuliusSrc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:elevation-val is deprecated.  Use hark_msgs-msg:elevation instead.")
  (elevation m))

(cl:ensure-generic-function 'sec-val :lambda-list '(m))
(cl:defmethod sec-val ((m <HarkJuliusSrc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:sec-val is deprecated.  Use hark_msgs-msg:sec instead.")
  (sec m))

(cl:ensure-generic-function 'usec-val :lambda-list '(m))
(cl:defmethod usec-val ((m <HarkJuliusSrc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:usec-val is deprecated.  Use hark_msgs-msg:usec instead.")
  (usec m))

(cl:ensure-generic-function 'frames-val :lambda-list '(m))
(cl:defmethod frames-val ((m <HarkJuliusSrc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:frames-val is deprecated.  Use hark_msgs-msg:frames instead.")
  (frames m))

(cl:ensure-generic-function 'msec-val :lambda-list '(m))
(cl:defmethod msec-val ((m <HarkJuliusSrc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:msec-val is deprecated.  Use hark_msgs-msg:msec instead.")
  (msec m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <HarkJuliusSrc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:status-val is deprecated.  Use hark_msgs-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'src-val :lambda-list '(m))
(cl:defmethod src-val ((m <HarkJuliusSrc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:src-val is deprecated.  Use hark_msgs-msg:src instead.")
  (src m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<HarkJuliusSrc>)))
    "Constants for message type '<HarkJuliusSrc>"
  '((:TERMINATED . 0)
    (:STARTRECOG . 1)
    (:INPUTPARAM . 2)
    (:SOURCEINFO . 3)
    (:RECOGOUT . 4)
    (:RECOGFAIL . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'HarkJuliusSrc)))
    "Constants for message type 'HarkJuliusSrc"
  '((:TERMINATED . 0)
    (:STARTRECOG . 1)
    (:INPUTPARAM . 2)
    (:SOURCEINFO . 3)
    (:RECOGOUT . 4)
    (:RECOGFAIL . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HarkJuliusSrc>) ostream)
  "Serializes a message object of type '<HarkJuliusSrc>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'azimuth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'elevation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'sec)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'usec)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'frames)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'msec)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'src))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'src))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HarkJuliusSrc>) istream)
  "Deserializes a message object of type '<HarkJuliusSrc>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'azimuth) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'elevation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sec) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'usec) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frames) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msec) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'src) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'src)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'hark_msgs-msg:HarkJuliusSrcVal))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HarkJuliusSrc>)))
  "Returns string type for a message object of type '<HarkJuliusSrc>"
  "hark_msgs/HarkJuliusSrc")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HarkJuliusSrc)))
  "Returns string type for a message object of type 'HarkJuliusSrc"
  "hark_msgs/HarkJuliusSrc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HarkJuliusSrc>)))
  "Returns md5sum for a message object of type '<HarkJuliusSrc>"
  "1aff8127ead3951895e7fd2d0cb19ac9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HarkJuliusSrc)))
  "Returns md5sum for a message object of type 'HarkJuliusSrc"
  "1aff8127ead3951895e7fd2d0cb19ac9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HarkJuliusSrc>)))
  "Returns full string definition for message of type '<HarkJuliusSrc>"
  (cl:format cl:nil "int32 id~%float32 azimuth~%float32 elevation~%int32 sec~%int32 usec~%int32 frames~%int32 msec~%int8 status            # status of recognition process defined below~%HarkJuliusSrcVal[] src # WHYPO results for each source~%~%int8 TERMINATED=0~%int8 STARTRECOG=1~%int8 INPUTPARAM=2~%int8 SOURCEINFO=3~%int8 RECOGOUT=4~%int8 RECOGFAIL=5~%~%================================================================================~%MSG: hark_msgs/HarkJuliusSrcVal~%string word~%int32 classid~%string phone~%float32 cm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HarkJuliusSrc)))
  "Returns full string definition for message of type 'HarkJuliusSrc"
  (cl:format cl:nil "int32 id~%float32 azimuth~%float32 elevation~%int32 sec~%int32 usec~%int32 frames~%int32 msec~%int8 status            # status of recognition process defined below~%HarkJuliusSrcVal[] src # WHYPO results for each source~%~%int8 TERMINATED=0~%int8 STARTRECOG=1~%int8 INPUTPARAM=2~%int8 SOURCEINFO=3~%int8 RECOGOUT=4~%int8 RECOGFAIL=5~%~%================================================================================~%MSG: hark_msgs/HarkJuliusSrcVal~%string word~%int32 classid~%string phone~%float32 cm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HarkJuliusSrc>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'src) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HarkJuliusSrc>))
  "Converts a ROS message object to a list"
  (cl:list 'HarkJuliusSrc
    (cl:cons ':id (id msg))
    (cl:cons ':azimuth (azimuth msg))
    (cl:cons ':elevation (elevation msg))
    (cl:cons ':sec (sec msg))
    (cl:cons ':usec (usec msg))
    (cl:cons ':frames (frames msg))
    (cl:cons ':msec (msec msg))
    (cl:cons ':status (status msg))
    (cl:cons ':src (src msg))
))
