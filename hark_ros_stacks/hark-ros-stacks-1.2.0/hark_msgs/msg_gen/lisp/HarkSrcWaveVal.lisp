; Auto-generated. Do not edit!


(cl:in-package hark_msgs-msg)


;//! \htmlinclude HarkSrcWaveVal.msg.html

(cl:defclass <HarkSrcWaveVal> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (power
    :reader power
    :initarg :power
    :type cl:float
    :initform 0.0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0)
   (length
    :reader length
    :initarg :length
    :type cl:integer
    :initform 0)
   (data_bytes
    :reader data_bytes
    :initarg :data_bytes
    :type cl:integer
    :initform 0)
   (wavedata
    :reader wavedata
    :initarg :wavedata
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass HarkSrcWaveVal (<HarkSrcWaveVal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HarkSrcWaveVal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HarkSrcWaveVal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hark_msgs-msg:<HarkSrcWaveVal> is deprecated: use hark_msgs-msg:HarkSrcWaveVal instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <HarkSrcWaveVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:id-val is deprecated.  Use hark_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'power-val :lambda-list '(m))
(cl:defmethod power-val ((m <HarkSrcWaveVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:power-val is deprecated.  Use hark_msgs-msg:power instead.")
  (power m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <HarkSrcWaveVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:x-val is deprecated.  Use hark_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <HarkSrcWaveVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:y-val is deprecated.  Use hark_msgs-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <HarkSrcWaveVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:z-val is deprecated.  Use hark_msgs-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <HarkSrcWaveVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:theta-val is deprecated.  Use hark_msgs-msg:theta instead.")
  (theta m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <HarkSrcWaveVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:length-val is deprecated.  Use hark_msgs-msg:length instead.")
  (length m))

(cl:ensure-generic-function 'data_bytes-val :lambda-list '(m))
(cl:defmethod data_bytes-val ((m <HarkSrcWaveVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:data_bytes-val is deprecated.  Use hark_msgs-msg:data_bytes instead.")
  (data_bytes m))

(cl:ensure-generic-function 'wavedata-val :lambda-list '(m))
(cl:defmethod wavedata-val ((m <HarkSrcWaveVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:wavedata-val is deprecated.  Use hark_msgs-msg:wavedata instead.")
  (wavedata m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HarkSrcWaveVal>) ostream)
  "Serializes a message object of type '<HarkSrcWaveVal>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'power))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'length)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'data_bytes)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HarkSrcWaveVal>) istream)
  "Deserializes a message object of type '<HarkSrcWaveVal>"
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
    (cl:setf (cl:slot-value msg 'power) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'length) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data_bytes) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HarkSrcWaveVal>)))
  "Returns string type for a message object of type '<HarkSrcWaveVal>"
  "hark_msgs/HarkSrcWaveVal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HarkSrcWaveVal)))
  "Returns string type for a message object of type 'HarkSrcWaveVal"
  "hark_msgs/HarkSrcWaveVal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HarkSrcWaveVal>)))
  "Returns md5sum for a message object of type '<HarkSrcWaveVal>"
  "1517c9bd67501893f1b4acd63440fa58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HarkSrcWaveVal)))
  "Returns md5sum for a message object of type 'HarkSrcWaveVal"
  "1517c9bd67501893f1b4acd63440fa58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HarkSrcWaveVal>)))
  "Returns full string definition for message of type '<HarkSrcWaveVal>"
  (cl:format cl:nil "int32 id~%float32 power~%float32 x~%float32 y~%float32 z~%float32 theta~%int32 length~%int32 data_bytes~%float32[] wavedata~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HarkSrcWaveVal)))
  "Returns full string definition for message of type 'HarkSrcWaveVal"
  (cl:format cl:nil "int32 id~%float32 power~%float32 x~%float32 y~%float32 z~%float32 theta~%int32 length~%int32 data_bytes~%float32[] wavedata~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HarkSrcWaveVal>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'wavedata) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HarkSrcWaveVal>))
  "Converts a ROS message object to a list"
  (cl:list 'HarkSrcWaveVal
    (cl:cons ':id (id msg))
    (cl:cons ':power (power msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':theta (theta msg))
    (cl:cons ':length (length msg))
    (cl:cons ':data_bytes (data_bytes msg))
    (cl:cons ':wavedata (wavedata msg))
))
