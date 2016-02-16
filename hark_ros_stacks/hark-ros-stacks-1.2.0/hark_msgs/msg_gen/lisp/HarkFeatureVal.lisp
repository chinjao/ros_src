; Auto-generated. Do not edit!


(cl:in-package hark_msgs-msg)


;//! \htmlinclude HarkFeatureVal.msg.html

(cl:defclass <HarkFeatureVal> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
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
   (featuredata
    :reader featuredata
    :initarg :featuredata
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass HarkFeatureVal (<HarkFeatureVal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HarkFeatureVal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HarkFeatureVal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hark_msgs-msg:<HarkFeatureVal> is deprecated: use hark_msgs-msg:HarkFeatureVal instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <HarkFeatureVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:id-val is deprecated.  Use hark_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <HarkFeatureVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:length-val is deprecated.  Use hark_msgs-msg:length instead.")
  (length m))

(cl:ensure-generic-function 'data_bytes-val :lambda-list '(m))
(cl:defmethod data_bytes-val ((m <HarkFeatureVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:data_bytes-val is deprecated.  Use hark_msgs-msg:data_bytes instead.")
  (data_bytes m))

(cl:ensure-generic-function 'featuredata-val :lambda-list '(m))
(cl:defmethod featuredata-val ((m <HarkFeatureVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:featuredata-val is deprecated.  Use hark_msgs-msg:featuredata instead.")
  (featuredata m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HarkFeatureVal>) ostream)
  "Serializes a message object of type '<HarkFeatureVal>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
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
  (cl:let* ((signed (cl:slot-value msg 'data_bytes)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'featuredata))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'featuredata))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HarkFeatureVal>) istream)
  "Deserializes a message object of type '<HarkFeatureVal>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
  (cl:setf (cl:slot-value msg 'featuredata) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'featuredata)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HarkFeatureVal>)))
  "Returns string type for a message object of type '<HarkFeatureVal>"
  "hark_msgs/HarkFeatureVal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HarkFeatureVal)))
  "Returns string type for a message object of type 'HarkFeatureVal"
  "hark_msgs/HarkFeatureVal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HarkFeatureVal>)))
  "Returns md5sum for a message object of type '<HarkFeatureVal>"
  "a91db0f9f140ce90dece0bff7d606075")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HarkFeatureVal)))
  "Returns md5sum for a message object of type 'HarkFeatureVal"
  "a91db0f9f140ce90dece0bff7d606075")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HarkFeatureVal>)))
  "Returns full string definition for message of type '<HarkFeatureVal>"
  (cl:format cl:nil "int32 id~%int32 length~%int32 data_bytes~%float32[] featuredata~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HarkFeatureVal)))
  "Returns full string definition for message of type 'HarkFeatureVal"
  (cl:format cl:nil "int32 id~%int32 length~%int32 data_bytes~%float32[] featuredata~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HarkFeatureVal>))
  (cl:+ 0
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'featuredata) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HarkFeatureVal>))
  "Converts a ROS message object to a list"
  (cl:list 'HarkFeatureVal
    (cl:cons ':id (id msg))
    (cl:cons ':length (length msg))
    (cl:cons ':data_bytes (data_bytes msg))
    (cl:cons ':featuredata (featuredata msg))
))
