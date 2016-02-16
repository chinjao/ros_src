; Auto-generated. Do not edit!


(cl:in-package hark_params-msg)


;//! \htmlinclude LocalizeMUSIC.msg.html

(cl:defclass <LocalizeMUSIC> (roslisp-msg-protocol:ros-message)
  ((num_source
    :reader num_source
    :initarg :num_source
    :type cl:integer
    :initform 0)
   (min_deg
    :reader min_deg
    :initarg :min_deg
    :type cl:integer
    :initform 0)
   (max_deg
    :reader max_deg
    :initarg :max_deg
    :type cl:integer
    :initform 0)
   (lower_bound_frequency
    :reader lower_bound_frequency
    :initarg :lower_bound_frequency
    :type cl:integer
    :initform 0)
   (upper_bound_frequency
    :reader upper_bound_frequency
    :initarg :upper_bound_frequency
    :type cl:integer
    :initform 0))
)

(cl:defclass LocalizeMUSIC (<LocalizeMUSIC>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LocalizeMUSIC>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LocalizeMUSIC)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hark_params-msg:<LocalizeMUSIC> is deprecated: use hark_params-msg:LocalizeMUSIC instead.")))

(cl:ensure-generic-function 'num_source-val :lambda-list '(m))
(cl:defmethod num_source-val ((m <LocalizeMUSIC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_params-msg:num_source-val is deprecated.  Use hark_params-msg:num_source instead.")
  (num_source m))

(cl:ensure-generic-function 'min_deg-val :lambda-list '(m))
(cl:defmethod min_deg-val ((m <LocalizeMUSIC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_params-msg:min_deg-val is deprecated.  Use hark_params-msg:min_deg instead.")
  (min_deg m))

(cl:ensure-generic-function 'max_deg-val :lambda-list '(m))
(cl:defmethod max_deg-val ((m <LocalizeMUSIC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_params-msg:max_deg-val is deprecated.  Use hark_params-msg:max_deg instead.")
  (max_deg m))

(cl:ensure-generic-function 'lower_bound_frequency-val :lambda-list '(m))
(cl:defmethod lower_bound_frequency-val ((m <LocalizeMUSIC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_params-msg:lower_bound_frequency-val is deprecated.  Use hark_params-msg:lower_bound_frequency instead.")
  (lower_bound_frequency m))

(cl:ensure-generic-function 'upper_bound_frequency-val :lambda-list '(m))
(cl:defmethod upper_bound_frequency-val ((m <LocalizeMUSIC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_params-msg:upper_bound_frequency-val is deprecated.  Use hark_params-msg:upper_bound_frequency instead.")
  (upper_bound_frequency m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LocalizeMUSIC>) ostream)
  "Serializes a message object of type '<LocalizeMUSIC>"
  (cl:let* ((signed (cl:slot-value msg 'num_source)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'min_deg)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'max_deg)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'lower_bound_frequency)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'upper_bound_frequency)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LocalizeMUSIC>) istream)
  "Deserializes a message object of type '<LocalizeMUSIC>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_source) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'min_deg) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'max_deg) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lower_bound_frequency) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'upper_bound_frequency) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LocalizeMUSIC>)))
  "Returns string type for a message object of type '<LocalizeMUSIC>"
  "hark_params/LocalizeMUSIC")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocalizeMUSIC)))
  "Returns string type for a message object of type 'LocalizeMUSIC"
  "hark_params/LocalizeMUSIC")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LocalizeMUSIC>)))
  "Returns md5sum for a message object of type '<LocalizeMUSIC>"
  "e153c85ee682045538315f87c1a9f5f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LocalizeMUSIC)))
  "Returns md5sum for a message object of type 'LocalizeMUSIC"
  "e153c85ee682045538315f87c1a9f5f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LocalizeMUSIC>)))
  "Returns full string definition for message of type '<LocalizeMUSIC>"
  (cl:format cl:nil "int32 num_source~%int32 min_deg~%int32 max_deg~%int32 lower_bound_frequency~%int32 upper_bound_frequency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LocalizeMUSIC)))
  "Returns full string definition for message of type 'LocalizeMUSIC"
  (cl:format cl:nil "int32 num_source~%int32 min_deg~%int32 max_deg~%int32 lower_bound_frequency~%int32 upper_bound_frequency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LocalizeMUSIC>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LocalizeMUSIC>))
  "Converts a ROS message object to a list"
  (cl:list 'LocalizeMUSIC
    (cl:cons ':num_source (num_source msg))
    (cl:cons ':min_deg (min_deg msg))
    (cl:cons ':max_deg (max_deg msg))
    (cl:cons ':lower_bound_frequency (lower_bound_frequency msg))
    (cl:cons ':upper_bound_frequency (upper_bound_frequency msg))
))
