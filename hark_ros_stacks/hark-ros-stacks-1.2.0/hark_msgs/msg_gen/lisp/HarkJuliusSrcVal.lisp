; Auto-generated. Do not edit!


(cl:in-package hark_msgs-msg)


;//! \htmlinclude HarkJuliusSrcVal.msg.html

(cl:defclass <HarkJuliusSrcVal> (roslisp-msg-protocol:ros-message)
  ((word
    :reader word
    :initarg :word
    :type cl:string
    :initform "")
   (classid
    :reader classid
    :initarg :classid
    :type cl:integer
    :initform 0)
   (phone
    :reader phone
    :initarg :phone
    :type cl:string
    :initform "")
   (cm
    :reader cm
    :initarg :cm
    :type cl:float
    :initform 0.0))
)

(cl:defclass HarkJuliusSrcVal (<HarkJuliusSrcVal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HarkJuliusSrcVal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HarkJuliusSrcVal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hark_msgs-msg:<HarkJuliusSrcVal> is deprecated: use hark_msgs-msg:HarkJuliusSrcVal instead.")))

(cl:ensure-generic-function 'word-val :lambda-list '(m))
(cl:defmethod word-val ((m <HarkJuliusSrcVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:word-val is deprecated.  Use hark_msgs-msg:word instead.")
  (word m))

(cl:ensure-generic-function 'classid-val :lambda-list '(m))
(cl:defmethod classid-val ((m <HarkJuliusSrcVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:classid-val is deprecated.  Use hark_msgs-msg:classid instead.")
  (classid m))

(cl:ensure-generic-function 'phone-val :lambda-list '(m))
(cl:defmethod phone-val ((m <HarkJuliusSrcVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:phone-val is deprecated.  Use hark_msgs-msg:phone instead.")
  (phone m))

(cl:ensure-generic-function 'cm-val :lambda-list '(m))
(cl:defmethod cm-val ((m <HarkJuliusSrcVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:cm-val is deprecated.  Use hark_msgs-msg:cm instead.")
  (cm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HarkJuliusSrcVal>) ostream)
  "Serializes a message object of type '<HarkJuliusSrcVal>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'word))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'word))
  (cl:let* ((signed (cl:slot-value msg 'classid)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'phone))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'phone))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HarkJuliusSrcVal>) istream)
  "Deserializes a message object of type '<HarkJuliusSrcVal>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'word) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'word) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'classid) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'phone) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'phone) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cm) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HarkJuliusSrcVal>)))
  "Returns string type for a message object of type '<HarkJuliusSrcVal>"
  "hark_msgs/HarkJuliusSrcVal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HarkJuliusSrcVal)))
  "Returns string type for a message object of type 'HarkJuliusSrcVal"
  "hark_msgs/HarkJuliusSrcVal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HarkJuliusSrcVal>)))
  "Returns md5sum for a message object of type '<HarkJuliusSrcVal>"
  "d68fa7d47b805f4384367952f7f8407e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HarkJuliusSrcVal)))
  "Returns md5sum for a message object of type 'HarkJuliusSrcVal"
  "d68fa7d47b805f4384367952f7f8407e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HarkJuliusSrcVal>)))
  "Returns full string definition for message of type '<HarkJuliusSrcVal>"
  (cl:format cl:nil "string word~%int32 classid~%string phone~%float32 cm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HarkJuliusSrcVal)))
  "Returns full string definition for message of type 'HarkJuliusSrcVal"
  (cl:format cl:nil "string word~%int32 classid~%string phone~%float32 cm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HarkJuliusSrcVal>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'word))
     4
     4 (cl:length (cl:slot-value msg 'phone))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HarkJuliusSrcVal>))
  "Converts a ROS message object to a list"
  (cl:list 'HarkJuliusSrcVal
    (cl:cons ':word (word msg))
    (cl:cons ':classid (classid msg))
    (cl:cons ':phone (phone msg))
    (cl:cons ':cm (cm msg))
))
