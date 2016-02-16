; Auto-generated. Do not edit!


(cl:in-package hark_msgs-msg)


;//! \htmlinclude HarkFFTVal.msg.html

(cl:defclass <HarkFFTVal> (roslisp-msg-protocol:ros-message)
  ((fftdata_real
    :reader fftdata_real
    :initarg :fftdata_real
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (fftdata_imag
    :reader fftdata_imag
    :initarg :fftdata_imag
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass HarkFFTVal (<HarkFFTVal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HarkFFTVal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HarkFFTVal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hark_msgs-msg:<HarkFFTVal> is deprecated: use hark_msgs-msg:HarkFFTVal instead.")))

(cl:ensure-generic-function 'fftdata_real-val :lambda-list '(m))
(cl:defmethod fftdata_real-val ((m <HarkFFTVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:fftdata_real-val is deprecated.  Use hark_msgs-msg:fftdata_real instead.")
  (fftdata_real m))

(cl:ensure-generic-function 'fftdata_imag-val :lambda-list '(m))
(cl:defmethod fftdata_imag-val ((m <HarkFFTVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hark_msgs-msg:fftdata_imag-val is deprecated.  Use hark_msgs-msg:fftdata_imag instead.")
  (fftdata_imag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HarkFFTVal>) ostream)
  "Serializes a message object of type '<HarkFFTVal>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'fftdata_real))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'fftdata_real))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'fftdata_imag))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'fftdata_imag))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HarkFFTVal>) istream)
  "Deserializes a message object of type '<HarkFFTVal>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'fftdata_real) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'fftdata_real)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'fftdata_imag) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'fftdata_imag)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HarkFFTVal>)))
  "Returns string type for a message object of type '<HarkFFTVal>"
  "hark_msgs/HarkFFTVal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HarkFFTVal)))
  "Returns string type for a message object of type 'HarkFFTVal"
  "hark_msgs/HarkFFTVal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HarkFFTVal>)))
  "Returns md5sum for a message object of type '<HarkFFTVal>"
  "a7b9ef54d8fd113072a96e094d31eaff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HarkFFTVal)))
  "Returns md5sum for a message object of type 'HarkFFTVal"
  "a7b9ef54d8fd113072a96e094d31eaff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HarkFFTVal>)))
  "Returns full string definition for message of type '<HarkFFTVal>"
  (cl:format cl:nil "float32[] fftdata_real~%float32[] fftdata_imag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HarkFFTVal)))
  "Returns full string definition for message of type 'HarkFFTVal"
  (cl:format cl:nil "float32[] fftdata_real~%float32[] fftdata_imag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HarkFFTVal>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'fftdata_real) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'fftdata_imag) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HarkFFTVal>))
  "Converts a ROS message object to a list"
  (cl:list 'HarkFFTVal
    (cl:cons ':fftdata_real (fftdata_real msg))
    (cl:cons ':fftdata_imag (fftdata_imag msg))
))
