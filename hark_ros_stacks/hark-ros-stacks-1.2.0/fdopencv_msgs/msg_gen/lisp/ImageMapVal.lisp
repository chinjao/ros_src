; Auto-generated. Do not edit!


(cl:in-package fdopencv_msgs-msg)


;//! \htmlinclude ImageMapVal.msg.html

(cl:defclass <ImageMapVal> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (img
    :reader img
    :initarg :img
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass ImageMapVal (<ImageMapVal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageMapVal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageMapVal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fdopencv_msgs-msg:<ImageMapVal> is deprecated: use fdopencv_msgs-msg:ImageMapVal instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ImageMapVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fdopencv_msgs-msg:id-val is deprecated.  Use fdopencv_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'img-val :lambda-list '(m))
(cl:defmethod img-val ((m <ImageMapVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fdopencv_msgs-msg:img-val is deprecated.  Use fdopencv_msgs-msg:img instead.")
  (img m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageMapVal>) ostream)
  "Serializes a message object of type '<ImageMapVal>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'img) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageMapVal>) istream)
  "Deserializes a message object of type '<ImageMapVal>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'img) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageMapVal>)))
  "Returns string type for a message object of type '<ImageMapVal>"
  "fdopencv_msgs/ImageMapVal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageMapVal)))
  "Returns string type for a message object of type 'ImageMapVal"
  "fdopencv_msgs/ImageMapVal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageMapVal>)))
  "Returns md5sum for a message object of type '<ImageMapVal>"
  "ec273785f6797c77d37579f49be2dd3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageMapVal)))
  "Returns md5sum for a message object of type 'ImageMapVal"
  "ec273785f6797c77d37579f49be2dd3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageMapVal>)))
  "Returns full string definition for message of type '<ImageMapVal>"
  (cl:format cl:nil "int32 id~%sensor_msgs/Image img~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageMapVal)))
  "Returns full string definition for message of type 'ImageMapVal"
  (cl:format cl:nil "int32 id~%sensor_msgs/Image img~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageMapVal>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'img))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageMapVal>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageMapVal
    (cl:cons ':id (id msg))
    (cl:cons ':img (img msg))
))
