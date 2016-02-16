
(cl:in-package :asdf)

(defsystem "fdopencv_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ImageMapVal" :depends-on ("_package_ImageMapVal"))
    (:file "_package_ImageMapVal" :depends-on ("_package"))
    (:file "ImageMap" :depends-on ("_package_ImageMap"))
    (:file "_package_ImageMap" :depends-on ("_package"))
  ))