
(cl:in-package :asdf)

(defsystem "abot_imu-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RawImu" :depends-on ("_package_RawImu"))
    (:file "_package_RawImu" :depends-on ("_package"))
  ))