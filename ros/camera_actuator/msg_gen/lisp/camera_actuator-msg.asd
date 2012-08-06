
(cl:in-package :asdf)

(defsystem "camera_actuator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "IntArray" :depends-on ("_package_IntArray"))
    (:file "_package_IntArray" :depends-on ("_package"))
    (:file "CamCmd" :depends-on ("_package_CamCmd"))
    (:file "_package_CamCmd" :depends-on ("_package"))
  ))