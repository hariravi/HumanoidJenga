
(cl:in-package :asdf)

(defsystem "move_gripper-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CloseGripper" :depends-on ("_package_CloseGripper"))
    (:file "_package_CloseGripper" :depends-on ("_package"))
  ))