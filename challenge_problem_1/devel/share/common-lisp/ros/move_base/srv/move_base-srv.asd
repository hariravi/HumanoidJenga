
(cl:in-package :asdf)

(defsystem "move_base-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MoveBase" :depends-on ("_package_MoveBase"))
    (:file "_package_MoveBase" :depends-on ("_package"))
    (:file "RotateBase" :depends-on ("_package_RotateBase"))
    (:file "_package_RotateBase" :depends-on ("_package"))
  ))