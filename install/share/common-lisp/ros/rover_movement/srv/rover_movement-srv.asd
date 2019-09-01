
(cl:in-package :asdf)

(defsystem "rover_movement-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "motor_command_server" :depends-on ("_package_motor_command_server"))
    (:file "_package_motor_command_server" :depends-on ("_package"))
  ))