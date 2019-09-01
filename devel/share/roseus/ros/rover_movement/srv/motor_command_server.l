;; Auto-generated. Do not edit!


(when (boundp 'rover_movement::motor_command_server)
  (if (not (find-package "ROVER_MOVEMENT"))
    (make-package "ROVER_MOVEMENT"))
  (shadow 'motor_command_server (find-package "ROVER_MOVEMENT")))
(unless (find-package "ROVER_MOVEMENT::MOTOR_COMMAND_SERVER")
  (make-package "ROVER_MOVEMENT::MOTOR_COMMAND_SERVER"))
(unless (find-package "ROVER_MOVEMENT::MOTOR_COMMAND_SERVERREQUEST")
  (make-package "ROVER_MOVEMENT::MOTOR_COMMAND_SERVERREQUEST"))
(unless (find-package "ROVER_MOVEMENT::MOTOR_COMMAND_SERVERRESPONSE")
  (make-package "ROVER_MOVEMENT::MOTOR_COMMAND_SERVERRESPONSE"))

(in-package "ROS")





(defclass rover_movement::motor_command_serverRequest
  :super ros::object
  :slots (_command ))

(defmethod rover_movement::motor_command_serverRequest
  (:init
   (&key
    ((:command __command) "")
    )
   (send-super :init)
   (setq _command (string __command))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rover_movement::motor_command_serverResponse
  :super ros::object
  :slots (_response ))

(defmethod rover_movement::motor_command_serverResponse
  (:init
   (&key
    ((:response __response) "")
    )
   (send-super :init)
   (setq _response (string __response))
   self)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:serialization-length
   ()
   (+
    ;; string _response
    4 (length _response)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _response
       (write-long (length _response) s) (princ _response s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _response
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rover_movement::motor_command_server
  :super ros::object
  :slots ())

(setf (get rover_movement::motor_command_server :md5sum-) "22c7c465d64c7e74c6ae22029c7ca150")
(setf (get rover_movement::motor_command_server :datatype-) "rover_movement/motor_command_server")
(setf (get rover_movement::motor_command_server :request) rover_movement::motor_command_serverRequest)
(setf (get rover_movement::motor_command_server :response) rover_movement::motor_command_serverResponse)

(defmethod rover_movement::motor_command_serverRequest
  (:response () (instance rover_movement::motor_command_serverResponse :init)))

(setf (get rover_movement::motor_command_serverRequest :md5sum-) "22c7c465d64c7e74c6ae22029c7ca150")
(setf (get rover_movement::motor_command_serverRequest :datatype-) "rover_movement/motor_command_serverRequest")
(setf (get rover_movement::motor_command_serverRequest :definition-)
      "string command
---
string response
")

(setf (get rover_movement::motor_command_serverResponse :md5sum-) "22c7c465d64c7e74c6ae22029c7ca150")
(setf (get rover_movement::motor_command_serverResponse :datatype-) "rover_movement/motor_command_serverResponse")
(setf (get rover_movement::motor_command_serverResponse :definition-)
      "string command
---
string response
")



(provide :rover_movement/motor_command_server "22c7c465d64c7e74c6ae22029c7ca150")


