; Auto-generated. Do not edit!


(cl:in-package rover_movement-srv)


;//! \htmlinclude motor_command_server-request.msg.html

(cl:defclass <motor_command_server-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:string
    :initform ""))
)

(cl:defclass motor_command_server-request (<motor_command_server-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motor_command_server-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motor_command_server-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rover_movement-srv:<motor_command_server-request> is deprecated: use rover_movement-srv:motor_command_server-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <motor_command_server-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_movement-srv:command-val is deprecated.  Use rover_movement-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motor_command_server-request>) ostream)
  "Serializes a message object of type '<motor_command_server-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motor_command_server-request>) istream)
  "Deserializes a message object of type '<motor_command_server-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motor_command_server-request>)))
  "Returns string type for a service object of type '<motor_command_server-request>"
  "rover_movement/motor_command_serverRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_command_server-request)))
  "Returns string type for a service object of type 'motor_command_server-request"
  "rover_movement/motor_command_serverRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motor_command_server-request>)))
  "Returns md5sum for a message object of type '<motor_command_server-request>"
  "22c7c465d64c7e74c6ae22029c7ca150")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motor_command_server-request)))
  "Returns md5sum for a message object of type 'motor_command_server-request"
  "22c7c465d64c7e74c6ae22029c7ca150")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motor_command_server-request>)))
  "Returns full string definition for message of type '<motor_command_server-request>"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motor_command_server-request)))
  "Returns full string definition for message of type 'motor_command_server-request"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motor_command_server-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motor_command_server-request>))
  "Converts a ROS message object to a list"
  (cl:list 'motor_command_server-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude motor_command_server-response.msg.html

(cl:defclass <motor_command_server-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass motor_command_server-response (<motor_command_server-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motor_command_server-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motor_command_server-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rover_movement-srv:<motor_command_server-response> is deprecated: use rover_movement-srv:motor_command_server-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <motor_command_server-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_movement-srv:response-val is deprecated.  Use rover_movement-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motor_command_server-response>) ostream)
  "Serializes a message object of type '<motor_command_server-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motor_command_server-response>) istream)
  "Deserializes a message object of type '<motor_command_server-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motor_command_server-response>)))
  "Returns string type for a service object of type '<motor_command_server-response>"
  "rover_movement/motor_command_serverResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_command_server-response)))
  "Returns string type for a service object of type 'motor_command_server-response"
  "rover_movement/motor_command_serverResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motor_command_server-response>)))
  "Returns md5sum for a message object of type '<motor_command_server-response>"
  "22c7c465d64c7e74c6ae22029c7ca150")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motor_command_server-response)))
  "Returns md5sum for a message object of type 'motor_command_server-response"
  "22c7c465d64c7e74c6ae22029c7ca150")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motor_command_server-response>)))
  "Returns full string definition for message of type '<motor_command_server-response>"
  (cl:format cl:nil "string response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motor_command_server-response)))
  "Returns full string definition for message of type 'motor_command_server-response"
  (cl:format cl:nil "string response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motor_command_server-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motor_command_server-response>))
  "Converts a ROS message object to a list"
  (cl:list 'motor_command_server-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'motor_command_server)))
  'motor_command_server-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'motor_command_server)))
  'motor_command_server-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_command_server)))
  "Returns string type for a service object of type '<motor_command_server>"
  "rover_movement/motor_command_server")