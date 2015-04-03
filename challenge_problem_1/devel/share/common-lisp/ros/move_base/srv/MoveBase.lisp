; Auto-generated. Do not edit!


(cl:in-package move_base-srv)


;//! \htmlinclude MoveBase-request.msg.html

(cl:defclass <MoveBase-request> (roslisp-msg-protocol:ros-message)
  ((distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass MoveBase-request (<MoveBase-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveBase-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveBase-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_base-srv:<MoveBase-request> is deprecated: use move_base-srv:MoveBase-request instead.")))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <MoveBase-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_base-srv:distance-val is deprecated.  Use move_base-srv:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveBase-request>) ostream)
  "Serializes a message object of type '<MoveBase-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveBase-request>) istream)
  "Deserializes a message object of type '<MoveBase-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveBase-request>)))
  "Returns string type for a service object of type '<MoveBase-request>"
  "move_base/MoveBaseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveBase-request)))
  "Returns string type for a service object of type 'MoveBase-request"
  "move_base/MoveBaseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveBase-request>)))
  "Returns md5sum for a message object of type '<MoveBase-request>"
  "4d1b79b5a5ef1f221b7952b382ecb8cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveBase-request)))
  "Returns md5sum for a message object of type 'MoveBase-request"
  "4d1b79b5a5ef1f221b7952b382ecb8cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveBase-request>)))
  "Returns full string definition for message of type '<MoveBase-request>"
  (cl:format cl:nil "~%float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveBase-request)))
  "Returns full string definition for message of type 'MoveBase-request"
  (cl:format cl:nil "~%float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveBase-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveBase-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveBase-request
    (cl:cons ':distance (distance msg))
))
;//! \htmlinclude MoveBase-response.msg.html

(cl:defclass <MoveBase-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoveBase-response (<MoveBase-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveBase-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveBase-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_base-srv:<MoveBase-response> is deprecated: use move_base-srv:MoveBase-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <MoveBase-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_base-srv:success-val is deprecated.  Use move_base-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveBase-response>) ostream)
  "Serializes a message object of type '<MoveBase-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveBase-response>) istream)
  "Deserializes a message object of type '<MoveBase-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveBase-response>)))
  "Returns string type for a service object of type '<MoveBase-response>"
  "move_base/MoveBaseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveBase-response)))
  "Returns string type for a service object of type 'MoveBase-response"
  "move_base/MoveBaseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveBase-response>)))
  "Returns md5sum for a message object of type '<MoveBase-response>"
  "4d1b79b5a5ef1f221b7952b382ecb8cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveBase-response)))
  "Returns md5sum for a message object of type 'MoveBase-response"
  "4d1b79b5a5ef1f221b7952b382ecb8cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveBase-response>)))
  "Returns full string definition for message of type '<MoveBase-response>"
  (cl:format cl:nil "bool success~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveBase-response)))
  "Returns full string definition for message of type 'MoveBase-response"
  (cl:format cl:nil "bool success~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveBase-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveBase-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveBase-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveBase)))
  'MoveBase-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveBase)))
  'MoveBase-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveBase)))
  "Returns string type for a service object of type '<MoveBase>"
  "move_base/MoveBase")