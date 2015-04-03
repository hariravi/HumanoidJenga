; Auto-generated. Do not edit!


(cl:in-package move_base-srv)


;//! \htmlinclude RotateBase-request.msg.html

(cl:defclass <RotateBase-request> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass RotateBase-request (<RotateBase-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RotateBase-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RotateBase-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_base-srv:<RotateBase-request> is deprecated: use move_base-srv:RotateBase-request instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <RotateBase-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_base-srv:angle-val is deprecated.  Use move_base-srv:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RotateBase-request>) ostream)
  "Serializes a message object of type '<RotateBase-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RotateBase-request>) istream)
  "Deserializes a message object of type '<RotateBase-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RotateBase-request>)))
  "Returns string type for a service object of type '<RotateBase-request>"
  "move_base/RotateBaseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RotateBase-request)))
  "Returns string type for a service object of type 'RotateBase-request"
  "move_base/RotateBaseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RotateBase-request>)))
  "Returns md5sum for a message object of type '<RotateBase-request>"
  "194939c001a63802dc4131cae92cda22")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RotateBase-request)))
  "Returns md5sum for a message object of type 'RotateBase-request"
  "194939c001a63802dc4131cae92cda22")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RotateBase-request>)))
  "Returns full string definition for message of type '<RotateBase-request>"
  (cl:format cl:nil "~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RotateBase-request)))
  "Returns full string definition for message of type 'RotateBase-request"
  (cl:format cl:nil "~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RotateBase-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RotateBase-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RotateBase-request
    (cl:cons ':angle (angle msg))
))
;//! \htmlinclude RotateBase-response.msg.html

(cl:defclass <RotateBase-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RotateBase-response (<RotateBase-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RotateBase-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RotateBase-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_base-srv:<RotateBase-response> is deprecated: use move_base-srv:RotateBase-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RotateBase-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_base-srv:success-val is deprecated.  Use move_base-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RotateBase-response>) ostream)
  "Serializes a message object of type '<RotateBase-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RotateBase-response>) istream)
  "Deserializes a message object of type '<RotateBase-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RotateBase-response>)))
  "Returns string type for a service object of type '<RotateBase-response>"
  "move_base/RotateBaseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RotateBase-response)))
  "Returns string type for a service object of type 'RotateBase-response"
  "move_base/RotateBaseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RotateBase-response>)))
  "Returns md5sum for a message object of type '<RotateBase-response>"
  "194939c001a63802dc4131cae92cda22")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RotateBase-response)))
  "Returns md5sum for a message object of type 'RotateBase-response"
  "194939c001a63802dc4131cae92cda22")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RotateBase-response>)))
  "Returns full string definition for message of type '<RotateBase-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RotateBase-response)))
  "Returns full string definition for message of type 'RotateBase-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RotateBase-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RotateBase-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RotateBase-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RotateBase)))
  'RotateBase-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RotateBase)))
  'RotateBase-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RotateBase)))
  "Returns string type for a service object of type '<RotateBase>"
  "move_base/RotateBase")