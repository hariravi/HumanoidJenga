; Auto-generated. Do not edit!


(cl:in-package move_gripper-srv)


;//! \htmlinclude CloseGripper-request.msg.html

(cl:defclass <CloseGripper-request> (roslisp-msg-protocol:ros-message)
  ((close
    :reader close
    :initarg :close
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CloseGripper-request (<CloseGripper-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CloseGripper-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CloseGripper-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_gripper-srv:<CloseGripper-request> is deprecated: use move_gripper-srv:CloseGripper-request instead.")))

(cl:ensure-generic-function 'close-val :lambda-list '(m))
(cl:defmethod close-val ((m <CloseGripper-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_gripper-srv:close-val is deprecated.  Use move_gripper-srv:close instead.")
  (close m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CloseGripper-request>) ostream)
  "Serializes a message object of type '<CloseGripper-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'close) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CloseGripper-request>) istream)
  "Deserializes a message object of type '<CloseGripper-request>"
    (cl:setf (cl:slot-value msg 'close) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CloseGripper-request>)))
  "Returns string type for a service object of type '<CloseGripper-request>"
  "move_gripper/CloseGripperRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CloseGripper-request)))
  "Returns string type for a service object of type 'CloseGripper-request"
  "move_gripper/CloseGripperRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CloseGripper-request>)))
  "Returns md5sum for a message object of type '<CloseGripper-request>"
  "d4fee5f1f332a01373ae327878f67206")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CloseGripper-request)))
  "Returns md5sum for a message object of type 'CloseGripper-request"
  "d4fee5f1f332a01373ae327878f67206")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CloseGripper-request>)))
  "Returns full string definition for message of type '<CloseGripper-request>"
  (cl:format cl:nil "~%bool close~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CloseGripper-request)))
  "Returns full string definition for message of type 'CloseGripper-request"
  (cl:format cl:nil "~%bool close~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CloseGripper-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CloseGripper-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CloseGripper-request
    (cl:cons ':close (close msg))
))
;//! \htmlinclude CloseGripper-response.msg.html

(cl:defclass <CloseGripper-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CloseGripper-response (<CloseGripper-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CloseGripper-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CloseGripper-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_gripper-srv:<CloseGripper-response> is deprecated: use move_gripper-srv:CloseGripper-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <CloseGripper-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_gripper-srv:success-val is deprecated.  Use move_gripper-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CloseGripper-response>) ostream)
  "Serializes a message object of type '<CloseGripper-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CloseGripper-response>) istream)
  "Deserializes a message object of type '<CloseGripper-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CloseGripper-response>)))
  "Returns string type for a service object of type '<CloseGripper-response>"
  "move_gripper/CloseGripperResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CloseGripper-response)))
  "Returns string type for a service object of type 'CloseGripper-response"
  "move_gripper/CloseGripperResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CloseGripper-response>)))
  "Returns md5sum for a message object of type '<CloseGripper-response>"
  "d4fee5f1f332a01373ae327878f67206")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CloseGripper-response)))
  "Returns md5sum for a message object of type 'CloseGripper-response"
  "d4fee5f1f332a01373ae327878f67206")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CloseGripper-response>)))
  "Returns full string definition for message of type '<CloseGripper-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CloseGripper-response)))
  "Returns full string definition for message of type 'CloseGripper-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CloseGripper-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CloseGripper-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CloseGripper-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CloseGripper)))
  'CloseGripper-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CloseGripper)))
  'CloseGripper-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CloseGripper)))
  "Returns string type for a service object of type '<CloseGripper>"
  "move_gripper/CloseGripper")