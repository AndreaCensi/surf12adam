; Auto-generated. Do not edit!


(cl:in-package camera_actuator-msg)


;//! \htmlinclude CamCmd.msg.html

(cl:defclass <CamCmd> (roslisp-msg-protocol:ros-message)
  ((Pvalue
    :reader Pvalue
    :initarg :Pvalue
    :type cl:integer
    :initform 0)
   (Tvalue
    :reader Tvalue
    :initarg :Tvalue
    :type cl:integer
    :initform 0)
   (Zvalue
    :reader Zvalue
    :initarg :Zvalue
    :type cl:integer
    :initform 0))
)

(cl:defclass CamCmd (<CamCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CamCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CamCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_actuator-msg:<CamCmd> is deprecated: use camera_actuator-msg:CamCmd instead.")))

(cl:ensure-generic-function 'Pvalue-val :lambda-list '(m))
(cl:defmethod Pvalue-val ((m <CamCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_actuator-msg:Pvalue-val is deprecated.  Use camera_actuator-msg:Pvalue instead.")
  (Pvalue m))

(cl:ensure-generic-function 'Tvalue-val :lambda-list '(m))
(cl:defmethod Tvalue-val ((m <CamCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_actuator-msg:Tvalue-val is deprecated.  Use camera_actuator-msg:Tvalue instead.")
  (Tvalue m))

(cl:ensure-generic-function 'Zvalue-val :lambda-list '(m))
(cl:defmethod Zvalue-val ((m <CamCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_actuator-msg:Zvalue-val is deprecated.  Use camera_actuator-msg:Zvalue instead.")
  (Zvalue m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CamCmd>) ostream)
  "Serializes a message object of type '<CamCmd>"
  (cl:let* ((signed (cl:slot-value msg 'Pvalue)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Tvalue)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Zvalue)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CamCmd>) istream)
  "Deserializes a message object of type '<CamCmd>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Pvalue) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Tvalue) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Zvalue) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CamCmd>)))
  "Returns string type for a message object of type '<CamCmd>"
  "camera_actuator/CamCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CamCmd)))
  "Returns string type for a message object of type 'CamCmd"
  "camera_actuator/CamCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CamCmd>)))
  "Returns md5sum for a message object of type '<CamCmd>"
  "46895988c20b47142a93febd21dd1e8a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CamCmd)))
  "Returns md5sum for a message object of type 'CamCmd"
  "46895988c20b47142a93febd21dd1e8a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CamCmd>)))
  "Returns full string definition for message of type '<CamCmd>"
  (cl:format cl:nil "int64 Pvalue~%int64 Tvalue~%int64 Zvalue~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CamCmd)))
  "Returns full string definition for message of type 'CamCmd"
  (cl:format cl:nil "int64 Pvalue~%int64 Tvalue~%int64 Zvalue~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CamCmd>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CamCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'CamCmd
    (cl:cons ':Pvalue (Pvalue msg))
    (cl:cons ':Tvalue (Tvalue msg))
    (cl:cons ':Zvalue (Zvalue msg))
))
