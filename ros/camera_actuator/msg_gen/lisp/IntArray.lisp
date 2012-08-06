; Auto-generated. Do not edit!


(cl:in-package camera_actuator-msg)


;//! \htmlinclude IntArray.msg.html

(cl:defclass <IntArray> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass IntArray (<IntArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IntArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IntArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_actuator-msg:<IntArray> is deprecated: use camera_actuator-msg:IntArray instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <IntArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_actuator-msg:data-val is deprecated.  Use camera_actuator-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IntArray>) ostream)
  "Serializes a message object of type '<IntArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IntArray>) istream)
  "Deserializes a message object of type '<IntArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IntArray>)))
  "Returns string type for a message object of type '<IntArray>"
  "camera_actuator/IntArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IntArray)))
  "Returns string type for a message object of type 'IntArray"
  "camera_actuator/IntArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IntArray>)))
  "Returns md5sum for a message object of type '<IntArray>"
  "8560fbebb34fa1b9472337b5c3d38fda")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IntArray)))
  "Returns md5sum for a message object of type 'IntArray"
  "8560fbebb34fa1b9472337b5c3d38fda")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IntArray>)))
  "Returns full string definition for message of type '<IntArray>"
  (cl:format cl:nil "int16[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IntArray)))
  "Returns full string definition for message of type 'IntArray"
  (cl:format cl:nil "int16[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IntArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IntArray>))
  "Converts a ROS message object to a list"
  (cl:list 'IntArray
    (cl:cons ':data (data msg))
))
