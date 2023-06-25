; Auto-generated. Do not edit!


(cl:in-package ikid_motion_control-msg)


;//! \htmlinclude cmd_walk.msg.html

(cl:defclass <cmd_walk> (roslisp-msg-protocol:ros-message)
  ((sx
    :reader sx
    :initarg :sx
    :type cl:float
    :initform 0.0)
   (sy
    :reader sy
    :initarg :sy
    :type cl:float
    :initform 0.0)
   (var_theta
    :reader var_theta
    :initarg :var_theta
    :type cl:float
    :initform 0.0)
   (walk_with_ball
    :reader walk_with_ball
    :initarg :walk_with_ball
    :type cl:boolean
    :initform cl:nil)
   (stop_walk
    :reader stop_walk
    :initarg :stop_walk
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass cmd_walk (<cmd_walk>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cmd_walk>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cmd_walk)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ikid_motion_control-msg:<cmd_walk> is deprecated: use ikid_motion_control-msg:cmd_walk instead.")))

(cl:ensure-generic-function 'sx-val :lambda-list '(m))
(cl:defmethod sx-val ((m <cmd_walk>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ikid_motion_control-msg:sx-val is deprecated.  Use ikid_motion_control-msg:sx instead.")
  (sx m))

(cl:ensure-generic-function 'sy-val :lambda-list '(m))
(cl:defmethod sy-val ((m <cmd_walk>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ikid_motion_control-msg:sy-val is deprecated.  Use ikid_motion_control-msg:sy instead.")
  (sy m))

(cl:ensure-generic-function 'var_theta-val :lambda-list '(m))
(cl:defmethod var_theta-val ((m <cmd_walk>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ikid_motion_control-msg:var_theta-val is deprecated.  Use ikid_motion_control-msg:var_theta instead.")
  (var_theta m))

(cl:ensure-generic-function 'walk_with_ball-val :lambda-list '(m))
(cl:defmethod walk_with_ball-val ((m <cmd_walk>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ikid_motion_control-msg:walk_with_ball-val is deprecated.  Use ikid_motion_control-msg:walk_with_ball instead.")
  (walk_with_ball m))

(cl:ensure-generic-function 'stop_walk-val :lambda-list '(m))
(cl:defmethod stop_walk-val ((m <cmd_walk>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ikid_motion_control-msg:stop_walk-val is deprecated.  Use ikid_motion_control-msg:stop_walk instead.")
  (stop_walk m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cmd_walk>) ostream)
  "Serializes a message object of type '<cmd_walk>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'sx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'sy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'var_theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'walk_with_ball) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stop_walk) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cmd_walk>) istream)
  "Deserializes a message object of type '<cmd_walk>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sx) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sy) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'var_theta) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'walk_with_ball) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'stop_walk) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cmd_walk>)))
  "Returns string type for a message object of type '<cmd_walk>"
  "ikid_motion_control/cmd_walk")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmd_walk)))
  "Returns string type for a message object of type 'cmd_walk"
  "ikid_motion_control/cmd_walk")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cmd_walk>)))
  "Returns md5sum for a message object of type '<cmd_walk>"
  "768710e1aa78c3a3025d8ab3e396f5e8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cmd_walk)))
  "Returns md5sum for a message object of type 'cmd_walk"
  "768710e1aa78c3a3025d8ab3e396f5e8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cmd_walk>)))
  "Returns full string definition for message of type '<cmd_walk>"
  (cl:format cl:nil "float64 sx  # 步长~%float64 sy   # 步宽~%float64 var_theta  # 转角增量~%bool walk_with_ball  # 是否带球行走/动态踢球~%bool stop_walk   # 停止行走标志位~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cmd_walk)))
  "Returns full string definition for message of type 'cmd_walk"
  (cl:format cl:nil "float64 sx  # 步长~%float64 sy   # 步宽~%float64 var_theta  # 转角增量~%bool walk_with_ball  # 是否带球行走/动态踢球~%bool stop_walk   # 停止行走标志位~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cmd_walk>))
  (cl:+ 0
     8
     8
     8
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cmd_walk>))
  "Converts a ROS message object to a list"
  (cl:list 'cmd_walk
    (cl:cons ':sx (sx msg))
    (cl:cons ':sy (sy msg))
    (cl:cons ':var_theta (var_theta msg))
    (cl:cons ':walk_with_ball (walk_with_ball msg))
    (cl:cons ':stop_walk (stop_walk msg))
))
