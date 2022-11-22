;; Auto-generated. Do not edit!


(when (boundp 'ikid_motion_control::cmd_walk)
  (if (not (find-package "IKID_MOTION_CONTROL"))
    (make-package "IKID_MOTION_CONTROL"))
  (shadow 'cmd_walk (find-package "IKID_MOTION_CONTROL")))
(unless (find-package "IKID_MOTION_CONTROL::CMD_WALK")
  (make-package "IKID_MOTION_CONTROL::CMD_WALK"))

(in-package "ROS")
;;//! \htmlinclude cmd_walk.msg.html


(defclass ikid_motion_control::cmd_walk
  :super ros::object
  :slots (_sx _sy _var_theta _walk_with_ball ))

(defmethod ikid_motion_control::cmd_walk
  (:init
   (&key
    ((:sx __sx) 0.0)
    ((:sy __sy) 0.0)
    ((:var_theta __var_theta) 0.0)
    ((:walk_with_ball __walk_with_ball) nil)
    )
   (send-super :init)
   (setq _sx (float __sx))
   (setq _sy (float __sy))
   (setq _var_theta (float __var_theta))
   (setq _walk_with_ball __walk_with_ball)
   self)
  (:sx
   (&optional __sx)
   (if __sx (setq _sx __sx)) _sx)
  (:sy
   (&optional __sy)
   (if __sy (setq _sy __sy)) _sy)
  (:var_theta
   (&optional __var_theta)
   (if __var_theta (setq _var_theta __var_theta)) _var_theta)
  (:walk_with_ball
   (&optional (__walk_with_ball :null))
   (if (not (eq __walk_with_ball :null)) (setq _walk_with_ball __walk_with_ball)) _walk_with_ball)
  (:serialization-length
   ()
   (+
    ;; float64 _sx
    8
    ;; float64 _sy
    8
    ;; float64 _var_theta
    8
    ;; bool _walk_with_ball
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _sx
       (sys::poke _sx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _sy
       (sys::poke _sy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _var_theta
       (sys::poke _var_theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _walk_with_ball
       (if _walk_with_ball (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _sx
     (setq _sx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _sy
     (setq _sy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _var_theta
     (setq _var_theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _walk_with_ball
     (setq _walk_with_ball (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ikid_motion_control::cmd_walk :md5sum-) "9ad235b8f570d179119733addd31d488")
(setf (get ikid_motion_control::cmd_walk :datatype-) "ikid_motion_control/cmd_walk")
(setf (get ikid_motion_control::cmd_walk :definition-)
      "float64 sx
float64 sy
float64 var_theta
bool walk_with_ball
")



(provide :ikid_motion_control/cmd_walk "9ad235b8f570d179119733addd31d488")


