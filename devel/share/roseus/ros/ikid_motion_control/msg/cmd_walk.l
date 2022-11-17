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
  :slots (_sx _sy _var_theta ))

(defmethod ikid_motion_control::cmd_walk
  (:init
   (&key
    ((:sx __sx) 0.0)
    ((:sy __sy) 0.0)
    ((:var_theta __var_theta) 0.0)
    )
   (send-super :init)
   (setq _sx (float __sx))
   (setq _sy (float __sy))
   (setq _var_theta (float __var_theta))
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
  (:serialization-length
   ()
   (+
    ;; float64 _sx
    8
    ;; float64 _sy
    8
    ;; float64 _var_theta
    8
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
   ;;
   self)
  )

(setf (get ikid_motion_control::cmd_walk :md5sum-) "642b1806da11ae8975a1c844cb320e6d")
(setf (get ikid_motion_control::cmd_walk :datatype-) "ikid_motion_control/cmd_walk")
(setf (get ikid_motion_control::cmd_walk :definition-)
      "float64 sx
float64 sy
float64 var_theta
")



(provide :ikid_motion_control/cmd_walk "642b1806da11ae8975a1c844cb320e6d")


