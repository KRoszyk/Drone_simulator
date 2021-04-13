;; Auto-generated. Do not edit!


(when (boundp 'tum_ardrone::SetMaxControl)
  (if (not (find-package "TUM_ARDRONE"))
    (make-package "TUM_ARDRONE"))
  (shadow 'SetMaxControl (find-package "TUM_ARDRONE")))
(unless (find-package "TUM_ARDRONE::SETMAXCONTROL")
  (make-package "TUM_ARDRONE::SETMAXCONTROL"))
(unless (find-package "TUM_ARDRONE::SETMAXCONTROLREQUEST")
  (make-package "TUM_ARDRONE::SETMAXCONTROLREQUEST"))
(unless (find-package "TUM_ARDRONE::SETMAXCONTROLRESPONSE")
  (make-package "TUM_ARDRONE::SETMAXCONTROLRESPONSE"))

(in-package "ROS")





(defclass tum_ardrone::SetMaxControlRequest
  :super ros::object
  :slots (_speed ))

(defmethod tum_ardrone::SetMaxControlRequest
  (:init
   (&key
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _speed (float __speed))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass tum_ardrone::SetMaxControlResponse
  :super ros::object
  :slots (_status ))

(defmethod tum_ardrone::SetMaxControlResponse
  (:init
   (&key
    ((:status __status) nil)
    )
   (send-super :init)
   (setq _status __status)
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; bool _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass tum_ardrone::SetMaxControl
  :super ros::object
  :slots ())

(setf (get tum_ardrone::SetMaxControl :md5sum-) "7ff7215b2a764d7df902316cc8f39be9")
(setf (get tum_ardrone::SetMaxControl :datatype-) "tum_ardrone/SetMaxControl")
(setf (get tum_ardrone::SetMaxControl :request) tum_ardrone::SetMaxControlRequest)
(setf (get tum_ardrone::SetMaxControl :response) tum_ardrone::SetMaxControlResponse)

(defmethod tum_ardrone::SetMaxControlRequest
  (:response () (instance tum_ardrone::SetMaxControlResponse :init)))

(setf (get tum_ardrone::SetMaxControlRequest :md5sum-) "7ff7215b2a764d7df902316cc8f39be9")
(setf (get tum_ardrone::SetMaxControlRequest :datatype-) "tum_ardrone/SetMaxControlRequest")
(setf (get tum_ardrone::SetMaxControlRequest :definition-)
      "
float32 speed
---

bool status


")

(setf (get tum_ardrone::SetMaxControlResponse :md5sum-) "7ff7215b2a764d7df902316cc8f39be9")
(setf (get tum_ardrone::SetMaxControlResponse :datatype-) "tum_ardrone/SetMaxControlResponse")
(setf (get tum_ardrone::SetMaxControlResponse :definition-)
      "
float32 speed
---

bool status


")



(provide :tum_ardrone/SetMaxControl "7ff7215b2a764d7df902316cc8f39be9")


