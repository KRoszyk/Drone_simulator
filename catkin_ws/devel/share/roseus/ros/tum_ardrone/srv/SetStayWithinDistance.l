;; Auto-generated. Do not edit!


(when (boundp 'tum_ardrone::SetStayWithinDistance)
  (if (not (find-package "TUM_ARDRONE"))
    (make-package "TUM_ARDRONE"))
  (shadow 'SetStayWithinDistance (find-package "TUM_ARDRONE")))
(unless (find-package "TUM_ARDRONE::SETSTAYWITHINDISTANCE")
  (make-package "TUM_ARDRONE::SETSTAYWITHINDISTANCE"))
(unless (find-package "TUM_ARDRONE::SETSTAYWITHINDISTANCEREQUEST")
  (make-package "TUM_ARDRONE::SETSTAYWITHINDISTANCEREQUEST"))
(unless (find-package "TUM_ARDRONE::SETSTAYWITHINDISTANCERESPONSE")
  (make-package "TUM_ARDRONE::SETSTAYWITHINDISTANCERESPONSE"))

(in-package "ROS")





(defclass tum_ardrone::SetStayWithinDistanceRequest
  :super ros::object
  :slots (_distance ))

(defmethod tum_ardrone::SetStayWithinDistanceRequest
  (:init
   (&key
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _distance (float __distance))
   self)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; float32 _distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass tum_ardrone::SetStayWithinDistanceResponse
  :super ros::object
  :slots (_status ))

(defmethod tum_ardrone::SetStayWithinDistanceResponse
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

(defclass tum_ardrone::SetStayWithinDistance
  :super ros::object
  :slots ())

(setf (get tum_ardrone::SetStayWithinDistance :md5sum-) "e42c385d60567bcc18375e4218bebdff")
(setf (get tum_ardrone::SetStayWithinDistance :datatype-) "tum_ardrone/SetStayWithinDistance")
(setf (get tum_ardrone::SetStayWithinDistance :request) tum_ardrone::SetStayWithinDistanceRequest)
(setf (get tum_ardrone::SetStayWithinDistance :response) tum_ardrone::SetStayWithinDistanceResponse)

(defmethod tum_ardrone::SetStayWithinDistanceRequest
  (:response () (instance tum_ardrone::SetStayWithinDistanceResponse :init)))

(setf (get tum_ardrone::SetStayWithinDistanceRequest :md5sum-) "e42c385d60567bcc18375e4218bebdff")
(setf (get tum_ardrone::SetStayWithinDistanceRequest :datatype-) "tum_ardrone/SetStayWithinDistanceRequest")
(setf (get tum_ardrone::SetStayWithinDistanceRequest :definition-)
      "
float32 distance
---

bool status


")

(setf (get tum_ardrone::SetStayWithinDistanceResponse :md5sum-) "e42c385d60567bcc18375e4218bebdff")
(setf (get tum_ardrone::SetStayWithinDistanceResponse :datatype-) "tum_ardrone/SetStayWithinDistanceResponse")
(setf (get tum_ardrone::SetStayWithinDistanceResponse :definition-)
      "
float32 distance
---

bool status


")



(provide :tum_ardrone/SetStayWithinDistance "e42c385d60567bcc18375e4218bebdff")


