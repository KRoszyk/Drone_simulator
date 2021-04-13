;; Auto-generated. Do not edit!


(when (boundp 'tum_ardrone::SetInitialReachDistance)
  (if (not (find-package "TUM_ARDRONE"))
    (make-package "TUM_ARDRONE"))
  (shadow 'SetInitialReachDistance (find-package "TUM_ARDRONE")))
(unless (find-package "TUM_ARDRONE::SETINITIALREACHDISTANCE")
  (make-package "TUM_ARDRONE::SETINITIALREACHDISTANCE"))
(unless (find-package "TUM_ARDRONE::SETINITIALREACHDISTANCEREQUEST")
  (make-package "TUM_ARDRONE::SETINITIALREACHDISTANCEREQUEST"))
(unless (find-package "TUM_ARDRONE::SETINITIALREACHDISTANCERESPONSE")
  (make-package "TUM_ARDRONE::SETINITIALREACHDISTANCERESPONSE"))

(in-package "ROS")





(defclass tum_ardrone::SetInitialReachDistanceRequest
  :super ros::object
  :slots (_distance ))

(defmethod tum_ardrone::SetInitialReachDistanceRequest
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

(defclass tum_ardrone::SetInitialReachDistanceResponse
  :super ros::object
  :slots (_status ))

(defmethod tum_ardrone::SetInitialReachDistanceResponse
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

(defclass tum_ardrone::SetInitialReachDistance
  :super ros::object
  :slots ())

(setf (get tum_ardrone::SetInitialReachDistance :md5sum-) "e42c385d60567bcc18375e4218bebdff")
(setf (get tum_ardrone::SetInitialReachDistance :datatype-) "tum_ardrone/SetInitialReachDistance")
(setf (get tum_ardrone::SetInitialReachDistance :request) tum_ardrone::SetInitialReachDistanceRequest)
(setf (get tum_ardrone::SetInitialReachDistance :response) tum_ardrone::SetInitialReachDistanceResponse)

(defmethod tum_ardrone::SetInitialReachDistanceRequest
  (:response () (instance tum_ardrone::SetInitialReachDistanceResponse :init)))

(setf (get tum_ardrone::SetInitialReachDistanceRequest :md5sum-) "e42c385d60567bcc18375e4218bebdff")
(setf (get tum_ardrone::SetInitialReachDistanceRequest :datatype-) "tum_ardrone/SetInitialReachDistanceRequest")
(setf (get tum_ardrone::SetInitialReachDistanceRequest :definition-)
      "
float32 distance
---

bool status


")

(setf (get tum_ardrone::SetInitialReachDistanceResponse :md5sum-) "e42c385d60567bcc18375e4218bebdff")
(setf (get tum_ardrone::SetInitialReachDistanceResponse :datatype-) "tum_ardrone/SetInitialReachDistanceResponse")
(setf (get tum_ardrone::SetInitialReachDistanceResponse :definition-)
      "
float32 distance
---

bool status


")



(provide :tum_ardrone/SetInitialReachDistance "e42c385d60567bcc18375e4218bebdff")


