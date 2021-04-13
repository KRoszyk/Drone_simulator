;; Auto-generated. Do not edit!


(when (boundp 'tum_ardrone::SetStayTime)
  (if (not (find-package "TUM_ARDRONE"))
    (make-package "TUM_ARDRONE"))
  (shadow 'SetStayTime (find-package "TUM_ARDRONE")))
(unless (find-package "TUM_ARDRONE::SETSTAYTIME")
  (make-package "TUM_ARDRONE::SETSTAYTIME"))
(unless (find-package "TUM_ARDRONE::SETSTAYTIMEREQUEST")
  (make-package "TUM_ARDRONE::SETSTAYTIMEREQUEST"))
(unless (find-package "TUM_ARDRONE::SETSTAYTIMERESPONSE")
  (make-package "TUM_ARDRONE::SETSTAYTIMERESPONSE"))

(in-package "ROS")





(defclass tum_ardrone::SetStayTimeRequest
  :super ros::object
  :slots (_duration ))

(defmethod tum_ardrone::SetStayTimeRequest
  (:init
   (&key
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _duration (float __duration))
   self)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; float32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass tum_ardrone::SetStayTimeResponse
  :super ros::object
  :slots (_status ))

(defmethod tum_ardrone::SetStayTimeResponse
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

(defclass tum_ardrone::SetStayTime
  :super ros::object
  :slots ())

(setf (get tum_ardrone::SetStayTime :md5sum-) "d3a1853188ab17a3b9ca3ccc971be236")
(setf (get tum_ardrone::SetStayTime :datatype-) "tum_ardrone/SetStayTime")
(setf (get tum_ardrone::SetStayTime :request) tum_ardrone::SetStayTimeRequest)
(setf (get tum_ardrone::SetStayTime :response) tum_ardrone::SetStayTimeResponse)

(defmethod tum_ardrone::SetStayTimeRequest
  (:response () (instance tum_ardrone::SetStayTimeResponse :init)))

(setf (get tum_ardrone::SetStayTimeRequest :md5sum-) "d3a1853188ab17a3b9ca3ccc971be236")
(setf (get tum_ardrone::SetStayTimeRequest :datatype-) "tum_ardrone/SetStayTimeRequest")
(setf (get tum_ardrone::SetStayTimeRequest :definition-)
      "
float32 duration
---

bool status


")

(setf (get tum_ardrone::SetStayTimeResponse :md5sum-) "d3a1853188ab17a3b9ca3ccc971be236")
(setf (get tum_ardrone::SetStayTimeResponse :datatype-) "tum_ardrone/SetStayTimeResponse")
(setf (get tum_ardrone::SetStayTimeResponse :definition-)
      "
float32 duration
---

bool status


")



(provide :tum_ardrone/SetStayTime "d3a1853188ab17a3b9ca3ccc971be236")


