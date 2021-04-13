;; Auto-generated. Do not edit!


(when (boundp 'tum_ardrone::SetReference)
  (if (not (find-package "TUM_ARDRONE"))
    (make-package "TUM_ARDRONE"))
  (shadow 'SetReference (find-package "TUM_ARDRONE")))
(unless (find-package "TUM_ARDRONE::SETREFERENCE")
  (make-package "TUM_ARDRONE::SETREFERENCE"))
(unless (find-package "TUM_ARDRONE::SETREFERENCEREQUEST")
  (make-package "TUM_ARDRONE::SETREFERENCEREQUEST"))
(unless (find-package "TUM_ARDRONE::SETREFERENCERESPONSE")
  (make-package "TUM_ARDRONE::SETREFERENCERESPONSE"))

(in-package "ROS")





(defclass tum_ardrone::SetReferenceRequest
  :super ros::object
  :slots (_x _y _z _heading ))

(defmethod tum_ardrone::SetReferenceRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:heading __heading) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _heading (float __heading))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ;; float32 _heading
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading
     (setq _heading (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass tum_ardrone::SetReferenceResponse
  :super ros::object
  :slots (_status ))

(defmethod tum_ardrone::SetReferenceResponse
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

(defclass tum_ardrone::SetReference
  :super ros::object
  :slots ())

(setf (get tum_ardrone::SetReference :md5sum-) "ceb0badddcb0258fe3612b618c482acf")
(setf (get tum_ardrone::SetReference :datatype-) "tum_ardrone/SetReference")
(setf (get tum_ardrone::SetReference :request) tum_ardrone::SetReferenceRequest)
(setf (get tum_ardrone::SetReference :response) tum_ardrone::SetReferenceResponse)

(defmethod tum_ardrone::SetReferenceRequest
  (:response () (instance tum_ardrone::SetReferenceResponse :init)))

(setf (get tum_ardrone::SetReferenceRequest :md5sum-) "ceb0badddcb0258fe3612b618c482acf")
(setf (get tum_ardrone::SetReferenceRequest :datatype-) "tum_ardrone/SetReferenceRequest")
(setf (get tum_ardrone::SetReferenceRequest :definition-)
      "
float32 x
float32 y
float32 z
float32 heading
---

bool status


")

(setf (get tum_ardrone::SetReferenceResponse :md5sum-) "ceb0badddcb0258fe3612b618c482acf")
(setf (get tum_ardrone::SetReferenceResponse :datatype-) "tum_ardrone/SetReferenceResponse")
(setf (get tum_ardrone::SetReferenceResponse :definition-)
      "
float32 x
float32 y
float32 z
float32 heading
---

bool status


")



(provide :tum_ardrone/SetReference "ceb0badddcb0258fe3612b618c482acf")


