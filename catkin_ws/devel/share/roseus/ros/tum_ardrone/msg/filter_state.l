;; Auto-generated. Do not edit!


(when (boundp 'tum_ardrone::filter_state)
  (if (not (find-package "TUM_ARDRONE"))
    (make-package "TUM_ARDRONE"))
  (shadow 'filter_state (find-package "TUM_ARDRONE")))
(unless (find-package "TUM_ARDRONE::FILTER_STATE")
  (make-package "TUM_ARDRONE::FILTER_STATE"))

(in-package "ROS")
;;//! \htmlinclude filter_state.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*PTAM_IDLE*" (find-package "TUM_ARDRONE::FILTER_STATE"))
(shadow '*PTAM_IDLE* (find-package "TUM_ARDRONE::FILTER_STATE"))
(defconstant tum_ardrone::filter_state::*PTAM_IDLE* 0)
(intern "*PTAM_INITIALIZING*" (find-package "TUM_ARDRONE::FILTER_STATE"))
(shadow '*PTAM_INITIALIZING* (find-package "TUM_ARDRONE::FILTER_STATE"))
(defconstant tum_ardrone::filter_state::*PTAM_INITIALIZING* 1)
(intern "*PTAM_LOST*" (find-package "TUM_ARDRONE::FILTER_STATE"))
(shadow '*PTAM_LOST* (find-package "TUM_ARDRONE::FILTER_STATE"))
(defconstant tum_ardrone::filter_state::*PTAM_LOST* 2)
(intern "*PTAM_GOOD*" (find-package "TUM_ARDRONE::FILTER_STATE"))
(shadow '*PTAM_GOOD* (find-package "TUM_ARDRONE::FILTER_STATE"))
(defconstant tum_ardrone::filter_state::*PTAM_GOOD* 3)
(intern "*PTAM_BEST*" (find-package "TUM_ARDRONE::FILTER_STATE"))
(shadow '*PTAM_BEST* (find-package "TUM_ARDRONE::FILTER_STATE"))
(defconstant tum_ardrone::filter_state::*PTAM_BEST* 4)
(intern "*PTAM_TOOKKF*" (find-package "TUM_ARDRONE::FILTER_STATE"))
(shadow '*PTAM_TOOKKF* (find-package "TUM_ARDRONE::FILTER_STATE"))
(defconstant tum_ardrone::filter_state::*PTAM_TOOKKF* 5)
(intern "*PTAM_FALSEPOSITIVE*" (find-package "TUM_ARDRONE::FILTER_STATE"))
(shadow '*PTAM_FALSEPOSITIVE* (find-package "TUM_ARDRONE::FILTER_STATE"))
(defconstant tum_ardrone::filter_state::*PTAM_FALSEPOSITIVE* 6)
(defclass tum_ardrone::filter_state
  :super ros::object
  :slots (_header _x _y _z _dx _dy _dz _roll _pitch _yaw _dyaw _scale _ptamState _scaleAccuracy _droneState _batteryPercent ))

(defmethod tum_ardrone::filter_state
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:dx __dx) 0.0)
    ((:dy __dy) 0.0)
    ((:dz __dz) 0.0)
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:dyaw __dyaw) 0.0)
    ((:scale __scale) 0.0)
    ((:ptamState __ptamState) 0)
    ((:scaleAccuracy __scaleAccuracy) 0.0)
    ((:droneState __droneState) 0)
    ((:batteryPercent __batteryPercent) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _dx (float __dx))
   (setq _dy (float __dy))
   (setq _dz (float __dz))
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _dyaw (float __dyaw))
   (setq _scale (float __scale))
   (setq _ptamState (round __ptamState))
   (setq _scaleAccuracy (float __scaleAccuracy))
   (setq _droneState (round __droneState))
   (setq _batteryPercent (float __batteryPercent))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:dx
   (&optional __dx)
   (if __dx (setq _dx __dx)) _dx)
  (:dy
   (&optional __dy)
   (if __dy (setq _dy __dy)) _dy)
  (:dz
   (&optional __dz)
   (if __dz (setq _dz __dz)) _dz)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:dyaw
   (&optional __dyaw)
   (if __dyaw (setq _dyaw __dyaw)) _dyaw)
  (:scale
   (&optional __scale)
   (if __scale (setq _scale __scale)) _scale)
  (:ptamState
   (&optional __ptamState)
   (if __ptamState (setq _ptamState __ptamState)) _ptamState)
  (:scaleAccuracy
   (&optional __scaleAccuracy)
   (if __scaleAccuracy (setq _scaleAccuracy __scaleAccuracy)) _scaleAccuracy)
  (:droneState
   (&optional __droneState)
   (if __droneState (setq _droneState __droneState)) _droneState)
  (:batteryPercent
   (&optional __batteryPercent)
   (if __batteryPercent (setq _batteryPercent __batteryPercent)) _batteryPercent)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ;; float32 _dx
    4
    ;; float32 _dy
    4
    ;; float32 _dz
    4
    ;; float32 _roll
    4
    ;; float32 _pitch
    4
    ;; float32 _yaw
    4
    ;; float32 _dyaw
    4
    ;; float32 _scale
    4
    ;; uint32 _ptamState
    4
    ;; float32 _scaleAccuracy
    4
    ;; uint32 _droneState
    4
    ;; float32 _batteryPercent
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dx
       (sys::poke _dx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dy
       (sys::poke _dy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dz
       (sys::poke _dz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dyaw
       (sys::poke _dyaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _scale
       (sys::poke _scale (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _ptamState
       (write-long _ptamState s)
     ;; float32 _scaleAccuracy
       (sys::poke _scaleAccuracy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _droneState
       (write-long _droneState s)
     ;; float32 _batteryPercent
       (sys::poke _batteryPercent (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dx
     (setq _dx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dy
     (setq _dy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dz
     (setq _dz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dyaw
     (setq _dyaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _scale
     (setq _scale (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _ptamState
     (setq _ptamState (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _scaleAccuracy
     (setq _scaleAccuracy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _droneState
     (setq _droneState (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _batteryPercent
     (setq _batteryPercent (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tum_ardrone::filter_state :md5sum-) "33f8050d082c4ebadff26def43dcfd15")
(setf (get tum_ardrone::filter_state :datatype-) "tum_ardrone/filter_state")
(setf (get tum_ardrone::filter_state :definition-)
      "# constants
uint32 PTAM_IDLE = 0           # PTAM not running.
uint32 PTAM_INITIALIZING = 1   # initialization (trails)
uint32 PTAM_LOST = 2           # ptam is running, but lost
uint32 PTAM_GOOD = 3           # tracking quality OK
uint32 PTAM_BEST = 4           # tracking quality best
uint32 PTAM_TOOKKF = 5         # just took a new KF (equivalent to PTAM_BEST)
uint32 PTAM_FALSEPOSITIVE = 6  # ptam thinks it is good, but its estimate is rejected.

# header
Header      header

# ----------------- raw 10d filter state ----------------------------
float32     x
float32     y
float32     z
float32     dx
float32     dy
float32     dz
float32     roll
float32     pitch
float32     yaw
float32     dyaw



# --------------------- other values ---------------------------
float32     scale         # ptam scale factor (PTAMpos * scale = WORLDpos).
uint32      ptamState
float32     scaleAccuracy # if scale is very inaccurate, this is about 0.5, and grows up to 1 (=very accurate).


# ----------------- propagated from drone messages: -----------------
# 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
# 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
# Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)
uint32       droneState
float32      batteryPercent    # 0 means no battery, 100 means full battery

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :tum_ardrone/filter_state "33f8050d082c4ebadff26def43dcfd15")


