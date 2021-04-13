// Auto-generated. Do not edit!

// (in-package tum_ardrone.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class filter_state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.dx = null;
      this.dy = null;
      this.dz = null;
      this.roll = null;
      this.pitch = null;
      this.yaw = null;
      this.dyaw = null;
      this.scale = null;
      this.ptamState = null;
      this.scaleAccuracy = null;
      this.droneState = null;
      this.batteryPercent = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('dx')) {
        this.dx = initObj.dx
      }
      else {
        this.dx = 0.0;
      }
      if (initObj.hasOwnProperty('dy')) {
        this.dy = initObj.dy
      }
      else {
        this.dy = 0.0;
      }
      if (initObj.hasOwnProperty('dz')) {
        this.dz = initObj.dz
      }
      else {
        this.dz = 0.0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('dyaw')) {
        this.dyaw = initObj.dyaw
      }
      else {
        this.dyaw = 0.0;
      }
      if (initObj.hasOwnProperty('scale')) {
        this.scale = initObj.scale
      }
      else {
        this.scale = 0.0;
      }
      if (initObj.hasOwnProperty('ptamState')) {
        this.ptamState = initObj.ptamState
      }
      else {
        this.ptamState = 0;
      }
      if (initObj.hasOwnProperty('scaleAccuracy')) {
        this.scaleAccuracy = initObj.scaleAccuracy
      }
      else {
        this.scaleAccuracy = 0.0;
      }
      if (initObj.hasOwnProperty('droneState')) {
        this.droneState = initObj.droneState
      }
      else {
        this.droneState = 0;
      }
      if (initObj.hasOwnProperty('batteryPercent')) {
        this.batteryPercent = initObj.batteryPercent
      }
      else {
        this.batteryPercent = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type filter_state
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float32(obj.z, buffer, bufferOffset);
    // Serialize message field [dx]
    bufferOffset = _serializer.float32(obj.dx, buffer, bufferOffset);
    // Serialize message field [dy]
    bufferOffset = _serializer.float32(obj.dy, buffer, bufferOffset);
    // Serialize message field [dz]
    bufferOffset = _serializer.float32(obj.dz, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float32(obj.roll, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float32(obj.pitch, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [dyaw]
    bufferOffset = _serializer.float32(obj.dyaw, buffer, bufferOffset);
    // Serialize message field [scale]
    bufferOffset = _serializer.float32(obj.scale, buffer, bufferOffset);
    // Serialize message field [ptamState]
    bufferOffset = _serializer.uint32(obj.ptamState, buffer, bufferOffset);
    // Serialize message field [scaleAccuracy]
    bufferOffset = _serializer.float32(obj.scaleAccuracy, buffer, bufferOffset);
    // Serialize message field [droneState]
    bufferOffset = _serializer.uint32(obj.droneState, buffer, bufferOffset);
    // Serialize message field [batteryPercent]
    bufferOffset = _serializer.float32(obj.batteryPercent, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type filter_state
    let len;
    let data = new filter_state(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dx]
    data.dx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dy]
    data.dy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dz]
    data.dz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dyaw]
    data.dyaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [scale]
    data.scale = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ptamState]
    data.ptamState = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [scaleAccuracy]
    data.scaleAccuracy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [droneState]
    data.droneState = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [batteryPercent]
    data.batteryPercent = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tum_ardrone/filter_state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '33f8050d082c4ebadff26def43dcfd15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # constants
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new filter_state(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.dx !== undefined) {
      resolved.dx = msg.dx;
    }
    else {
      resolved.dx = 0.0
    }

    if (msg.dy !== undefined) {
      resolved.dy = msg.dy;
    }
    else {
      resolved.dy = 0.0
    }

    if (msg.dz !== undefined) {
      resolved.dz = msg.dz;
    }
    else {
      resolved.dz = 0.0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.dyaw !== undefined) {
      resolved.dyaw = msg.dyaw;
    }
    else {
      resolved.dyaw = 0.0
    }

    if (msg.scale !== undefined) {
      resolved.scale = msg.scale;
    }
    else {
      resolved.scale = 0.0
    }

    if (msg.ptamState !== undefined) {
      resolved.ptamState = msg.ptamState;
    }
    else {
      resolved.ptamState = 0
    }

    if (msg.scaleAccuracy !== undefined) {
      resolved.scaleAccuracy = msg.scaleAccuracy;
    }
    else {
      resolved.scaleAccuracy = 0.0
    }

    if (msg.droneState !== undefined) {
      resolved.droneState = msg.droneState;
    }
    else {
      resolved.droneState = 0
    }

    if (msg.batteryPercent !== undefined) {
      resolved.batteryPercent = msg.batteryPercent;
    }
    else {
      resolved.batteryPercent = 0.0
    }

    return resolved;
    }
};

// Constants for message
filter_state.Constants = {
  PTAM_IDLE: 0,
  PTAM_INITIALIZING: 1,
  PTAM_LOST: 2,
  PTAM_GOOD: 3,
  PTAM_BEST: 4,
  PTAM_TOOKKF: 5,
  PTAM_FALSEPOSITIVE: 6,
}

module.exports = filter_state;
