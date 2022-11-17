// Auto-generated. Do not edit!

// (in-package ikid_motion_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class cmd_walk {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sx = null;
      this.sy = null;
      this.var_theta = null;
    }
    else {
      if (initObj.hasOwnProperty('sx')) {
        this.sx = initObj.sx
      }
      else {
        this.sx = 0.0;
      }
      if (initObj.hasOwnProperty('sy')) {
        this.sy = initObj.sy
      }
      else {
        this.sy = 0.0;
      }
      if (initObj.hasOwnProperty('var_theta')) {
        this.var_theta = initObj.var_theta
      }
      else {
        this.var_theta = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cmd_walk
    // Serialize message field [sx]
    bufferOffset = _serializer.float64(obj.sx, buffer, bufferOffset);
    // Serialize message field [sy]
    bufferOffset = _serializer.float64(obj.sy, buffer, bufferOffset);
    // Serialize message field [var_theta]
    bufferOffset = _serializer.float64(obj.var_theta, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cmd_walk
    let len;
    let data = new cmd_walk(null);
    // Deserialize message field [sx]
    data.sx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [sy]
    data.sy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [var_theta]
    data.var_theta = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ikid_motion_control/cmd_walk';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '642b1806da11ae8975a1c844cb320e6d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 sx
    float64 sy
    float64 var_theta
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cmd_walk(null);
    if (msg.sx !== undefined) {
      resolved.sx = msg.sx;
    }
    else {
      resolved.sx = 0.0
    }

    if (msg.sy !== undefined) {
      resolved.sy = msg.sy;
    }
    else {
      resolved.sy = 0.0
    }

    if (msg.var_theta !== undefined) {
      resolved.var_theta = msg.var_theta;
    }
    else {
      resolved.var_theta = 0.0
    }

    return resolved;
    }
};

module.exports = cmd_walk;
