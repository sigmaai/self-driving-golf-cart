// Auto-generated. Do not edit!

// (in-package object_detection.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DetectionResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.out_class = null;
      this.out_score = null;
      this.location = null;
    }
    else {
      if (initObj.hasOwnProperty('out_class')) {
        this.out_class = initObj.out_class
      }
      else {
        this.out_class = 0;
      }
      if (initObj.hasOwnProperty('out_score')) {
        this.out_score = initObj.out_score
      }
      else {
        this.out_score = 0.0;
      }
      if (initObj.hasOwnProperty('location')) {
        this.location = initObj.location
      }
      else {
        this.location = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DetectionResult
    // Serialize message field [out_class]
    bufferOffset = _serializer.uint32(obj.out_class, buffer, bufferOffset);
    // Serialize message field [out_score]
    bufferOffset = _serializer.float32(obj.out_score, buffer, bufferOffset);
    // Serialize message field [location]
    bufferOffset = _arraySerializer.float32(obj.location, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DetectionResult
    let len;
    let data = new DetectionResult(null);
    // Deserialize message field [out_class]
    data.out_class = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [out_score]
    data.out_score = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [location]
    data.location = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.location.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_detection/DetectionResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3b7fae78ce58faba54a3eab2d7ca9e3c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 out_class
    float32 out_score
    float32[] location
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DetectionResult(null);
    if (msg.out_class !== undefined) {
      resolved.out_class = msg.out_class;
    }
    else {
      resolved.out_class = 0
    }

    if (msg.out_score !== undefined) {
      resolved.out_score = msg.out_score;
    }
    else {
      resolved.out_score = 0.0
    }

    if (msg.location !== undefined) {
      resolved.location = msg.location;
    }
    else {
      resolved.location = []
    }

    return resolved;
    }
};

module.exports = DetectionResult;
