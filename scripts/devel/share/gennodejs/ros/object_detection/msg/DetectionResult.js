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
      this.upper_left = null;
      this.upper_right = null;
      this.lower_left = null;
      this.lower_right = null;
    }
    else {
      if (initObj.hasOwnProperty('out_class')) {
        this.out_class = initObj.out_class
      }
      else {
        this.out_class = '';
      }
      if (initObj.hasOwnProperty('out_score')) {
        this.out_score = initObj.out_score
      }
      else {
        this.out_score = 0.0;
      }
      if (initObj.hasOwnProperty('upper_left')) {
        this.upper_left = initObj.upper_left
      }
      else {
        this.upper_left = 0;
      }
      if (initObj.hasOwnProperty('upper_right')) {
        this.upper_right = initObj.upper_right
      }
      else {
        this.upper_right = 0;
      }
      if (initObj.hasOwnProperty('lower_left')) {
        this.lower_left = initObj.lower_left
      }
      else {
        this.lower_left = 0;
      }
      if (initObj.hasOwnProperty('lower_right')) {
        this.lower_right = initObj.lower_right
      }
      else {
        this.lower_right = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DetectionResult
    // Serialize message field [out_class]
    bufferOffset = _serializer.string(obj.out_class, buffer, bufferOffset);
    // Serialize message field [out_score]
    bufferOffset = _serializer.float32(obj.out_score, buffer, bufferOffset);
    // Serialize message field [upper_left]
    bufferOffset = _serializer.uint32(obj.upper_left, buffer, bufferOffset);
    // Serialize message field [upper_right]
    bufferOffset = _serializer.uint32(obj.upper_right, buffer, bufferOffset);
    // Serialize message field [lower_left]
    bufferOffset = _serializer.uint32(obj.lower_left, buffer, bufferOffset);
    // Serialize message field [lower_right]
    bufferOffset = _serializer.uint32(obj.lower_right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DetectionResult
    let len;
    let data = new DetectionResult(null);
    // Deserialize message field [out_class]
    data.out_class = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [out_score]
    data.out_score = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [upper_left]
    data.upper_left = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [upper_right]
    data.upper_right = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [lower_left]
    data.lower_left = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [lower_right]
    data.lower_right = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.out_class.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_detection/DetectionResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5fb792480f070cbc69af8da53d614697';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string out_class
    float32 out_score
    uint32 upper_left
    uint32 upper_right
    uint32 lower_left
    uint32 lower_right
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
      resolved.out_class = ''
    }

    if (msg.out_score !== undefined) {
      resolved.out_score = msg.out_score;
    }
    else {
      resolved.out_score = 0.0
    }

    if (msg.upper_left !== undefined) {
      resolved.upper_left = msg.upper_left;
    }
    else {
      resolved.upper_left = 0
    }

    if (msg.upper_right !== undefined) {
      resolved.upper_right = msg.upper_right;
    }
    else {
      resolved.upper_right = 0
    }

    if (msg.lower_left !== undefined) {
      resolved.lower_left = msg.lower_left;
    }
    else {
      resolved.lower_left = 0
    }

    if (msg.lower_right !== undefined) {
      resolved.lower_right = msg.lower_right;
    }
    else {
      resolved.lower_right = 0
    }

    return resolved;
    }
};

module.exports = DetectionResult;
