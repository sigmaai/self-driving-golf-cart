// Auto-generated. Do not edit!

// (in-package object_detection.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DetectionResult = require('./DetectionResult.js');

//-----------------------------------------------------------

class DetectionResults {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.results = null;
    }
    else {
      if (initObj.hasOwnProperty('results')) {
        this.results = initObj.results
      }
      else {
        this.results = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DetectionResults
    // Serialize message field [results]
    // Serialize the length for message field [results]
    bufferOffset = _serializer.uint32(obj.results.length, buffer, bufferOffset);
    obj.results.forEach((val) => {
      bufferOffset = DetectionResult.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DetectionResults
    let len;
    let data = new DetectionResults(null);
    // Deserialize message field [results]
    // Deserialize array length for message field [results]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.results = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.results[i] = DetectionResult.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.results.forEach((val) => {
      length += DetectionResult.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_detection/DetectionResults';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c90c5dbacc052c188fd5956bea4e6f91';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    DetectionResult[] results
    ================================================================================
    MSG: object_detection/DetectionResult
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
    const resolved = new DetectionResults(null);
    if (msg.results !== undefined) {
      resolved.results = new Array(msg.results.length);
      for (let i = 0; i < resolved.results.length; ++i) {
        resolved.results[i] = DetectionResult.Resolve(msg.results[i]);
      }
    }
    else {
      resolved.results = []
    }

    return resolved;
    }
};

module.exports = DetectionResults;
