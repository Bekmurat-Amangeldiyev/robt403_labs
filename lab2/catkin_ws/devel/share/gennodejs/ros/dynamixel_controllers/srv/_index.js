
"use strict";

let SetSpeed = require('./SetSpeed.js')
let StartController = require('./StartController.js')
let SetCompliancePunch = require('./SetCompliancePunch.js')
let SetComplianceMargin = require('./SetComplianceMargin.js')
let SetTorqueLimit = require('./SetTorqueLimit.js')
let StopController = require('./StopController.js')
let RestartController = require('./RestartController.js')
let SetComplianceSlope = require('./SetComplianceSlope.js')
let TorqueEnable = require('./TorqueEnable.js')

module.exports = {
  SetSpeed: SetSpeed,
  StartController: StartController,
  SetCompliancePunch: SetCompliancePunch,
  SetComplianceMargin: SetComplianceMargin,
  SetTorqueLimit: SetTorqueLimit,
  StopController: StopController,
  RestartController: RestartController,
  SetComplianceSlope: SetComplianceSlope,
  TorqueEnable: TorqueEnable,
};
