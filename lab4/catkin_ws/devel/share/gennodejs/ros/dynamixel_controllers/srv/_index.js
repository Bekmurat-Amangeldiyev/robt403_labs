
"use strict";

let TorqueEnable = require('./TorqueEnable.js')
let StopController = require('./StopController.js')
let StartController = require('./StartController.js')
let RestartController = require('./RestartController.js')
let SetComplianceSlope = require('./SetComplianceSlope.js')
let SetTorqueLimit = require('./SetTorqueLimit.js')
let SetCompliancePunch = require('./SetCompliancePunch.js')
let SetComplianceMargin = require('./SetComplianceMargin.js')
let SetSpeed = require('./SetSpeed.js')

module.exports = {
  TorqueEnable: TorqueEnable,
  StopController: StopController,
  StartController: StartController,
  RestartController: RestartController,
  SetComplianceSlope: SetComplianceSlope,
  SetTorqueLimit: SetTorqueLimit,
  SetCompliancePunch: SetCompliancePunch,
  SetComplianceMargin: SetComplianceMargin,
  SetSpeed: SetSpeed,
};
