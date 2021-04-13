
"use strict";

let YawrateCommand = require('./YawrateCommand.js');
let Altimeter = require('./Altimeter.js');
let RawMagnetic = require('./RawMagnetic.js');
let MotorCommand = require('./MotorCommand.js');
let VelocityZCommand = require('./VelocityZCommand.js');
let RawImu = require('./RawImu.js');
let MotorStatus = require('./MotorStatus.js');
let RuddersCommand = require('./RuddersCommand.js');
let HeightCommand = require('./HeightCommand.js');
let ControllerState = require('./ControllerState.js');
let Compass = require('./Compass.js');
let AttitudeCommand = require('./AttitudeCommand.js');
let HeadingCommand = require('./HeadingCommand.js');
let Supply = require('./Supply.js');
let PositionXYCommand = require('./PositionXYCommand.js');
let Altitude = require('./Altitude.js');
let RawRC = require('./RawRC.js');
let VelocityXYCommand = require('./VelocityXYCommand.js');
let ServoCommand = require('./ServoCommand.js');
let RC = require('./RC.js');
let ThrustCommand = require('./ThrustCommand.js');
let MotorPWM = require('./MotorPWM.js');

module.exports = {
  YawrateCommand: YawrateCommand,
  Altimeter: Altimeter,
  RawMagnetic: RawMagnetic,
  MotorCommand: MotorCommand,
  VelocityZCommand: VelocityZCommand,
  RawImu: RawImu,
  MotorStatus: MotorStatus,
  RuddersCommand: RuddersCommand,
  HeightCommand: HeightCommand,
  ControllerState: ControllerState,
  Compass: Compass,
  AttitudeCommand: AttitudeCommand,
  HeadingCommand: HeadingCommand,
  Supply: Supply,
  PositionXYCommand: PositionXYCommand,
  Altitude: Altitude,
  RawRC: RawRC,
  VelocityXYCommand: VelocityXYCommand,
  ServoCommand: ServoCommand,
  RC: RC,
  ThrustCommand: ThrustCommand,
  MotorPWM: MotorPWM,
};
