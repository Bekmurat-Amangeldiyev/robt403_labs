
"use strict";

let coord = require('./coord.js');
let contact = require('./contact.js');
let accelerometr = require('./accelerometr.js');
let tactile = require('./tactile.js');
let rigid = require('./rigid.js');
let newtactile = require('./newtactile.js');
let fsrInput = require('./fsrInput.js');
let state = require('./state.js');

module.exports = {
  coord: coord,
  contact: contact,
  accelerometr: accelerometr,
  tactile: tactile,
  rigid: rigid,
  newtactile: newtactile,
  fsrInput: fsrInput,
  state: state,
};
