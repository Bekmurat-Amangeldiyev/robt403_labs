
"use strict";

let state = require('./state.js');
let contact = require('./contact.js');
let newtactile = require('./newtactile.js');
let coord = require('./coord.js');
let accelerometr = require('./accelerometr.js');
let tactile = require('./tactile.js');
let fsrInput = require('./fsrInput.js');
let rigid = require('./rigid.js');

module.exports = {
  state: state,
  contact: contact,
  newtactile: newtactile,
  coord: coord,
  accelerometr: accelerometr,
  tactile: tactile,
  fsrInput: fsrInput,
  rigid: rigid,
};
