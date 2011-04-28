/******* Reflecting the Stars *********
Radio Test - SLAVE CODE
Version: 0.1
Authors: Adam Berenzweig
Date: 2010-08-15
----------
Library for the radio protocol between master and slave.
**************************************/

#include <PrintUtil.h>
#include "WProgram.h"
#include "RtsMessage.h"

RtsMessage::RtsMessage(byte* raw_bytes) {
  message_ = raw_bytes;
  write_protect_ = 1;
  num_ids_ = 0;
}

void RtsMessage::DebugPrint() const {
  DPrintByte("cmd", message_[0]);
  DPrint(" : ");
  for (int i = 0; i < RTS_MSG_PARAM_SIZE; ++i) {
    DPrintByte(message_[RTS_MSG_COMMAND_BYTES + i]);
    DPrint(" ");
  }
  DPrint(" : ");
  for (int i = RTS_MSG_COMMAND_BYTES + RTS_MSG_PARAM_SIZE;
       i < RTS_MESSAGE_SIZE; ++i) {
    if (message_[i] == 0) {
      break;
    }
    DPrintByte(message_[i]);
  }
  DPrintln();
}

// Zero-out the data buffer before starting a new message.
// It's important that this gets called before we send a message,
// because the data buffer will be re-used between messages.
// We enforce that this gets called by using the write_protect_ bit.
void RtsMessage::initWrite() {
  for (int i = 0; i < RTS_MESSAGE_SIZE; ++i) {
    message_[i] = 0;
  }
  write_protect_ = 0;
}

bool RtsMessage::writeByteAt(int index, byte value) {
  if (index >= RTS_MESSAGE_SIZE) {
    DPRINT("OOB");
    return 0;
  }
  if (write_protect_) {
    DPRINT("write protected");
    return 0;
  }
  message_[index] = value;
  return 1;
}

// TODO(madadam): Use RTS_MSG_COMMAND_BYTES, don't assume single byte.
byte RtsMessage::command() {
  return message_[0];
}

void RtsMessage::setCommand(byte command) {
  writeByteAt(0, command);
}

byte RtsMessage::addId(byte id) {
  if (writeByteAt(num_ids_ + RTS_MSG_COMMAND_BYTES + RTS_MSG_PARAM_SIZE, id)) {
    ++num_ids_;
    return num_ids_;
  }
  return 0;
}

byte RtsMessage::getId(byte index) const {
  if (index >= RTS_MSG_MAX_IDS) {
    DPRINT("OOB");
    return 0;
  }
  return message_[index + RTS_MSG_COMMAND_BYTES + RTS_MSG_PARAM_SIZE];
}

bool RtsMessage::addParam(int param_id, byte value) {
  if (param_id >= RTS_MSG_PARAM_SIZE) {
    DPRINT("OOB");
    return 0;
  }
  writeByteAt(param_id + RTS_MSG_COMMAND_BYTES, value);
}

byte RtsMessage::getParam(int param_id) const {
  if (param_id >= RTS_MSG_PARAM_SIZE) {
    DPRINT("OOB");
    return 0;
  }
  return message_[param_id + RTS_MSG_COMMAND_BYTES];
}

byte RtsMessage::getMyState(byte my_id) const {
  byte command = message_[0];
  byte message_for_me = 0;

  // Decode the id list if necessary.
  // Could do if command > ALL_TWINKLE, but that's dangerous, the enum could
  // change.
  if (command == SELECT_OFF ||
      command == SELECT_CONSTELLATION ||
      command == SELECT_TWINKLE) {
    // Skip the command and param bytes.
    for (int i = RTS_MSG_PARAM_SIZE + RTS_MSG_COMMAND_BYTES;
         i < RTS_MESSAGE_SIZE; ++i) {
      if (my_id == message_[i]) {
        message_for_me = 1;
        break;
      }
      if (0 == message_[i]) {
        // Zero means end-of-list.
        break;
      }
    }
  }

  byte state = 0;
  switch (command) {
    case ALL_OFF:
      state = LED_OFF;
      break;
    case ALL_CONSTELLATION:
      state = LED_CONSTELLATION;
      break;
    case ALL_TWINKLE:
      state = LED_TWINKLE;
      break;
    case SELECT_OFF:
      state = message_for_me ? LED_OFF : IGNORE;
      break;
    case SELECT_CONSTELLATION:
      state = message_for_me ? LED_CONSTELLATION : LED_OFF;
      break;
    case SELECT_TWINKLE:
      state = message_for_me ? LED_TWINKLE : IGNORE;
      break;
    case ALL_SLEEP:
      state = SLEEP_NOW;
      break;
    case ALL_AT_EASE:
      state = AT_EASE;
      break;
    case ALL_ATTENTION:
      state = ATTENTION;
      break;
    default:
      state = IGNORE;
      break;
  }
  return state;
}

byte RtsMessage::checksum() const {
  byte sum = 0;
  for (int i = 0; i < RTS_MESSAGE_SIZE; ++i) {
    sum += message_[i];
  }
  return sum;
}
