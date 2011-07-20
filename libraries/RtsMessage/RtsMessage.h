/******* Reflecting the Stars *********
Radio Test - SLAVE CODE
Version: 0.1
Authors: Adam Berenzweig
Date: 2010-08-15
----------
Library for the radio protocol between master and slave.
**************************************/

#ifndef RTS_MESSAGE_H 
#define RTS_MESSAGE_H 1

#include "WProgram.h"

// RadioSleep settings:
// Put these here so they are consistent between master and slave.
// The Transmitter will change the message at most every MESSAGE_PERIOD_MS.
#define MESSAGE_PERIOD_MS 5000
// The Transmiter will send one packet every PACKET_PERIOD_MS.
#define PACKET_PERIOD_MS 250

// **NOTE!!** If you edit this, you must also edit RtsMessageParser.cpp.
enum RTS_COMMAND {
  RTS_IGNORE,
  RTS_OFF,
  RTS_CONSTELLATION,
  RTS_TWINKLE,
  RTS_RESET,
  RTS_SLEEP,
  RTS_ATTENTION,
  RTS_AT_EASE,
  RTS_STAR_WARS,
  RTS_SEND_STATUS,
  NUM_RTS_COMMANDS
};

// If you add more params, you'll need to make RTS_MSG_PARAM_SIZE bigger.
enum TWINKLE_PARAMS {
  TWINKLE_SPEED,
  TWINKLE_OFF_TIME,
  TWINKLE_COLOR  // 0: blue, 1: white, 2: random, 3: both
};

enum STAR_PARAMS {
  STAR_FADE_IN,
  STAR_BLUE_BRIGHTNESS,
  STAR_WHITE_BRIGHTNESS
};

enum SLEEP_PARAMS {
  SLEEP_TIME_SEC,
  SLEEP_TIME_MULTIPLIER
};

// In addition to these, the first two IDs are used as the start and end ids.
enum STAR_WARS_PARAMS {
  STAR_WARS_NUM,
  STAR_WARS_TYPE  // 1: one-way, 2: two-way
};


// Messages are RTS_MESSAGE_SIZE bytes long.
// The structure of the bytes is:
//
// command: RTS_MSG_COMMAND_BYTES bytes
// params:  RTS_MSG_PARAM_SIZE bytes
// id_list: RTS_MESSAGE_SIZE - RTS_MSG_PARAM_SIZE - RTS_MSG_COMMAND_BYTES 
//
// So for a 32 byte message with RTS_MSG_PARAM_SIZE 3, there is room for 28
// ids in the id list.

// The total message size.
#define RTS_MESSAGE_SIZE 32

// Number of command bytes
#define RTS_MSG_COMMAND_BYTES 1

// Number of param bytes.
#define RTS_MSG_PARAM_SIZE 3

// Maximum number of IDs that can fit in one message.
#define RTS_MSG_MAX_IDS 28

class RtsMessage {
 public:
  // raw_bytes must be allocated for at least RTS_MESSAGE_SIZE
  RtsMessage(byte* raw_bytes);
  
  void DebugPrint() const;

  // Return the RTS_COMMAND that this slave should run based on the message.
  byte getMyState(byte my_id) const;

  // You must call this before writing to the message.
  void initWrite();

  byte command() const;
  void setCommand(byte command);

  // return 0 on error.
  byte addId(byte id);
  byte getId(byte index) const;

  // 0 <= param_id < RTS_MSG_PARAM_SIZE
  bool addParam(int param_id, byte value);
  byte getParam(int param_id) const;

  // Compute a very simple checksum over the message.
  byte checksum() const;

 private:
  // Return true if we wrote, false on error.
  bool writeByteAt(int index, byte value);

  byte command_;
  byte* message_;
  byte num_ids_;

  // See the comment in initWrite() for why this is important.
  byte write_protect_;
};

#endif  // RTS_MESSAGE_H
