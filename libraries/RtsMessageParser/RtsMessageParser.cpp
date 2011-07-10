/******* Reflecting the Stars *********
Authors: Adam Berenzweig
Date: 2010-08-15
----------
Parses human-readable commands into RtsMessages.
**************************************/

#include "RtsMessageParser.h"
#include <PrintUtil.h>
#include <stdio.h>
#include <string.h>
#include <avr/pgmspace.h>

// NOTE: Keep this in sync with the RTS_COMMANDS enum in RtsMessage.h
#define MAX_CMD_NAME_SIZE 7
prog_char cmd_name0[] PROGMEM = "IGNORE";
prog_char cmd_name1[] PROGMEM = "OFF";
prog_char cmd_name2[] PROGMEM = "CST";
prog_char cmd_name3[] PROGMEM = "TWK";
prog_char cmd_name4[] PROGMEM = "RESET";
prog_char cmd_name5[] PROGMEM = "SLEEP";
prog_char cmd_name6[] PROGMEM = "ATTN";
prog_char cmd_name7[] PROGMEM = "ATEZ";
prog_char cmd_name8[] PROGMEM = "SW";
prog_char cmd_name9[] PROGMEM = "STATUS";

PROGMEM const char *cmd_names[] = {
  cmd_name0,
  cmd_name1,
  cmd_name2,
  cmd_name3,
  cmd_name4,
  cmd_name5,
  cmd_name6,
  cmd_name7,
  cmd_name8,
  cmd_name9,
};

// Returns -1 on error.
int CommandFromName(const char* name) {
  for (int i = 0; i < NUM_RTS_COMMANDS; ++i) {
    //strcpy_P(buf, (char*)pgm_read_word(&(cmd_names[i])));
    //Serial.println(buf);
    if (!strncmp_P(name, (char*)pgm_read_word(&(cmd_names[i])),
                   MAX_CMD_NAME_SIZE)) {
      return i;
    }
  }
  return -1;
}

bool ParseRtsMessageFromString(char* input, RtsMessage* rts_message) {
  // Printing here slows down star wars mode.
  // DPrint(input);
  // DPrintln();

  int tok_num = 0;
  byte command = NUM_RTS_COMMANDS;
  rts_message->initWrite();
  char* tok = strtok(input, " \n");
  while (tok != NULL) {
    if (tok_num == 0) {
      // The first token is the command.
      int maybe_cmd = CommandFromName(tok);
      if (maybe_cmd < 0) {
        DPrint("Invalid cmd");
        return false;
      }
      command = maybe_cmd;
      rts_message->setCommand(command);
    } else if (tok_num < RTS_MSG_PARAM_SIZE + RTS_MSG_COMMAND_BYTES &&
               (command == RTS_CONSTELLATION ||
                command == RTS_TWINKLE ||
                command == RTS_SLEEP ||
                command == RTS_STAR_WARS)) {
      // It's a param.
      int val = atoi(tok);
      if (val > 255 || val < 0) {
        DPrint("bad prm");
        return false;
      }
      rts_message->addParam(tok_num - 1, val);
    } else {
      // It's an ID.
      int id = atoi(tok);
      if (id > 255 || id < 0) {
        DPrint("bad id");
        return false;
      }
      rts_message->addId(id);
    }

    tok = strtok(NULL, " \n");
    tok_num++;
  }
  return true;
}

#define SERIALCMDTERMINATOR 10  // <cr>

// Read available data from the serial buffer up to a newline or buflen.
// Return true iff we read something.
bool ReadMessageStringFromSerial(byte* buffer, byte buflen) {
  static byte pos = 0;
  while (Serial.available() && pos < buflen) {
    byte data = Serial.read();
    // Ganky! Arduino IDE doesn't send newline or cr. So use '$' also.
    if (data == SERIALCMDTERMINATOR || data == '$') {
      DPrintByte("bytes read", pos);
      DPrintln();
      // Null-terminate.
      buffer[pos] = NULL;
      pos = 0;
      return true;
    }
    buffer[pos] = data;
    ++pos;
  }
  if (pos >= buflen) {
    // Prevent buffer overrun, discard previous data.
    pos = 0;
  }
  return false;
}

bool ReadRtsMessageFromSerial(byte* buffer, byte buflen,
                              RtsMessage* rts_message) {
  if (ReadMessageStringFromSerial(buffer, buflen)) {
    return ParseRtsMessageFromString((char*)buffer, rts_message);
  }
  return false;
}

void FormatStarWarsMessage(char* buf,
                           const char* star_wars_prefix,
                           byte* ids,
                           int num_ids) {
  strcpy(buf, star_wars_prefix);
  char id_buf[5];
  for (int i = 0; i < num_ids && i < RTS_MSG_MAX_IDS; ++i) {
    sprintf(id_buf, " %d", ids[i]);
    strcat(buf, id_buf);
  }
}
