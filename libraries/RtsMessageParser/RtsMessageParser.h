/******* Reflecting the Stars *********
Authors: Adam Berenzweig
Date: 2010-08-15
----------
Parses human-readable commands into RtsMessages.
**************************************/

#ifndef RTS_MESSAGE_PARSER_H 
#define RTS_MESSAGE_PARSER_H 1

#include <RtsMessage.h>
#include "HardwareSerial.h"
#include "WProgram.h"

bool ParseRtsMessageFromString(char* input, RtsMessage* rts_message);

// Read available data from the serial buffer up to a newline or buflen.
// The newline is stripped off and buffer is null-terminated.
// Return true iff we read something.
bool ReadMessageStringFromSerial(HardwareSerial* serial,
                                 byte* buffer,
                                 byte buflen);

// Read available data from the serial buffer up to a newline or buflen,
// and parse into the RtsMessage.
// Return true on success.
bool ReadRtsMessageFromSerial(byte* buffer, byte buflen,
                              RtsMessage* rts_message);

bool ReadRtsMessageFromSerial(HardwareSerial* serial,
                              byte* buffer, byte buflen,
                              RtsMessage* rts_message);

void FormatStarWarsMessage(char* buf,
                           const char* star_wars_prefix,
                           byte* ids,
                           int num_ids);
#endif  // RTS_MESSAGE_PARSER_H
