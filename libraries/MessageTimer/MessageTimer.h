/******* Reflecting The Stars ***********
MessageTimer
Author: Adam Berenzweig
Date: 2011-07-19
*****************************************/

// MessageTimer: A class that changes messages after a configurable duration.
// Given an array of TimedMessage structs, the timer will move from one message
// to the next after TimedMessage.duration_ms has elapsed.
//
// It's not really a timer -- it doesn't act by itself.  You need to call
// MessageTimer::MaybeChangeMessage() periodically with the current time,
// and the timer will switch messages if enough time has elapsed.
//
// To "stop" the timer, simply stop calling MaybeChangeMessage().

// TODO(madadam):
// It would be nice to support programmatic sending of messages in addition to
// timed messages.  It doesn't quite seem to belong in this class, but something
// like SetMessage() and SendNMessages().  Ideally it could share the underlying
// data buffer with this class so we don't need to use more memory.
// You could just call SetMessageFromString(), but that seems a little gross.

#ifndef MESSAGE_TIMER_H
#define MESSAGE_TIMER_H

#include <RtsMessage.h>
#include <RtsMessageParser.h>
#include <RtsUtil.h>
#include "WProgram.h"

// Big enough for the longest test_message.
// FIXME: Make this bigger, this is dangerously small.  We were running
// out of space in the Master.
#define BUFLEN 160

struct TimedMessage {
  // How long to play this message for.  0 means indefnitely.
  unsigned long duration_ms;

  // The message, in the format parseable by RtsMessageParser.
  char* message;
};

class MessageTimer {
 public:
  MessageTimer() : rts_message_(message_data_) {}

  const RtsMessage& rts_message() const { return rts_message_; }
  byte* message_data() { return message_data_; }

  // It's an error to call this before StartWithMessages.
  const char* current_message_string() const { return current_message_string_; }

  // num_messages must be less than 256.
  void StartWithMessages(TimedMessage* messages, byte num_messages) {
    messages_ = messages;
    num_messages_ = num_messages;
    SetCurrentMessage(0);
    ResetTimer();
  }

  void SetCurrentMessage(byte current_message) {
    current_message_ = current_message;
    SetTimedMessage(messages_[current_message_]);
  }

  void SetTimedMessage(const TimedMessage& timed_message) {
    duration_ms_ = timed_message.duration_ms;
    SetMessageFromString(timed_message.message);
    ResetTimer();
  }

  // Change to the next message if enough time has elapsed.
  // Call this periodically, see the comment at the top of the file.
  // Returns true if the message changed.
  bool MaybeChangeMessage(unsigned long now) {
    // duration_ms_ == 0 means never transition automatically.
    if (duration_ms_ > 0 && IsExpired(now)) {
      current_message_ = (current_message_ + 1) % num_messages_;
      SetCurrentMessage(current_message_);
      return true;
    }
    return false;
  }

  void ResetTimer() {
    last_message_time_ = millis();
  }

  bool IsExpired(unsigned long now) {
    return IsTimerExpired(now, &last_message_time_, duration_ms_);
  }

  // Set a message directly.  Doesn't affect the timer.
  void SetMessageFromString(const char* message) {
    // TODO(madadam): This is a bit dangerous when called by an external caller,
    // because we could be holding a pointer to a string that disappears.
    current_message_string_ = message;
    // Make a copy because ParseRtsMessage is destructive.
    // TODO(madadam): Do this nondestructively, avoid the copy, get rid of buf_.
    strncpy(buf_, message, BUFLEN - 1);
    ParseRtsMessageFromString(buf_, &rts_message_);
  }

 private:
  // Buffer for parsing string commands.
  char buf_[BUFLEN];

  byte message_data_[RTS_MESSAGE_SIZE];
  RtsMessage rts_message_;
  unsigned long last_message_time_;
  // Message duration.  0 means forever.
  unsigned long duration_ms_;

  TimedMessage* messages_;
  // These should be ints, but I'm being frugal.
  byte num_messages_;
  byte current_message_;
  const char* current_message_string_;
};

#endif  // MESSAGE_TIMER_H
