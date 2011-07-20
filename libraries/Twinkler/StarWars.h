// FIXME  make star wars work with MessageTimer.  Probably easiest to rip this
// out of the master and put into the Mega.

char* star_wars_prefix = "CST 0 0 255";

// Return true if we ran a star wars sequence.
bool MaybeRunStarWars() {
  // Assume that the message has already been written.
  const RtsMessage& message = message_timer_.rts_message();
  // We can't keep the message around while star wars is running because the
  // underlying rtsMessageData buffer gets reused by each message in the
  // sequence.  So extract all params now.  If that becomes a pain, then make a
  // separate buffer for the star wars message and pass the full message to
  // RunStarWars().
  if (message.command() != RTS_STAR_WARS) {
    return false;
  }
  byte num = message.getParam(STAR_WARS_NUM);
  byte type = message.getParam(STAR_WARS_TYPE);
  byte min_id = message.getId(0);
  byte max_id = message.getId(1);
  RunStarWars(num, type, min_id, max_id);
  // After we're done, transition to the next message.
  SetNextMessage();
  message_timer_.ResetTimer();
  return true;
}

void SetStarWarsMessage(byte* ids, int num_ids) {
  // FIXME: Which buffer to use?  Maybe move this out of the master and into the
  // Mega, have plenty of buffer space there.
  FormatStarWarsMessage(buf, star_wars_prefix, ids, num_ids);
  message_timer_.SetMessageFromString(buf);
}

void RunStarWars(byte num, byte type, byte min_puck, byte max_puck) {
  DPrintln("Running Star Wars.");
  SetMessage(all_attention_message);
  SendNMessages(16);
#define kNumPucksPerMessage 2

  byte ids[kNumPucksPerMessage];
  for (int j = 0; j < num; ++j) {
    for (int i = min_puck; i <= max_puck; ++i) {
      // Sending ATTENTION in between every message gives a pretty good
      // chance that all pucks will get it.  If they don't they'll go into
      // radio sleep each time they receive a packet.
      SetMessage(all_attention_message);
      SendOneMessage();

      ids[0] = i;
      if (type == 2) {
        // both ways
        ids[1] = max_puck - i + min_puck;
      }
      SetStarWarsMessage(ids, kNumPucksPerMessage);
      SendOneMessage();
    }
    // Now pause for a bit, but keep sending messages so pucks don't sleep.
    SetMessage(all_off_message);
    SendNMessages(4);
  }
  // A bit dangerous, if somebody misses this, they'll run out of batteries
  // real fast.  We send regular AT_EASE messages to prevent that.
  SetMessage(at_ease_message);
  SendNMessages(10);
}

