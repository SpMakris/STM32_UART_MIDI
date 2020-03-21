/*!
 * Original Author: Francois Best
 *
 * This is a direct port of the arduino MIDI library to STM32 using HAL driver commands.
 * Unline the Arduino version, this class assumes that the main program handles all peripheral
 * initializations. Create a MidiInterface object and run the .begin(Channel, *serial_in, *serial_out)
 * to specify the in/out uart ports to be used (they can be a different USART peripheral for each).
 * The rest of the functionality is identical to that on arduino: run read() as often as possible and
 * set callbacks to run your handles for each MIDI message type received.
 *
 */

#include "MIDI.h"
#include "stm32f0xx_hal.h"
MidiInterface::MidiInterface() {
	mRunningStatus_TX = InvalidType;
	mRunningStatus_RX = InvalidType;

	mPendingMessageIndex = 0;
	mPendingMessageExpectedLenght = 0;

	mCurrentRpnNumber = 0xffff;
	mCurrentNrpnNumber = 0xffff;

	mMessage.valid = false;
	mMessage.type = InvalidType;
	mMessage.channel = 0;
	mMessage.data1 = 0;
	mMessage.data2 = 0;
	mMessage.SysExMaxSize = Settings.SysExMaxSize;
	mThruFilterMode = Thru::Full;
	mThruActivated = true;

	mInputChannel = 0;
	mRunningStatus_RX = InvalidType;
	mRunningStatus_TX = InvalidType;
	mPendingMessageExpectedLenght = 0;
	mPendingMessageIndex = 0;
	mCurrentRpnNumber = 0xffff;
	mCurrentNrpnNumber = 0xffff;
	mThruActivated = false;
	mThruFilterMode = Thru::Off;
	mNoteOffCallback = 0;
	mNoteOnCallback = 0;
	mAfterTouchPolyCallback = 0;
	mControlChangeCallback = 0;
	mProgramChangeCallback = 0;
	mAfterTouchChannelCallback = 0;
	mPitchBendCallback = 0;
	mSystemExclusiveCallback = 0;
	mTimeCodeQuarterFrameCallback = 0;
	mSongPositionCallback = 0;
	mSongSelectCallback = 0;
	mTuneRequestCallback = 0;
	mClockCallback = 0;
	mStartCallback = 0;
	mContinueCallback = 0;
	mStopCallback = 0;
	mActiveSensingCallback = 0;
	mSystemResetCallback = 0;

}
MidiInterface::~MidiInterface() {
}

/*! \brief Call the begin method in the setup() function of the Arduino.

 All parameters are set to their default values:
 - Input channel set to 1 if no value is specified

 */

void MidiInterface::begin(Channel inChannel, UART_HandleTypeDef *huart_in,
		UART_HandleTypeDef *huart_out) {
	serial_in = *huart_in;
	serial_out = *huart_out;
	mInputChannel = inChannel;

}

/*! \brief Generate and send a MIDI message from the values given.
 \param inType    The message type (see type defines for reference)
 \param inData1   The first data byte.
 \param inData2   The second data byte (if the message contains only 1 data byte,
 set this one to 0).
 \param inChannel The output channel on which the message will be sent
 (values from 1 to 16). Note: you cannot send to OMNI.

 This is an internal method, use it only if you need to send raw data
 from your code, at your own risks.
 */

void MidiInterface::send(MidiType inType, DataByte inData1, DataByte inData2,
		Channel inChannel) {
	// Then test if channel is valid
	if (inChannel >= MIDI_CHANNEL_OFF || inChannel == MIDI_CHANNEL_OMNI
			|| inType < 0x80) {
		return; // Don't send anything
	}

	if (inType <= PitchBend)  // Channel messages
			{
// Protection: remove MSBs on data
		inData1 &= 0x7f;
		inData2 &= 0x7f;

		StatusByte status = getStatus(inType, inChannel);

		if (Settings.UseRunningStatus) {
			if (mRunningStatus_TX != status) {
// New message, memorise and send header
				mRunningStatus_TX = status;
				HAL_UART_Transmit(&serial_out, &mRunningStatus_TX, 1,
				MidiTimeout);
			}
		} else {
			// Don't care about running status, send the status byte.
			HAL_UART_Transmit(&serial_out, &status, 1, MidiTimeout);
		}

// Then send data
		HAL_UART_Transmit(&serial_out, &inData1, 1, MidiTimeout);
		if (inType != ProgramChange && inType != AfterTouchChannel) {
			HAL_UART_Transmit(&serial_out, &inData2, 1, MidiTimeout);
		}
	} else if (inType >= Clock && inType <= SystemReset) {
		sendRealTime(inType); // System Real-time and 1 byte.
	}
}

/*! \brief Send a Note On message
 \param inNoteNumber  Pitch value in the MIDI format (0 to 127).
 \param inVelocity    Note attack velocity (0 to 127). A NoteOn with 0 velocity
 is considered as a NoteOff.
 \param inChannel     The channel on which the message will be sent (1 to 16).

 Take a look at the values, names and frequencies of notes here:
 http://www.phys.unsw.edu.au/jw/notes.html
 */

void MidiInterface::sendNoteOn(DataByte inNoteNumber, DataByte inVelocity,
		Channel inChannel) {
	send(NoteOn, inNoteNumber, inVelocity, inChannel);
}

/*! \brief Send a Note Off message
 \param inNoteNumber  Pitch value in the MIDI format (0 to 127).
 \param inVelocity    Release velocity (0 to 127).
 \param inChannel     The channel on which the message will be sent (1 to 16).

 Note: you can send NoteOn with zero velocity to make a NoteOff, this is based
 on the Running Status principle, to avoid sending status messages and thus
 sending only NoteOn data. sendNoteOff will always send a real NoteOff message.
 Take a look at the values, names and frequencies of notes here:
 http://www.phys.unsw.edu.au/jw/notes.html
 */

void MidiInterface::sendNoteOff(DataByte inNoteNumber, DataByte inVelocity,
		Channel inChannel) {
	send(NoteOff, inNoteNumber, inVelocity, inChannel);
}

/*! \brief Send a Program Change message
 \param inProgramNumber The Program to select (0 to 127).
 \param inChannel       The channel on which the message will be sent (1 to 16).
 */

void MidiInterface::sendProgramChange(DataByte inProgramNumber,
		Channel inChannel) {
	send(ProgramChange, inProgramNumber, 0, inChannel);
}

/*! \brief Send a Control Change message
 \param inControlNumber The controller number (0 to 127).
 \param inControlValue  The value for the specified controller (0 to 127).
 \param inChannel       The channel on which the message will be sent (1 to 16).
 @see MidiControlChangeNumber
 */

void MidiInterface::sendControlChange(DataByte inControlNumber,
		DataByte inControlValue, Channel inChannel) {
	send(ControlChange, inControlNumber, inControlValue, inChannel);
}

/*! \brief Send a Polyphonic AfterTouch message (applies to a specified note)
 \param inNoteNumber  The note to apply AfterTouch to (0 to 127).
 \param inPressure    The amount of AfterTouch to apply (0 to 127).
 \param inChannel     The channel on which the message will be sent (1 to 16).
 Note: this method is deprecated and will be removed in a future revision of the
 library, @see sendAfterTouch to send polyphonic and monophonic AfterTouch messages.
 */

void MidiInterface::sendPolyPressure(DataByte inNoteNumber, DataByte inPressure,
		Channel inChannel) {
	send(AfterTouchPoly, inNoteNumber, inPressure, inChannel);
}

/*! \brief Send a MonoPhonic AfterTouch message (applies to all notes)
 \param inPressure    The amount of AfterTouch to apply to all notes.
 \param inChannel     The channel on which the message will be sent (1 to 16).
 */

void MidiInterface::sendAfterTouch(DataByte inPressure, Channel inChannel) {
	send(AfterTouchChannel, inPressure, 0, inChannel);
}

/*! \brief Send a Polyphonic AfterTouch message (applies to a specified note)
 \param inNoteNumber  The note to apply AfterTouch to (0 to 127).
 \param inPressure    The amount of AfterTouch to apply (0 to 127).
 \param inChannel     The channel on which the message will be sent (1 to 16).
 @see Replaces sendPolyPressure (which is now deprecated).
 */

void MidiInterface::sendAfterTouch(DataByte inNoteNumber, DataByte inPressure,
		Channel inChannel) {
	send(AfterTouchPoly, inNoteNumber, inPressure, inChannel);
}

/*! \brief Send a Pitch Bend message using a signed integer value.
 \param inPitchValue  The amount of bend to send (in a signed integer format),
 between MIDI_PITCHBEND_MIN and MIDI_PITCHBEND_MAX,
 center value is 0.
 \param inChannel     The channel on which the message will be sent (1 to 16).
 */

void MidiInterface::sendPitchBend(int inPitchValue, Channel inChannel) {
	unsigned bend = inPitchValue - MIDI_PITCHBEND_MIN;
	send(PitchBend, (bend & 0x7f), (bend >> 7) & 0x7f, inChannel);
}

/*! \brief Send a Pitch Bend message using a floating point value.
 \param inPitchValue  The amount of bend to send (in a floating point format),
 between -1.0f (maximum downwards bend)
 and +1.0f (max upwards bend), center value is 0.0f.
 \param inChannel     The channel on which the message will be sent (1 to 16).
 */

void MidiInterface::sendPitchBend(double inPitchValue, Channel inChannel) {
	int scale = inPitchValue > 0.0 ? MIDI_PITCHBEND_MAX : MIDI_PITCHBEND_MIN;
	int value = int(inPitchValue * double(scale));
	sendPitchBend(value, inChannel);
}

/*! \brief Generate and send a System Exclusive frame.
 \param inLength  The size of the array to send
 \param inArray   The byte array containing the data to send
 \param inArrayContainsBoundaries When set to 'true', 0xf0 & 0xf7 bytes
 (start & stop SysEx) will NOT be sent
 (and therefore must be included in the array).
 default value for ArrayContainsBoundaries is set to 'false' for compatibility
 with previous versions of the library.
 */

void MidiInterface::sendSysEx(uint8_t inLength, byte *inArray,
		bool inArrayContainsBoundaries) {
	bool writeBeginEndBytes = !inArrayContainsBoundaries;
	uint8_t StartByte = 0xf0;
	uint8_t EndByte = 0xf7;
	if (writeBeginEndBytes) {
		HAL_UART_Transmit(&serial_out, &StartByte, 1, MidiTimeout);
	}

	HAL_UART_Transmit(&serial_out, inArray, sizeof(inLength), MidiTimeout);

	if (writeBeginEndBytes) {
		HAL_UART_Transmit(&serial_out, &EndByte, 1, MidiTimeout);
	}

	if (Settings.UseRunningStatus) {
		mRunningStatus_TX = InvalidType;
	}
}

/*! \brief Send a Tune Request message.

 When a MIDI unit receives this message,
 it should tune its oscillators (if equipped with any).
 */

void MidiInterface::sendTuneRequest() {
	uint8_t TuneMessage = TuneRequest;
	HAL_UART_Transmit(&serial_out, &TuneMessage, 1, MidiTimeout);
	if (Settings.UseRunningStatus) {
		mRunningStatus_TX = InvalidType;
	}
}

/*! \brief Send a MIDI Time Code Quarter Frame.

 \param inTypeNibble      MTC type
 \param inValuesNibble    MTC data
 See MIDI Specification for more information.
 */

void MidiInterface::sendTimeCodeQuarterFrame(DataByte inTypeNibble,
		DataByte inValuesNibble) {
	byte data = (((inTypeNibble & 0x07) << 4) | (inValuesNibble & 0x0f));
	sendTimeCodeQuarterFrame(data);
}

/*! \brief Send a MIDI Time Code Quarter Frame.

 See MIDI Specification for more information.
 \param inData  if you want to encode directly the nibbles in your program,
 you can send the byte here.
 */

void MidiInterface::sendTimeCodeQuarterFrame(DataByte inData) {
	uint8_t TimeCodeMessage[] = { TimeCodeQuarterFrame, inData };
	HAL_UART_Transmit(&serial_out, TimeCodeMessage, 2, MidiTimeout);

	if (Settings.UseRunningStatus) {
		mRunningStatus_TX = InvalidType;
	}
}

/*! \brief Send a Song Position Pointer message.
 \param inBeats    The number of beats since the start of the song.
 */

void MidiInterface::sendSongPosition(unsigned inBeats) {
	uint8_t SongPositionMessage[] = { SongPosition, (uint8_t) (inBeats & 0x7f),
			(uint8_t) ((inBeats >> 7) & 0x7f) };
	HAL_UART_Transmit(&serial_out, SongPositionMessage, 3, MidiTimeout);

	if (Settings.UseRunningStatus) {
		mRunningStatus_TX = InvalidType;
	}
}

/*! \brief Send a Song Select message */

void MidiInterface::sendSongSelect(DataByte inSongNumber) {
	uint8_t SongSelectMessage[] =
			{ SongSelect, (uint8_t) (inSongNumber & 0x7f) };
	HAL_UART_Transmit(&serial_out, SongSelectMessage, 2, MidiTimeout);
	if (Settings.UseRunningStatus) {
		mRunningStatus_TX = InvalidType;
	}
}

/*! \brief Send a Real Time (one byte) message.

 \param inType    The available Real Time types are:
 Start, Stop, Continue, Clock, ActiveSensing and SystemReset.
 @see MidiType
 */

void MidiInterface::sendRealTime(MidiType inType) {
	// Do not invalidate Running Status for real-time messages
	// as they can be interleaved within any message.
	uint8_t RealTimeMessage;
	switch (inType) {
	case Clock:
	case Start:
	case Stop:
	case Continue:
	case ActiveSensing:
	case SystemReset:
		HAL_UART_Transmit(&serial_out, &RealTimeMessage, 1, MidiTimeout);
		break;
	default:
// Invalid Real Time marker
		break;
	}
}

/*! \brief Start a Registered Parameter Number frame.
 \param inNumber The 14-bit number of the RPN you want to select.
 \param inChannel The channel on which the message will be sent (1 to 16).
 */

void MidiInterface::beginRpn(unsigned inNumber, Channel inChannel) {
	if (mCurrentRpnNumber != inNumber) {
		byte numMsb = 0x7f & (inNumber >> 7);
		byte numLsb = 0x7f & inNumber;
		sendControlChange(RPNLSB, numLsb, inChannel);
		sendControlChange(RPNMSB, numMsb, inChannel);
		mCurrentRpnNumber = inNumber;
	}
}

/*! \brief Send a 14-bit value for the currently selected RPN number.
 \param inValue  The 14-bit value of the selected RPN.
 \param inChannel The channel on which the message will be sent (1 to 16).
 */

void MidiInterface::sendRpnValue(unsigned inValue, Channel inChannel) {
	;
	byte valMsb = 0x7f & (inValue >> 7);
	byte valLsb = 0x7f & inValue;
	sendControlChange(DataEntryMSB, valMsb, inChannel);
	sendControlChange(DataEntryLSB, valLsb, inChannel);
}

/*! \brief Send separate MSB/LSB values for the currently selected RPN number.
 \param inMsb The MSB part of the value to send. Meaning depends on RPN number.
 \param inLsb The LSB part of the value to send. Meaning depends on RPN number.
 \param inChannel The channel on which the message will be sent (1 to 16).
 */

void MidiInterface::sendRpnValue(byte inMsb,
byte inLsb, Channel inChannel) {
	sendControlChange(DataEntryMSB, inMsb, inChannel);
	sendControlChange(DataEntryLSB, inLsb, inChannel);
}

/* \brief Increment the value of the currently selected RPN number by the specified amount.
 \param inAmount The amount to add to the currently selected RPN value.
 */

void MidiInterface::sendRpnIncrement(byte inAmount, Channel inChannel) {
	sendControlChange(DataIncrement, inAmount, inChannel);
}

/* \brief Decrement the value of the currently selected RPN number by the specified amount.
 \param inAmount The amount to subtract to the currently selected RPN value.
 */

void MidiInterface::sendRpnDecrement(byte inAmount, Channel inChannel) {
	sendControlChange(DataDecrement, inAmount, inChannel);
}

/*! \brief Terminate an RPN frame.
 This will send a Null Function to deselect the currently selected RPN.
 \param inChannel The channel on which the message will be sent (1 to 16).
 */

void MidiInterface::endRpn(Channel inChannel) {
	sendControlChange(RPNLSB, 0x7f, inChannel);
	sendControlChange(RPNMSB, 0x7f, inChannel);
	mCurrentRpnNumber = 0xffff;
}

/*! \brief Start a Non-Registered Parameter Number frame.
 \param inNumber The 14-bit number of the NRPN you want to select.
 \param inChannel The channel on which the message will be sent (1 to 16).
 */

void MidiInterface::beginNrpn(unsigned inNumber, Channel inChannel) {
	if (mCurrentNrpnNumber != inNumber) {
		byte numMsb = 0x7f & (inNumber >> 7);
		byte numLsb = 0x7f & inNumber;
		sendControlChange(NRPNLSB, numLsb, inChannel);
		sendControlChange(NRPNMSB, numMsb, inChannel);
		mCurrentNrpnNumber = inNumber;
	}
}

/*! \brief Send a 14-bit value for the currently selected NRPN number.
 \param inValue  The 14-bit value of the selected NRPN.
 \param inChannel The channel on which the message will be sent (1 to 16).
 */

void MidiInterface::sendNrpnValue(unsigned inValue, Channel inChannel) {
	;
	byte valMsb = 0x7f & (inValue >> 7);
	byte valLsb = 0x7f & inValue;
	sendControlChange(DataEntryMSB, valMsb, inChannel);
	sendControlChange(DataEntryLSB, valLsb, inChannel);
}

/*! \brief Send separate MSB/LSB values for the currently selected NRPN number.
 \param inMsb The MSB part of the value to send. Meaning depends on NRPN number.
 \param inLsb The LSB part of the value to send. Meaning depends on NRPN number.
 \param inChannel The channel on which the message will be sent (1 to 16).
 */

void MidiInterface::sendNrpnValue(byte inMsb,
byte inLsb, Channel inChannel) {
	sendControlChange(DataEntryMSB, inMsb, inChannel);
	sendControlChange(DataEntryLSB, inLsb, inChannel);
}

/* \brief Increment the value of the currently selected NRPN number by the specified amount.
 \param inAmount The amount to add to the currently selected NRPN value.
 */

void MidiInterface::sendNrpnIncrement(
byte inAmount, Channel inChannel) {
	sendControlChange(DataIncrement, inAmount, inChannel);
}

/* \brief Decrement the value of the currently selected NRPN number by the specified amount.
 \param inAmount The amount to subtract to the currently selected NRPN value.
 */

void MidiInterface::sendNrpnDecrement(
byte inAmount, Channel inChannel) {
	sendControlChange(DataDecrement, inAmount, inChannel);
}

/*! \brief Terminate an NRPN frame.
 This will send a Null Function to deselect the currently selected NRPN.
 \param inChannel The channel on which the message will be sent (1 to 16).
 */

void MidiInterface::endNrpn(Channel inChannel) {
	sendControlChange(NRPNLSB, 0x7f, inChannel);
	sendControlChange(NRPNMSB, 0x7f, inChannel);
	mCurrentNrpnNumber = 0xffff;
}

/*! @} */ // End of doc group MIDI Output
// -----------------------------------------------------------------------------
StatusByte MidiInterface::getStatus(MidiType inType, Channel inChannel) {
	return ((byte) inType | ((inChannel - 1) & 0x0f));
}

/*! \brief Read messages from the serial port using the main input channel.

 \return True if a valid message has been stored in the structure, false if not.
 A valid message is a message that matches the input channel. \n\n
 If the Thru is enabled and the message matches the filter,
 it is sent back on the MIDI output.
 @see see setInputChannel()
 */

bool MidiInterface::read() {
	return read(mInputChannel);
}

/*! \brief Read messages on a specified channel.
 */

bool MidiInterface::read(Channel inChannel) {
	if (inChannel >= MIDI_CHANNEL_OFF)
		return false; // MIDI Input disabled.

	if (!parse())
		return false;

	handleNullVelocityNoteOnAsNoteOff();
	bool channelMatch = inputFilter(inChannel);

	if (channelMatch) {
		launchCallback();
	}

	thruFilter(inChannel);

	return channelMatch;
}

bool MidiInterface::parse2() {
	byte extracted;
	if (HAL_UART_Receive(&serial_in, &extracted, 1, MidiTimeout) != HAL_OK) {
		return false;
	}
	if (extracted == 0xf9 || extracted == 0xfd) {
		if (Settings.Use1ByteParsing) {
			return false;
		} else {
			return parse2();
		}
	}
	mPendingMessage[0] = extracted;
	switch (getTypeFromStatusByte(mPendingMessage[0])) {
	// 1 byte messages
	case Start:
	case Continue:
	case Stop:
	case Clock:
	case ActiveSensing:
	case SystemReset:
	case TuneRequest:
		// Handle the message type directly here.
		mMessage.type = getTypeFromStatusByte(mPendingMessage[0]);
		mMessage.channel = 0;
		mMessage.data1 = 0;
		mMessage.data2 = 0;
		mMessage.valid = true;

		// Do not reset all input attributes, Running Status must remain unchanged.
		// We still need to reset these
		mPendingMessageIndex = 0;
		mPendingMessageExpectedLenght = 0;

		return true;
		break;
	}
}
// Private method: MIDI parser

bool MidiInterface::parse() {
	byte extracted;
	if (HAL_UART_Receive(&serial_in, &extracted, 1, MidiTimeout) != HAL_OK) {
		return false;
	}

	// Parsing algorithm:
	// Get a byte from the serial buffer.
	// If there is no pending message to be recomposed, start a new one.
	//  - Find type and channel (if pertinent)
	//  - Look for other bytes in buffer, call parser recursively,
	//    until the message is assembled or the buffer is empty.
	// Else, add the extracted byte to the pending message, and check validity.
	// When the message is done, store it.

	// Ignore Undefined
	if (extracted == 0xf9 || extracted == 0xfd) {
		if (Settings.Use1ByteParsing) {
			return false;
		} else {
			return parse();
		}
	}

	if (mPendingMessageIndex == 0) {
// Start a new pending message
		mPendingMessage[0] = extracted;

// Check for running status first
		if (isChannelMessage(getTypeFromStatusByte(mRunningStatus_RX))) {
			// Only these types allow Running Status

			// If the status byte is not received, prepend it
			// to the pending message
			if (extracted < 0x80) {
				mPendingMessage[0] = mRunningStatus_RX;
				mPendingMessage[1] = extracted;
				mPendingMessageIndex = 1;
			}
			// Else: well, we received another status byte,
			// so the running status does not apply here.
			// It will be updated upon completion of this message.
		}

		switch (getTypeFromStatusByte(mPendingMessage[0])) {
// 1 byte messages
		case Start:
		case Continue:
		case Stop:
		case Clock:
		case ActiveSensing:
		case SystemReset:
		case TuneRequest:
			// Handle the message type directly here.
			mMessage.type = getTypeFromStatusByte(mPendingMessage[0]);
			mMessage.channel = 0;
			mMessage.data1 = 0;
			mMessage.data2 = 0;
			mMessage.valid = true;

			// Do not reset all input attributes, Running Status must remain unchanged.
			// We still need to reset these
			mPendingMessageIndex = 0;
			mPendingMessageExpectedLenght = 0;

			return true;
			break;

			// 2 bytes messages
		case ProgramChange:
		case AfterTouchChannel:
		case TimeCodeQuarterFrame:
		case SongSelect:
			mPendingMessageExpectedLenght = 2;
			break;

			// 3 bytes messages
		case NoteOn:
		case NoteOff:
		case ControlChange:
		case PitchBend:
		case AfterTouchPoly:
		case SongPosition:
			mPendingMessageExpectedLenght = 3;
			break;

		case SystemExclusive:
			// The message can be any lenght
			// between 3 and Settings.SysExMaxSize bytes
			mPendingMessageExpectedLenght = Settings.SysExMaxSize;
			mRunningStatus_RX = InvalidType;
			mMessage.sysexArray[0] = SystemExclusive;
			break;

		case InvalidType:
		default:
			// This is obviously wrong. Let's get the hell out'a here.
			resetInput();
			return false;
			break;
		}

		if (mPendingMessageIndex >= (mPendingMessageExpectedLenght - 1)) {
			// Reception complete
			mMessage.type = getTypeFromStatusByte(mPendingMessage[0]);
			mMessage.channel = getChannelFromStatusByte(mPendingMessage[0]);
			mMessage.data1 = mPendingMessage[1];
			mMessage.data2 = 0;			// Completed new message has 1 data byte

			mPendingMessageIndex = 0;
			mPendingMessageExpectedLenght = 0;
			mMessage.valid = true;
			return true;
		} else {
			// Waiting for more data
			mPendingMessageIndex++;
		}

		if (Settings.Use1ByteParsing) {
			// Message is not complete.
			return false;
		} else {
			// Call the parser recursively
			// to parse the rest of the message.
			return parse();
		}
	} else {
// First, test if this is a status byte
		if (extracted >= 0x80) {
			// Reception of status bytes in the middle of an uncompleted message
			// are allowed only for interleaved Real Time message or EOX
			switch (extracted) {
			case Clock:
			case Start:
			case Continue:
			case Stop:
			case ActiveSensing:
			case SystemReset:

// Here we will have to extract the one-byte message,
// pass it to the structure for being read outside
// the MIDI class, and recompose the message it was
// interleaved into. Oh, and without killing the running status..
// This is done by leaving the pending message as is,
// it will be completed on next calls.

				mMessage.type = (MidiType) extracted;
				mMessage.data1 = 0;
				mMessage.data2 = 0;
				mMessage.channel = 0;
				mMessage.valid = true;
				return true;

// End of Exclusive
			case 0xf7:
				if (mMessage.sysexArray[0] == SystemExclusive) {
					// Store the last byte (EOX)
					mMessage.sysexArray[mPendingMessageIndex++] = 0xf7;
					mMessage.type = SystemExclusive;

					// Get length
					mMessage.data1 = mPendingMessageIndex & 0xff;		// LSB
					mMessage.data2 = mPendingMessageIndex >> 8;			// MSB
					mMessage.channel = 0;
					mMessage.valid = true;

					resetInput();
					return true;
				} else {
					// Well well well.. error.
					resetInput();
					return false;
				}

			default:
				break; // LCOV_EXCL_LINE - Coverage blind spot
			}
		}

// Add extracted data byte to pending message
		if (mPendingMessage[0] == SystemExclusive)
			mMessage.sysexArray[mPendingMessageIndex] = extracted;
		else
			mPendingMessage[mPendingMessageIndex] = extracted;

// Now we are going to check if we have reached the end of the message
		if (mPendingMessageIndex >= (mPendingMessageExpectedLenght - 1)) {
			// "FML" case: fall down here with an overflown SysEx..
			// This means we received the last possible data byte that can fit
			// the buffer. If this happens, try increasing Settings.SysExMaxSize.
			if (mPendingMessage[0] == SystemExclusive) {
				resetInput();
				return false;
			}

			mMessage.type = getTypeFromStatusByte(mPendingMessage[0]);

			if (isChannelMessage(mMessage.type))
				mMessage.channel = getChannelFromStatusByte(mPendingMessage[0]);
			else
				mMessage.channel = 0;

			mMessage.data1 = mPendingMessage[1];

			// Save data2 only if applicable
			mMessage.data2 =
					mPendingMessageExpectedLenght == 3 ? mPendingMessage[2] : 0;

			// Reset local variables
			mPendingMessageIndex = 0;
			mPendingMessageExpectedLenght = 0;

			mMessage.valid = true;

			// Activate running status (if enabled for the received type)
			switch (mMessage.type) {
			case NoteOff:
			case NoteOn:
			case AfterTouchPoly:
			case ControlChange:
			case ProgramChange:
			case AfterTouchChannel:
			case PitchBend:
// Running status enabled: store it from received message
				mRunningStatus_RX = mPendingMessage[0];
				break;

			default:
// No running status
				mRunningStatus_RX = InvalidType;
				break;
			}
			return true;
		} else {
			// Then update the index of the pending message.
			mPendingMessageIndex++;

			if (Settings.Use1ByteParsing) {
// Message is not complete.
				return false;
			} else {
// Call the parser recursively to parse the rest of the message.
				return parse();
			}
		}
	}
}

void MidiInterface::handleNullVelocityNoteOnAsNoteOff() {
	if (Settings.HandleNullVelocityNoteOnAsNoteOff && getType() == NoteOn
			&& getData2() == 0) {
		mMessage.type = NoteOff;
	}
}

// Private method: check if the received message is on the listened channel

bool MidiInterface::inputFilter(Channel inChannel) {
	// This method handles recognition of channel
	// (to know if the message is destinated to the Arduino)

	// First, check if the received message is Channel
	if (mMessage.type >= NoteOff && mMessage.type <= PitchBend) {
// Then we need to know if we listen to it
		if ((mMessage.channel == inChannel)
				|| (inChannel == MIDI_CHANNEL_OMNI)) {
			return true;
		} else {
			// We don't listen to this channel
			return false;
		}
	} else {
// System messages are always received
		return true;
	}
}

// Private method: reset input attributes

void MidiInterface::resetInput() {
	mPendingMessageIndex = 0;
	mPendingMessageExpectedLenght = 0;
	mRunningStatus_RX = InvalidType;
}

// -----------------------------------------------------------------------------

/*! \brief Get the last received message's type

 Returns an enumerated type. @see MidiType
 */

MidiType MidiInterface::getType() {
	return mMessage.type;
}

/*! \brief Get the channel of the message stored in the structure.

 \return Channel range is 1 to 16.
 For non-channel messages, this will return 0.
 */

Channel MidiInterface::getChannel() {
	return mMessage.channel;
}

/*! \brief Get the first data byte of the last received message. */

DataByte MidiInterface::getData1() {
	return mMessage.data1;
}

/*! \brief Get the second data byte of the last received message. */

DataByte MidiInterface::getData2() {
	return mMessage.data2;
}

/*! \brief Get the System Exclusive byte array.

 @see getSysExArrayLength to get the array's length in bytes.
 */

byte* MidiInterface::getSysExArray() {
	return mMessage.sysexArray;
}

/*! \brief Get the lenght of the System Exclusive array.

 It is coded using data1 as LSB and data2 as MSB.
 \return The array's length, in bytes.
 */

unsigned MidiInterface::getSysExArrayLength() {
	return mMessage.getSysExSize();
}

/*! \brief Check if a valid message is stored in the structure. */

bool MidiInterface::check() {
	return mMessage.valid;
}

// -----------------------------------------------------------------------------

Channel MidiInterface::getInputChannel() {
	return mInputChannel;
}

/*! \brief Set the value for the input MIDI channel
 \param inChannel the channel value. Valid values are 1 to 16, MIDI_CHANNEL_OMNI
 if you want to listen to all channels, and MIDI_CHANNEL_OFF to disable input.
 */

void MidiInterface::setInputChannel(Channel inChannel) {
	mInputChannel = inChannel;
}

// -----------------------------------------------------------------------------

/*! \brief Extract an enumerated MIDI type from a status byte.

 This is a utility static method, used internally,
 made public so you can handle MidiTypes more easily.
 */

MidiType MidiInterface::getTypeFromStatusByte(
byte inStatus) {
	if ((inStatus < 0x80) || (inStatus == 0xf4) || (inStatus == 0xf5)
			|| (inStatus == 0xf9) || (inStatus == 0xfD)) {
// Data bytes and undefined.
		return InvalidType;
	}
	if (inStatus < 0xf0) {
// Channel message, remove channel nibble.
		return MidiType(inStatus & 0xf0);
	}

	return MidiType(inStatus);
}

/*! \brief Returns channel in the range 1-16
 */

Channel MidiInterface::getChannelFromStatusByte(
byte inStatus) {
	return (inStatus & 0x0f) + 1;
}

bool MidiInterface::isChannelMessage(MidiType inType) {
	return (inType == NoteOff || inType == NoteOn || inType == ControlChange
			|| inType == AfterTouchPoly || inType == AfterTouchChannel
			|| inType == PitchBend || inType == ProgramChange);
}

void MidiInterface::setHandleNoteOff(
		void (*fptr)(byte channel, byte note, byte velocity)) {
	mNoteOffCallback = fptr;
}
void MidiInterface::setHandleNoteOn(
		void (*fptr)(byte channel, byte note, byte velocity)) {
	mNoteOnCallback = fptr;
}
void MidiInterface::setHandleAfterTouchPoly(
		void (*fptr)(byte channel, byte note, byte pressure)) {
	mAfterTouchPolyCallback = fptr;
}
void MidiInterface::setHandleControlChange(
		void (*fptr)(byte channel, byte number, byte value)) {
	mControlChangeCallback = fptr;
}
void MidiInterface::setHandleProgramChange(
		void (*fptr)(byte channel, byte number)) {
	mProgramChangeCallback = fptr;
}
void MidiInterface::setHandleAfterTouchChannel(
		void (*fptr)(byte channel, byte pressure)) {
	mAfterTouchChannelCallback = fptr;
}
void MidiInterface::setHandlePitchBend(void (*fptr)(byte channel, int bend)) {
	mPitchBendCallback = fptr;
}
void MidiInterface::setHandleSystemExclusive(
		void (*fptr)(byte *array, unsigned size)) {
	mSystemExclusiveCallback = fptr;
}
void MidiInterface::setHandleTimeCodeQuarterFrame(void (*fptr)(byte data)) {
	mTimeCodeQuarterFrameCallback = fptr;
}
void MidiInterface::setHandleSongPosition(void (*fptr)(unsigned beats)) {
	mSongPositionCallback = fptr;
}
void MidiInterface::setHandleSongSelect(void (*fptr)(byte songnumber)) {
	mSongSelectCallback = fptr;
}
void MidiInterface::setHandleTuneRequest(void (*fptr)(void)) {
	mTuneRequestCallback = fptr;
}
void MidiInterface::setHandleClock(void (*fptr)(void)) {
	mClockCallback = fptr;
}
void MidiInterface::setHandleStart(void (*fptr)(void)) {
	mStartCallback = fptr;
}
void MidiInterface::setHandleContinue(void (*fptr)(void)) {
	mContinueCallback = fptr;
}
void MidiInterface::setHandleStop(void (*fptr)(void)) {
	mStopCallback = fptr;
}
void MidiInterface::setHandleActiveSensing(void (*fptr)(void)) {
	mActiveSensingCallback = fptr;
}
void MidiInterface::setHandleSystemReset(void (*fptr)(void)) {
	mSystemResetCallback = fptr;
}

/*! \brief Detach an external function from the given type.

 Use this method to cancel the effects of setHandle********.
 \param inType        The type of message to unbind.
 When a message of this type is received, no function will be called.
 */
void MidiInterface::disconnectCallbackFromType(MidiType inType) {
	switch (inType) {
	case NoteOff:
		mNoteOffCallback = 0;
		break;
	case NoteOn:
		mNoteOnCallback = 0;
		break;
	case AfterTouchPoly:
		mAfterTouchPolyCallback = 0;
		break;
	case ControlChange:
		mControlChangeCallback = 0;
		break;
	case ProgramChange:
		mProgramChangeCallback = 0;
		break;
	case AfterTouchChannel:
		mAfterTouchChannelCallback = 0;
		break;
	case PitchBend:
		mPitchBendCallback = 0;
		break;
	case SystemExclusive:
		mSystemExclusiveCallback = 0;
		break;
	case TimeCodeQuarterFrame:
		mTimeCodeQuarterFrameCallback = 0;
		break;
	case SongPosition:
		mSongPositionCallback = 0;
		break;
	case SongSelect:
		mSongSelectCallback = 0;
		break;
	case TuneRequest:
		mTuneRequestCallback = 0;
		break;
	case Clock:
		mClockCallback = 0;
		break;
	case Start:
		mStartCallback = 0;
		break;
	case Continue:
		mContinueCallback = 0;
		break;
	case Stop:
		mStopCallback = 0;
		break;
	case ActiveSensing:
		mActiveSensingCallback = 0;
		break;
	case SystemReset:
		mSystemResetCallback = 0;
		break;
	default:
		break;
	}
}

/*! @} */ // End of doc group MIDI Callbacks
// Private - launch callback function based on received type.
void MidiInterface::launchCallback() {
	// The order is mixed to allow frequent messages to trigger their callback faster.
	switch (mMessage.type) {
	// Notes
	case NoteOff:
		if (mNoteOffCallback != 0)
			mNoteOffCallback(mMessage.channel, mMessage.data1, mMessage.data2);
		break;
	case NoteOn:
		if (mNoteOnCallback != 0)
			mNoteOnCallback(mMessage.channel, mMessage.data1, mMessage.data2);
		break;

// Real-time messages
	case Clock:
		if (mClockCallback != 0)
			mClockCallback();
		break;
	case Start:
		if (mStartCallback != 0)
			mStartCallback();
		break;
	case Continue:
		if (mContinueCallback != 0)
			mContinueCallback();
		break;
	case Stop:
		if (mStopCallback != 0)
			mStopCallback();
		break;
	case ActiveSensing:
		if (mActiveSensingCallback != 0)
			mActiveSensingCallback();
		break;

// Continuous controllers
	case ControlChange:
		if (mControlChangeCallback != 0)
			mControlChangeCallback(mMessage.channel, mMessage.data1,
					mMessage.data2);
		break;
	case PitchBend:
		if (mPitchBendCallback != 0)
			mPitchBendCallback(mMessage.channel,
					(int) ((mMessage.data1 & 0x7f)
							| ((mMessage.data2 & 0x7f) << 7))
							+ MIDI_PITCHBEND_MIN);
		break;
	case AfterTouchPoly:
		if (mAfterTouchPolyCallback != 0)
			mAfterTouchPolyCallback(mMessage.channel, mMessage.data1,
					mMessage.data2);
		break;
	case AfterTouchChannel:
		if (mAfterTouchChannelCallback != 0)
			mAfterTouchChannelCallback(mMessage.channel, mMessage.data1);
		break;

	case ProgramChange:
		if (mProgramChangeCallback != 0)
			mProgramChangeCallback(mMessage.channel, mMessage.data1);
		break;
	case SystemExclusive:
		if (mSystemExclusiveCallback != 0)
			mSystemExclusiveCallback(mMessage.sysexArray,
					mMessage.getSysExSize());
		break;

// Occasional messages
	case TimeCodeQuarterFrame:
		if (mTimeCodeQuarterFrameCallback != 0)
			mTimeCodeQuarterFrameCallback(mMessage.data1);
		break;
	case SongPosition:
		if (mSongPositionCallback != 0)
			mSongPositionCallback(
					(mMessage.data1 & 0x7f) | ((mMessage.data2 & 0x7f) << 7));
		break;
	case SongSelect:
		if (mSongSelectCallback != 0)
			mSongSelectCallback(mMessage.data1);
		break;
	case TuneRequest:
		if (mTuneRequestCallback != 0)
			mTuneRequestCallback();
		break;

	case SystemReset:
		if (mSystemResetCallback != 0)
			mSystemResetCallback();
		break;

	case InvalidType:
	default:
		break; // LCOV_EXCL_LINE - Unreacheable code, but prevents unhandled case warning.
	}
}

void MidiInterface::thruFilter(Channel inChannel) {
	// If the feature is disabled, don't do anything.
	if (!mThruActivated || (mThruFilterMode == Thru::Off))
		return;

	// First, check if the received message is Channel
	if (mMessage.type >= NoteOff && mMessage.type <= PitchBend) {
		bool filter_condition = ((mMessage.channel == inChannel)
				|| (inChannel == MIDI_CHANNEL_OMNI));

		// Now let's pass it to the output
		switch (mThruFilterMode) {
		case Thru::Full:
			send(mMessage.type, mMessage.data1, mMessage.data2,
					mMessage.channel);
			break;

		case Thru::SameChannel:
			if (filter_condition) {
				send(mMessage.type, mMessage.data1, mMessage.data2,
						mMessage.channel);
			}
			break;

		case Thru::DifferentChannel:
			if (!filter_condition) {
				send(mMessage.type, mMessage.data1, mMessage.data2,
						mMessage.channel);
			}
			break;

		default:
			break;
		}
	} else {
		// Send the message to the output
		switch (mMessage.type) {
		// Real Time and 1 byte
		case Clock:
		case Start:
		case Stop:
		case Continue:
		case ActiveSensing:
		case SystemReset:
		case TuneRequest:
			sendRealTime(mMessage.type);
			break;

		case SystemExclusive:
			// Send SysEx (0xf0 and 0xf7 are included in the buffer)
			sendSysEx(getSysExArrayLength(), getSysExArray(), true);
			break;

		case SongSelect:
			sendSongSelect(mMessage.data1);
			break;

		case SongPosition:
			sendSongPosition(mMessage.data1 | ((unsigned) mMessage.data2 << 7));
			break;

		case TimeCodeQuarterFrame:
			sendTimeCodeQuarterFrame(mMessage.data1, mMessage.data2);
			break;

		default:
			break; // LCOV_EXCL_LINE - Unreacheable code, but prevents unhandled case warning.
		}
	}
}
