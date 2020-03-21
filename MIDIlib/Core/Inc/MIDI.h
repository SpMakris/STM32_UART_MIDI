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
#ifndef SRC_MIDI_H_
#define SRC_MIDI_H_

#include "stm32f0xx_hal.h"
#define byte uint8_t


// -----------------------------------------------------------------------------

#define MIDI_CHANNEL_OMNI       0
#define MIDI_CHANNEL_OFF        17 // and over
#define MidiTimeout 10
#define MIDI_PITCHBEND_MIN      -8192
#define MIDI_PITCHBEND_MAX      8191

// -----------------------------------------------------------------------------
// Type definitions

typedef byte StatusByte;
typedef byte DataByte;
typedef byte Channel;
typedef byte FilterMode;

// -----------------------------------------------------------------------------

/*! Enumeration of MIDI types */
enum MidiType {
	InvalidType = 0x00,    ///< For notifying errors
	NoteOff = 0x80,    ///< Note Off
	NoteOn = 0x90,    ///< Note On
	AfterTouchPoly = 0xA0,    ///< Polyphonic AfterTouch
	ControlChange = 0xB0,    ///< Control Change / Channel Mode
	ProgramChange = 0xC0,    ///< Program Change
	AfterTouchChannel = 0xD0,    ///< Channel (monophonic) AfterTouch
	PitchBend = 0xE0,    ///< Pitch Bend
	SystemExclusive = 0xF0,    ///< System Exclusive
	TimeCodeQuarterFrame = 0xF1, ///< System Common - MIDI Time Code Quarter Frame
	SongPosition = 0xF2,    ///< System Common - Song Position Pointer
	SongSelect = 0xF3,    ///< System Common - Song Select
	TuneRequest = 0xF6,    ///< System Common - Tune Request
	Clock = 0xF8,    ///< System Real Time - Timing Clock
	Start = 0xFA,    ///< System Real Time - Start
	Continue = 0xFB,    ///< System Real Time - Continue
	Stop = 0xFC,    ///< System Real Time - Stop
	ActiveSensing = 0xFE,    ///< System Real Time - Active Sensing
	SystemReset = 0xFF,    ///< System Real Time - System Reset
};

// -----------------------------------------------------------------------------

/*! Enumeration of Thru filter modes */
struct Thru {
	enum Mode {
		Off = 0,  ///< Thru disabled (nothing passes through).
		Full = 1, ///< Fully enabled Thru (every incoming message is sent back).
		SameChannel = 2, ///< Only the messages on the Input Channel will be sent back.
		DifferentChannel = 3, ///< All the messages but the ones on the Input Channel will be sent back.
	};
};

// -----------------------------------------------------------------------------

/*! \brief Enumeration of Control Change command numbers.
 See the detailed controllers numbers & description here:
 http://www.somascape.org/midi/tech/spec.html#ctrlnums
 */
enum MidiControlChangeNumber {
	// High resolution Continuous Controllers MSB (+32 for LSB) ----------------
	BankSelect = 0,
	ModulationWheel = 1,
	BreathController = 2,
	// CC3 undefined
	FootController = 4,
	PortamentoTime = 5,
	DataEntryMSB = 6,
	ChannelVolume = 7,
	Balance = 8,
	// CC9 undefined
	Pan = 10,
	ExpressionController = 11,
	EffectControl1 = 12,
	EffectControl2 = 13,
	// CC14 undefined
	// CC15 undefined
	GeneralPurposeController1 = 16,
	GeneralPurposeController2 = 17,
	GeneralPurposeController3 = 18,
	GeneralPurposeController4 = 19,

	DataEntryLSB = 38,

	// Switches ----------------------------------------------------------------
	Sustain = 64,
	Portamento = 65,
	Sostenuto = 66,
	SoftPedal = 67,
	Legato = 68,
	Hold = 69,

	// Low resolution continuous controllers -----------------------------------
	SoundController1 = 70,   ///< Synth: Sound Variation   FX: Exciter On/Off
	SoundController2 = 71,   ///< Synth: Harmonic Content  FX: Compressor On/Off
	SoundController3 = 72,   ///< Synth: Release Time      FX: Distortion On/Off
	SoundController4 = 73,   ///< Synth: Attack Time       FX: EQ On/Off
	SoundController5 = 74,   ///< Synth: Brightness        FX: Expander On/Off
	SoundController6 = 75,   ///< Synth: Decay Time        FX: Reverb On/Off
	SoundController7 = 76,   ///< Synth: Vibrato Rate      FX: Delay On/Off
	SoundController8 = 77, ///< Synth: Vibrato Depth     FX: Pitch Transpose On/Off
	SoundController9 = 78, ///< Synth: Vibrato Delay     FX: Flange/Chorus On/Off
	SoundController10 = 79, ///< Synth: Undefined         FX: Special Effects On/Off
	GeneralPurposeController5 = 80,
	GeneralPurposeController6 = 81,
	GeneralPurposeController7 = 82,
	GeneralPurposeController8 = 83,
	PortamentoControl = 84,
	// CC85 to CC90 undefined
	Effects1 = 91,   ///< Reverb send level
	Effects2 = 92,   ///< Tremolo depth
	Effects3 = 93,   ///< Chorus send level
	Effects4 = 94,   ///< Celeste depth
	Effects5 = 95,   ///< Phaser depth
	DataIncrement = 96,
	DataDecrement = 97,
	NRPNLSB = 98,   ///< Non-Registered Parameter Number (LSB)
	NRPNMSB = 99,   ///< Non-Registered Parameter Number (MSB)
	RPNLSB = 100,  ///< Registered Parameter Number (LSB)
	RPNMSB = 101,  ///< Registered Parameter Number (MSB)

	// Channel Mode messages ---------------------------------------------------
	AllSoundOff = 120,
	ResetAllControllers = 121,
	LocalControl = 122,
	AllNotesOff = 123,
	OmniModeOff = 124,
	OmniModeOn = 125,
	MonoModeOn = 126,
	PolyModeOn = 127
};

struct RPN {
	enum RegisteredParameterNumbers {
		PitchBendSensitivity = 0x0000,
		ChannelFineTuning = 0x0001,
		ChannelCoarseTuning = 0x0002,
		SelectTuningProgram = 0x0003,
		SelectTuningBank = 0x0004,
		ModulationDepthRange = 0x0005,
		NullFunction = (0x7f << 7) + 0x7f,
	};
};

// -------
struct mMessage {
	bool valid = false;
	MidiType type = InvalidType;
	Channel channel = 0;
	uint8_t data1 = 0;
	uint8_t data2 = 0;

	uint8_t SysExMaxSize;
	 unsigned getSysExSize()  {
		 unsigned size = unsigned(data2) << 8 | data1;
		return size > SysExMaxSize ? SysExMaxSize : size;
	}
	uint8_t sysexArray[];
};

struct mSettings {
	/*! Running status enables short messages when sending multiple values
	 of the same type and channel.\n
	 Warning: does not work with some hardware, enable with caution.
	 */
	bool UseRunningStatus = false;

	/*! NoteOn with 0 velocity should be handled as NoteOf.\n
	 Set to true  to get NoteOff events when receiving null-velocity NoteOn messages.\n
	 Set to false to get NoteOn  events when receiving null-velocity NoteOn messages.
	 */
	bool HandleNullVelocityNoteOnAsNoteOff = true;

	/*! Setting this to true will make MIDI.read parse only one byte of data for each
	 call when data is available. This can speed up your application if receiving
	 a lot of traffic, but might induce MIDI Thru and treatment latency.
	 */
	bool Use1ByteParsing = true;

	/*! Maximum size of SysEx receivable. Decrease to save RAM if you don't expect
	 to receive SysEx, or adjust accordingly.
	 */
	static const  unsigned SysExMaxSize = 128;
};
/*! \brief The main class for MIDI handling.
 It is templated over the type of serial port to provide abstraction from
 the hardware interface, meaning you can use HardwareSerial, SoftwareSerial
 or ak47's Uart classes. The only requirement is that the class implements
 the begin, read, write and available methods.
 */

class MidiInterface {

public:
	 MidiInterface();
	~MidiInterface();

public:
	void begin(Channel inChannel, UART_HandleTypeDef *huart_in,
			UART_HandleTypeDef *huart_out);

	// -------------------------------------------------------------------------
	// MIDI Output

public:
	 void sendNoteOn(DataByte inNoteNumber, DataByte inVelocity,
			Channel inChannel);

	 void sendNoteOff(DataByte inNoteNumber, DataByte inVelocity,
			Channel inChannel);

	 void sendProgramChange(DataByte inProgramNumber, Channel inChannel);

	 void sendControlChange(DataByte inControlNumber,
			DataByte inControlValue, Channel inChannel);

	 void sendPitchBend(int inPitchValue, Channel inChannel);
	 void sendPitchBend(double inPitchValue, Channel inChannel);

	 void sendPolyPressure(DataByte inNoteNumber, DataByte inPressure,
			Channel inChannel) __attribute__ ((deprecated));

	 void sendAfterTouch(DataByte inPressure, Channel inChannel);
	 void sendAfterTouch(DataByte inNoteNumber, DataByte inPressure,
			Channel inChannel);

	 void sendSysEx(uint8_t inLength, byte *inArray,
				bool inArrayContainsBoundaries);

	 void sendTimeCodeQuarterFrame(DataByte inTypeNibble,
			DataByte inValuesNibble);
	 void sendTimeCodeQuarterFrame(DataByte inData);

	 void sendSongPosition(unsigned inBeats);
	 void sendSongSelect(DataByte inSongNumber);
	 void sendTuneRequest();
	 void sendRealTime(MidiType inType);

	 void beginRpn(unsigned inNumber, Channel inChannel);
	 void sendRpnValue(unsigned inValue, Channel inChannel);
	 void sendRpnValue(byte inMsb,
	byte inLsb, Channel inChannel);
	 void sendRpnIncrement(byte inAmount, Channel inChannel);
	 void sendRpnDecrement(byte inAmount, Channel inChannel);
	 void endRpn(Channel inChannel);

	 void beginNrpn(unsigned inNumber, Channel inChannel);
	 void sendNrpnValue(unsigned inValue, Channel inChannel);
	 void sendNrpnValue(byte inMsb,
	byte inLsb, Channel inChannel);
	 void sendNrpnIncrement(byte inAmount, Channel inChannel);
	 void sendNrpnDecrement(byte inAmount, Channel inChannel);
	 void endNrpn(Channel inChannel);

public:
	void send(MidiType inType, DataByte inData1, DataByte inData2,
			Channel inChannel);

	// -------------------------------------------------------------------------
	// MIDI Input

public:
	 bool read();
	 bool read(Channel inChannel);

public:
	 MidiType getType() ;
	 Channel getChannel() ;
	 DataByte getData1() ;
	 DataByte getData2() ;
	  byte* getSysExArray() ;
	 unsigned getSysExArrayLength() ;
	 bool check() ;

public:
	 Channel getInputChannel() ;
	 void setInputChannel(Channel inChannel);

public:
	static  MidiType getTypeFromStatusByte(byte inStatus);
	static  Channel getChannelFromStatusByte(byte inStatus);
	static  bool isChannelMessage(MidiType inType);

	// -------------------------------------------------------------------------
	// Input Callbacks

public:
	 void setHandleNoteOff(
			void (*fptr)(byte channel, byte note, byte velocity));
	 void setHandleNoteOn(
			void (*fptr)(byte channel, byte note, byte velocity));
	 void setHandleAfterTouchPoly(
			void (*fptr)(byte channel, byte note, byte pressure));
	 void setHandleControlChange(
			void (*fptr)(byte channel, byte number, byte value));
	 void setHandleProgramChange(void (*fptr)(byte channel, byte number));
	 void setHandleAfterTouchChannel(
			void (*fptr)(byte channel, byte pressure));
	 void setHandlePitchBend(void (*fptr)(byte channel, int bend));
	 void setHandleSystemExclusive(
			void (*fptr)(byte *array, unsigned size));
	 void setHandleTimeCodeQuarterFrame(void (*fptr)(byte data));
	 void setHandleSongPosition(void (*fptr)(unsigned beats));
	 void setHandleSongSelect(void (*fptr)(byte songnumber));
	 void setHandleTuneRequest(void (*fptr)(void));
	 void setHandleClock(void (*fptr)(void));
	 void setHandleStart(void (*fptr)(void));
	 void setHandleContinue(void (*fptr)(void));
	 void setHandleStop(void (*fptr)(void));
	 void setHandleActiveSensing(void (*fptr)(void));
	 void setHandleSystemReset(void (*fptr)(void));

	 void disconnectCallbackFromType(MidiType inType);

private:
	void launchCallback();

	void (*mNoteOffCallback)(byte channel, byte note, byte velocity);
	void (*mNoteOnCallback)(byte channel, byte note, byte velocity);
	void (*mAfterTouchPolyCallback)(byte channel, byte note, byte velocity);
	void (*mControlChangeCallback)(byte channel, byte, byte);
	void (*mProgramChangeCallback)(byte channel, byte);
	void (*mAfterTouchChannelCallback)(byte channel, byte);
	void (*mPitchBendCallback)(byte channel, int);
	void (*mSystemExclusiveCallback)(byte *array, unsigned size);
	void (*mTimeCodeQuarterFrameCallback)(byte data);
	void (*mSongPositionCallback)(unsigned beats);
	void (*mSongSelectCallback)(byte songnumber);
	void (*mTuneRequestCallback)(void);
	void (*mClockCallback)(void);
	void (*mStartCallback)(void);
	void (*mContinueCallback)(void);
	void (*mStopCallback)(void);
	void (*mActiveSensingCallback)(void);
	void (*mSystemResetCallback)(void);

	// -------------------------------------------------------------------------
	// MIDI Soft Thru

public:
	 Thru::Mode getFilterMode() ;
	 bool getThruState() ;

	 void turnThruOn(Thru::Mode inThruFilterMode = Thru::Off);
	 void turnThruOff();
	 void setThruFilterMode(Thru::Mode inThruFilterMode);

private:
	void thruFilter(byte inChannel);

private:
	bool parse();
	bool parse2();
	 void handleNullVelocityNoteOnAsNoteOff();
	 bool inputFilter(Channel inChannel);
	 void resetInput();

private:
//	typedef Message<Settings::SysExMaxSize> MidiMessage;

private:
	UART_HandleTypeDef serial_out;
	UART_HandleTypeDef serial_in;

private:
	uint8_t mInputChannel;
	uint8_t mRunningStatus_RX;
	uint8_t mRunningStatus_TX;
	uint8_t mPendingMessage[3];
	uint8_t mPendingMessageExpectedLenght;
	uint8_t mPendingMessageIndex;
	uint16_t mCurrentRpnNumber;
	uint16_t mCurrentNrpnNumber;
	bool mThruActivated :1;
	Thru::Mode mThruFilterMode :7;

	struct mSettings Settings;
private:
	 uint8_t getStatus(MidiType inType, Channel inChannel) ;
public:
	struct mMessage mMessage;
};

// -----------------------------------------------------------------------------

unsigned encodeSysEx( byte *inData, byte *outSysEx, unsigned inLenght);
unsigned decodeSysEx( byte *inSysEx, byte *outData, unsigned inLenght);

#endif /* SRC_MIDI_H_ */
