#pragma once
#include "pch.h"
#include <deque>
#include "NES/Input/Nesp32/Nesp32Device.h"

//An NESP32 on the expansion port, with a fast way back to the console: an
//experiment, not a board that exists. Everything is Nesp32Device's (the
//firmware, its hardware layer, the UART the console sends requests on,
//OUT0) but the line toward the console.
//
//The console asks for the fast way by holding OUT1 (bit 1 of $4016 writes)
//up while it waits for a reply. A reply the board has ready while OUT1 is
//up goes into a FIFO, and every read of $4017 takes the next nibble of it:
//
//  bit 0     1: a nibble is here (and is taken by this read); 0: none yet
//            (nothing is taken)
//  bits 1-4  the nibble: a byte's low nibble, then its high
//
//It is synchronous: the nibble is there as the read is made, and the next
//comes up as the read ends (the /OE2 rising edge, as a 4021 shifts). OUT1
//going down empties the FIFO and drops a reply not yet given, so what an
//exchange cut short left behind is not read by the next.
//
//With OUT1 down the board answers on its UART on D0, as the controller-port
//NESP32 does, so one board serves a ROM that uses either.
//
//TURBO (CAM's op $04): the firmware queues a burst of bytes with the hal's
//fast_send, apart from the replies. While OUT2 (bit 2 of $4016 writes) is
//up, every read of $4017 is the burst's next byte, all eight bits: the
//board drives /D0-/D4 (inverted by the '368s, so it drives each low for a
//1) and D5-D7 straight onto the CPU's data bus while /OE2 is low, which on
//the NES-001 nothing else drives then (they are open bus). So it needs
//port 2 empty, and a NES-001: a Famicom's port has no data bus. A read with
//nothing there yet drives nothing. The burst stays until the next one
//takes its place; OUT2 going down does not drop it.
//
//What the console sees is written here; a real board drives the lines
//inverted, through the '368s. NESP32_EXP_STATS=1 prints how fast the
//console took the bytes, to stderr.
class Nesp32ExpDevice : public Nesp32Device
{
private:
	//The board's front end (a PIO or CPLD with the FIFO) has the reply from
	//the ESP32 over a fast serial link: a byte about every 2 us
	static constexpr double FeedCycles = 4;

	struct Nibble
	{
		double Ready;
		uint8_t Value;
		bool Last; //the last of a reply
	};

	struct Pending
	{
		vector<uint8_t> Data;
		double At;
	};

	std::deque<Nibble> _fifo;
	std::deque<Pending> _pending;
	bool _out1 = false;
	bool _out2 = false;

	//TURBO's burst, and the next byte of it; byte i is there from
	//_burstStart + i x FeedCycles
	vector<uint8_t> _burst;
	size_t _burstAt = 0;
	double _burstStart = 0;
	uint8_t _driven = 0; //the bits the last read drove above D4

	//How fast the console takes them
	bool _stats = false;
	bool _inReply = false;
	double _replyFirst = 0;
	uint32_t _replyNibbles = 0;
	uint64_t _statReplies = 0;
	uint64_t _statBytes = 0;
	double _statCycles = 0;
	double _statWait = 0;
	double _replyReady = 0;
	uint32_t _statFrames = 0;
	uint64_t _statBurstBytes = 0;
	double _statBurstCycles = 0;
	double _statFirstByte = -1;
	double _statLastByte = -1;

	void Flush(uint64_t now);
	void Route(const uint8_t* data, size_t len, double at);
	void Drop();
	void PrintStats();
	uint8_t ReadByte(double now);

protected:
	void TxSend(const uint8_t* data, size_t len, double at) override;
	size_t TxWaiting() override;
	bool TxIdle() override;
	void TxReset() override;
	bool FastSend(const uint8_t* data, size_t len, bool first) override;

public:
	Nesp32ExpDevice(Emulator* emu, uint8_t port, KeyMappingSet keyMappings);
	~Nesp32ExpDevice();

	void OnAfterSetState() override;
	uint8_t ReadRam(uint16_t addr) override;
	void WriteRam(uint16_t addr, uint8_t value) override;

	//Bits of the last read that the board drove itself, where the console
	//would otherwise have open bus (D5-D7 in a TURBO read)
	uint8_t GetDrivenBits() const { return _driven; }
};
