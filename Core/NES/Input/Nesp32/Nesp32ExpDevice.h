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

	void Flush(uint64_t now);
	void Route(const uint8_t* data, size_t len, double at);
	void Drop();
	void PrintStats();

protected:
	void TxSend(const uint8_t* data, size_t len, double at) override;
	size_t TxWaiting() override;
	bool TxIdle() override;
	void TxReset() override;

public:
	Nesp32ExpDevice(Emulator* emu, uint8_t port, KeyMappingSet keyMappings);
	~Nesp32ExpDevice();

	void OnAfterSetState() override;
	uint8_t ReadRam(uint16_t addr) override;
	void WriteRam(uint16_t addr, uint8_t value) override;
};
