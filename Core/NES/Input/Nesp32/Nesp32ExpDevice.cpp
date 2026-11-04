#include "pch.h"
#include <cstdio>
#include <cstdlib>
#include "NES/Input/Nesp32/Nesp32ExpDevice.h"
#include "Shared/Emulator.h"

Nesp32ExpDevice::Nesp32ExpDevice(Emulator* emu, uint8_t port, KeyMappingSet keyMappings)
	: Nesp32Device(emu, ControllerType::Nesp32Exp, port, keyMappings)
{
#ifdef _MSC_VER
	char* stats = nullptr;
	size_t statsLen = 0;
	if(_dupenv_s(&stats, &statsLen, "NESP32_EXP_STATS") != 0) {
		stats = nullptr;
	}
	_stats = stats && stats[0] && stats[0] != '0';
	free(stats);
#else
	const char* stats = getenv("NESP32_EXP_STATS");
	_stats = stats && stats[0] && stats[0] != '0';
#endif
}

Nesp32ExpDevice::~Nesp32ExpDevice()
{
	PrintStats();
}

void Nesp32ExpDevice::PrintStats()
{
	if(!_stats || (_statReplies == 0 && _statBurstBytes == 0)) {
		return;
	}
	if(_statBurstBytes > 0) {
		//Bytes read in runs (reads under 64 cycles apart): the read loop's
		//rate; and all of them over the time from the first to the last
		double inRuns = _statBurstCycles / NESP_CPU_HZ;
		double overall = (_statLastByte - _statFirstByte) / NESP_CPU_HZ;
		fprintf(stderr, "[NESP32 EXP] turbo: %llu bytes; read at %.0f bytes/s (%.1f cycles a byte) in runs, %.0f bytes/s over the whole time\n",
			(unsigned long long)_statBurstBytes,
			inRuns > 0 ? _statBurstBytes / inRuns : 0.0,
			_statBurstBytes ? _statBurstCycles / _statBurstBytes : 0.0,
			overall > 0 ? _statBurstBytes / overall : 0.0);
	}
	if(_statReplies == 0) {
		return;
	}
	//From each reply's first nibble taken to its last: the link's own rate
	double seconds = _statCycles / NESP_CPU_HZ;
	fprintf(stderr, "[NESP32 EXP] %llu replies, %llu bytes; taken at %.0f bytes/s (%.1f cycles a byte); %.0f cycles from ready to the first nibble, on average\n",
		(unsigned long long)_statReplies, (unsigned long long)_statBytes,
		seconds > 0 ? _statBytes / seconds : 0.0,
		_statBytes ? _statCycles / _statBytes : 0.0,
		_statWait / _statReplies);
}

void Nesp32ExpDevice::OnAfterSetState()
{
	Nesp32Device::OnAfterSetState();
	if(_stats && ++_statFrames % 600 == 0) {
		PrintStats();
	}
}

//A reply (or a stream's bytes) from the board: given when its time comes,
//by the way OUT1 says then
void Nesp32ExpDevice::TxSend(const uint8_t* data, size_t len, double at)
{
	if(at <= (double)_now) {
		Route(data, len, at);
	} else {
		_pending.push_back({ vector<uint8_t>(data, data + len), at });
	}
}

void Nesp32ExpDevice::Flush(uint64_t now)
{
	while(!_pending.empty() && _pending.front().At <= (double)now) {
		Pending item = std::move(_pending.front());
		_pending.pop_front();
		Route(item.Data.data(), item.Data.size(), item.At);
	}
}

void Nesp32ExpDevice::Route(const uint8_t* data, size_t len, double at)
{
	if(!_out1) {
		_wire.Send(data, len, at, _bit);
		return;
	}
	if(_stats && _fifo.empty()) {
		_replyReady = at;
	}
	for(size_t i = 0; i < len; i++) {
		double ready = at + i * FeedCycles;
		_fifo.push_back({ ready, (uint8_t)(data[i] & 0x0F), false });
		_fifo.push_back({ ready, (uint8_t)(data[i] >> 4), i == len - 1 });
	}
}

void Nesp32ExpDevice::Drop()
{
	_fifo.clear();
	_pending.clear();
	_inReply = false;
}

size_t Nesp32ExpDevice::TxWaiting()
{
	size_t waiting = _fifo.size() / 2 + Nesp32Device::TxWaiting();
	for(Pending& item : _pending) {
		waiting += item.Data.size();
	}
	return waiting;
}

bool Nesp32ExpDevice::TxIdle()
{
	return _fifo.empty() && _pending.empty() && Nesp32Device::TxIdle();
}

void Nesp32ExpDevice::TxReset()
{
	Drop();
	_burst.clear();
	_burstAt = 0;
	Nesp32Device::TxReset();
}

//TURBO's burst: queued apart from the replies, from a moment after the
//request (as a reply is), a byte every FeedCycles
bool Nesp32ExpDevice::FastSend(const uint8_t* data, size_t len, bool first)
{
	if(first) {
		_burst.clear();
		_burstAt = 0;
		_burstStart = (double)_now + Turnaround;
	}
	_burst.insert(_burst.end(), data, data + len);
	return true;
}

uint8_t Nesp32ExpDevice::ReadByte(double now)
{
	if(_burstAt >= _burst.size() || _burstStart + _burstAt * FeedCycles > now) {
		return 0; //nothing there yet: the board drives nothing
	}
	uint8_t value = _burst[_burstAt++];
	_driven = 0xE0;
	if(_stats) {
		if(_statLastByte >= 0 && now - _statLastByte < 64) {
			_statBurstCycles += now - _statLastByte;
		} else {
			_statBurstCycles += 8; //a run's first byte: a read's worth
		}
		if(_statFirstByte < 0) {
			_statFirstByte = now;
		}
		_statLastByte = now;
		_statBurstBytes++;
	}
	return value;
}

uint8_t Nesp32ExpDevice::ReadRam(uint16_t addr)
{
	_driven = 0;
	if(addr != 0x4017) {
		return 0;
	}
	uint64_t now = _emu->GetMasterClock();
	Advance(now);
	Flush(now);
	if(_out2) {
		return ReadByte((double)now);
	}
	//D0: the UART, as on the controller port (the '368 inverts it)
	uint8_t value = (_wire.Level((double)now) ^ 1) & 0x01;
	if(!_fifo.empty() && _fifo.front().Ready <= (double)now) {
		Nibble nibble = _fifo.front();
		_fifo.pop_front();
		value |= 0x01 | (nibble.Value << 1);
		if(_stats) {
			if(!_inReply) {
				_inReply = true;
				_replyFirst = (double)now;
				_replyNibbles = 0;
				_statWait += (double)now - _replyReady;
			}
			_replyNibbles++;
			if(nibble.Last) {
				//A byte for each two nibbles, from the first read to just
				//past the last (one byte's time is counted per byte)
				double perByte = _replyNibbles > 2 ? ((double)now - _replyFirst) / (_replyNibbles / 2 - 1) : 0;
				_statReplies++;
				_statBytes += _replyNibbles / 2;
				_statCycles += perByte * (_replyNibbles / 2);
				_inReply = false;
			}
		}
	}
	return value;
}

void Nesp32ExpDevice::WriteRam(uint16_t addr, uint8_t value)
{
	if(addr != 0x4016) {
		return;
	}
	uint64_t now = _emu->GetMasterClock();
	Advance(now);
	Flush(now);
	bool out1 = (value & 0x02) != 0;
	if(_out1 && !out1) {
		Drop();
	}
	_out1 = out1;
	_out2 = (value & 0x04) != 0;
	_tap.Write((double)now, value & 0x01);
}
