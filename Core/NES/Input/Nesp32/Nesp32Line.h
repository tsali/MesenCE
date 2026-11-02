#pragma once
#include "pch.h"
#include <deque>
#include <cmath>

//The serial line between the console and an NESP32, in CPU cycles.
//A port of the NESP32 project's own line model (tests/wire.py).
//
//Wire: what the NESP32's UART sends, as the console sees it on D0.
//Tap: OUT0 (bit 0 of $4016 writes), as the NESP32's UART receives it.
//Mark (idle) is 1, space is 0. Frames are 8N1 from the console, and 8N2
//(NESP_TO_NES_BITS = 11 bits a byte) from the NESP32.
namespace Nesp32Line
{
	constexpr int Mark = 1;
	constexpr int Space = 0;

	class Wire
	{
	private:
		struct Item
		{
			double Start;
			double Bit;
			uint8_t Byte;
		};

		std::deque<Item> _items;
		double _busyUntil = 0;
		int _frameBits;

	public:
		Wire(int frameBits) : _frameBits(frameBits) {}

		void Reset()
		{
			_items.clear();
			_busyUntil = 0;
		}

		//Queue bytes back to back from cycle "at". Returns when they end.
		double Send(const uint8_t* data, size_t len, double at, double bit)
		{
			double start = std::max(at, _busyUntil);
			for(size_t i = 0; i < len; i++) {
				_items.push_back({ start, bit, data[i] });
				start += bit * _frameBits;
			}
			_busyUntil = start;
			return start;
		}

		//The level on the line at a cycle. Reads come in order of time, so
		//bytes that are over are dropped as the reads pass them.
		int Level(double cycle)
		{
			while(_items.size() >= 2 && _items[1].Start <= cycle) {
				_items.pop_front();
			}
			if(_items.empty() || _items[0].Start > cycle) {
				return Mark;
			}
			const Item& item = _items[0];
			double offset = (cycle - item.Start) / item.Bit;
			if(offset < 1) {
				return Space;
			}
			if(offset < 9) {
				return (item.Byte >> (int)(offset - 1)) & 1;
			}
			return Mark;
		}

		double BusyUntil() const { return _busyUntil; }

		//Bytes still to go out at a cycle, at this bit length.
		size_t Waiting(double now, double bit) const
		{
			double left = std::max(0.0, _busyUntil - now);
			return (size_t)std::ceil(left / (bit * _frameBits));
		}
	};

	class Tap
	{
	private:
		//(cycle, level) of every change. OUT0 is low (a break) until the ROM sets it.
		std::vector<std::pair<double, int>> _changes = { { -1e300, Space } };

		int LevelAt(double cycle) const
		{
			//The last change at or before the cycle
			size_t lo = 0, hi = _changes.size();
			while(hi - lo > 1) {
				size_t mid = (lo + hi) / 2;
				if(_changes[mid].first <= cycle) {
					lo = mid;
				} else {
					hi = mid;
				}
			}
			return _changes[lo].second;
		}

	public:
		void Reset()
		{
			_changes = { { -1e300, Space } };
		}

		void Write(double cycle, int level)
		{
			if(level != _changes.back().second) {
				_changes.push_back({ cycle, level });
			}
		}

		//Bytes a UART with this bit length receives between two cycles, with
		//the cycle each one's stop bit was sampled. searchFrom moves on.
		void Decode(double& searchFrom, double until, double bit, std::vector<std::pair<uint8_t, double>>& out)
		{
			while(true) {
				//The first change to space after searchFrom: a start bit
				size_t index = 0;
				while(index < _changes.size() && _changes[index].first <= searchFrom) {
					index++;
				}
				while(index < _changes.size() && _changes[index].second != Space) {
					index++;
				}
				if(index >= _changes.size()) {
					break;
				}
				double edge = _changes[index].first;
				if(edge + 9.5 * bit > until) {
					break;
				}
				if(LevelAt(edge + 0.5 * bit) != Space) {
					searchFrom = edge; //a glitch, not a start bit
					continue;
				}
				uint8_t value = 0;
				for(int k = 0; k < 8; k++) {
					value |= LevelAt(edge + (1.5 + k) * bit) << k;
				}
				double stop = edge + 9.5 * bit;
				if(LevelAt(stop) == Mark) {
					out.push_back({ value, stop });
				}
				searchFrom = stop;
			}

			//Keep only what a later decode can still need: the change in
			//force at searchFrom, and everything after it.
			size_t keep = 0;
			while(keep + 1 < _changes.size() && _changes[keep + 1].first <= searchFrom) {
				keep++;
			}
			if(keep > 0) {
				_changes.erase(_changes.begin(), _changes.begin() + keep);
			}
		}
	};
}
