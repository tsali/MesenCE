#pragma once
#include "pch.h"
#include <mutex>
#include "Shared/BaseControlDevice.h"
#include "NES/Input/Nesp32/Nesp32Line.h"
#include "NES/Input/Nesp32/Nesp32Fetcher.h"

extern "C" {
#include "NES/Input/Nesp32/firmware/nesp_services.h"
}

//An NESP32 on controller port 2: the Wi-Fi interface for the NES from
//https://github.com/Armitage64/nesp32 (an ESP32 board on the controller
//port, talking to the console over a cycle-timed serial line).
//
//The console's side is the real thing: its ROMs bit-bang a UART through
//$4016 bit 0 (OUT0, console -> NESP32) and $4017 bit 0 (D0, NESP32 ->
//console), and Mesen runs them cycle for cycle. The NESP32's side is the
//board's own firmware (its link and services, in firmware/), run here with
//a hardware layer of Mesen's: the line model for its UART, real HTTP(S)
//fetches, real time, and a Wi-Fi network that is always there.
class Nesp32Device : public BaseControlDevice
{
public:
	enum Buttons { A = 0, B };

protected:
	static constexpr double Turnaround = 179; //~0.1 ms from a request's last byte to the reply
	static constexpr size_t TxBuffer = 2048;  //the board's UART transmit buffer
	static constexpr uint64_t BusyTickCycles = NESP_CPU_HZ * 2 / 1000;
	static constexpr uint64_t IdleTickCycles = NESP_CPU_HZ * 20 / 1000;

private:
	nesp_hal_t _hal = {};
	unique_ptr<nesp_stick_t> _stick;
	Nesp32Fetcher _fetcher;
	std::recursive_mutex _lock;

protected:
	Nesp32Line::Wire _wire = Nesp32Line::Wire(NESP_TO_NES_BITS);
	Nesp32Line::Tap _tap;
	double _bit = NESP_PERIOD;   //the UART's bit length, in CPU cycles, both ways
	uint64_t _now = 0;

private:
	double _searchFrom = 0;
	double _heardAt = 0;
	uint64_t _nextTick = 0;
	vector<std::pair<uint8_t, double>> _heard;

	//What the board keeps: Wi-Fi, and what the services ask it to keep
	nesp_wifi_t _wifi = {};
	uint8_t _scanState = NESP_SCAN_IDLE;
	string _app;
	bool _haveAirport = false;
	char _airport[4] = {};
	int32_t _airportPlace[2] = {};
	bool _haveZip = false;
	char _zip[NESP_WEATHER_ZIP_LEN] = {};
	int32_t _zipPlace[2] = {};
	char _zipName[NESP_WEATHER_PLACE_MAX] = {};
	bool _haveFavorites = false;
	nesp_bsky_favorites_t _favorites = {};
	uint8_t _noteIn[NESP_ARQ_NOTE] = {};
	vector<string> _tracks;
	bool _wasPressed[2] = {};

	void SetupHal();
	void LoadTracks();

	//The hardware layer (nesp_hal_t), as the board's firmware/main has it
	static uint8_t HalBoard(void* ctx);
	static const char* HalBoardName(void* ctx);
	static uint8_t HalBattery(void* ctx);
	static bool HalTime(void* ctx, uint32_t* seconds);
	static void HalWifi(void* ctx, nesp_wifi_t* out);
	static bool HalScanStart(void* ctx);
	static uint8_t HalScanState(void* ctx, uint8_t* count);
	static bool HalScanResult(void* ctx, uint8_t index, nesp_network_t* out);
	static bool HalJoin(void* ctx, const char* name, uint8_t nameLen, const char* password, uint8_t passwordLen);
	static void HalForget(void* ctx);
	static void HalApp(void* ctx, const char* id, uint8_t version);
	static bool HalFetch(void* ctx, const char* url, const char* agent, uint32_t id, const nesp_sink_t* sink);
	static void HalFetchStop(void* ctx);
	static void* HalAlloc(void* ctx, size_t bytes);
	static void HalRelease(void* ctx, void* memory);
	static void HalLock(void* ctx);
	static void HalUnlock(void* ctx);
	static void HalLineRate(void* ctx, uint8_t period);
	static size_t HalLineWrite(void* ctx, const uint8_t* data, size_t len);
	static bool HalLineIdle(void* ctx);
	static uint16_t HalTracks(void* ctx);
	static uint32_t HalTrackSize(void* ctx, uint16_t track);
	static size_t HalTrackRead(void* ctx, uint16_t track, uint32_t offset, uint8_t* out, size_t len);
	static void HalShow(void* ctx, uint8_t row, const char* text);
	static uint8_t HalPeerState(void* ctx);
	static bool HalPeerListen(void* ctx, uint16_t port);
	static bool HalPeerCall(void* ctx, const uint8_t* address, uint16_t port);
	static void HalPeerClose(void* ctx);
	static size_t HalPeerSend(void* ctx, const uint8_t* data, size_t len);
	static size_t HalPeerRecv(void* ctx, uint8_t* out, size_t max);
	static bool HalPeerLast(void* ctx, uint8_t* address);
	static void HalPeerKeep(void* ctx, const uint8_t* address);
	static uint32_t HalPeerRtt(void* ctx);
	static void HalPeerNote(void* ctx, const uint8_t* note);
	static void HalPeerNoted(void* ctx, uint8_t* note);
	static void HalPeerSpread(void* ctx, uint32_t* counts);
	static bool HalAirportLast(void* ctx, char* code, int32_t* latitude, int32_t* longitude);
	static void HalAirportKeep(void* ctx, const char* code, int32_t latitude, int32_t longitude);
	static bool HalZipLast(void* ctx, char* zip, int32_t* latitude, int32_t* longitude, char* name);
	static void HalZipKeep(void* ctx, const char* zip, int32_t latitude, int32_t longitude, const char* name);
	static bool HalFavoritesLast(void* ctx, nesp_bsky_favorites_t* out);
	static bool HalSetting(void* ctx, const char* name, char* out, size_t max);
	static void HalFavoritesKeep(void* ctx, const nesp_bsky_favorites_t* favorites);

protected:
	string GetKeyNames() override { return "AB"; }
	void InternalSetStateFromInput() override;

	//Brings the board up to a cycle (its link, its services' tick)
	void Advance(uint64_t now);

	//The board's side of the line toward the console. Here, its UART on
	//D0; Nesp32ExpDevice (the expansion-port variant) puts its own in.
	//Send: bytes from cycle "at" on (which may be ahead of _now).
	//Waiting: bytes not yet out, for the transmit buffer's room.
	//Idle: nothing left to go out. Reset: time went back.
	virtual void TxSend(const uint8_t* data, size_t len, double at);
	virtual size_t TxWaiting();
	virtual bool TxIdle();
	virtual void TxReset();

	Nesp32Device(Emulator* emu, ControllerType type, uint8_t port, KeyMappingSet keyMappings);

public:
	Nesp32Device(Emulator* emu, uint8_t port, KeyMappingSet keyMappings);
	~Nesp32Device();

	void OnAfterSetState() override;
	uint8_t ReadRam(uint16_t addr) override;
	void WriteRam(uint16_t addr, uint8_t value) override;
};
