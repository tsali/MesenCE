#include "pch.h"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include "NES/Input/Nesp32/Nesp32Device.h"
#include "Shared/Emulator.h"
#include "Shared/MessageManager.h"
#include "Shared/KeyManager.h"
#include "Utilities/FolderUtilities.h"
#include "Utilities/VirtualFile.h"

static Nesp32Device* Self(void* ctx) { return (Nesp32Device*)ctx; }

static const char* NetworkName = "Mesen";

Nesp32Device::Nesp32Device(Emulator* emu, uint8_t port, KeyMappingSet keyMappings)
	: Nesp32Device(emu, ControllerType::Nesp32, port, keyMappings)
{
}

Nesp32Device::Nesp32Device(Emulator* emu, ControllerType type, uint8_t port, KeyMappingSet keyMappings)
	: BaseControlDevice(emu, type, port, keyMappings)
{
	//On the network from the start, as a board that has joined one before
	_wifi.state = NESP_WIFI_JOINED;
	_wifi.rssi = -45;
	_wifi.address[0] = 192;
	_wifi.address[1] = 168;
	_wifi.address[2] = 0;
	_wifi.address[3] = 32;
	_wifi.name_len = (uint8_t)strlen(NetworkName);
	memcpy(_wifi.name, NetworkName, _wifi.name_len);

	LoadTracks();
	SetupHal();
	_stick.reset(new nesp_stick_t());
	nesp_stick_init(_stick.get(), &_hal);
}

Nesp32Device::~Nesp32Device()
{
	_fetcher.Stop();
}

void Nesp32Device::SetupHal()
{
	_hal.ctx = this;
	_hal.board = HalBoard;
	_hal.board_name = HalBoardName;
	_hal.battery = HalBattery;
	_hal.time = HalTime;
	_hal.wifi = HalWifi;
	_hal.scan_start = HalScanStart;
	_hal.scan_state = HalScanState;
	_hal.scan_result = HalScanResult;
	_hal.join = HalJoin;
	_hal.forget = HalForget;
	_hal.drive = nullptr;
	_hal.screen = nullptr;
	_hal.app = HalApp;
	_hal.fetch = HalFetch;
	_hal.fetch_stop = HalFetchStop;
	_hal.alloc = HalAlloc;
	_hal.release = HalRelease;
	_hal.lock = HalLock;
	_hal.unlock = HalUnlock;
	_hal.line_rate = HalLineRate;
	_hal.line_write = HalLineWrite;
	_hal.line_idle = HalLineIdle;
	_hal.line_hear = nullptr;
	_hal.tracks = HalTracks;
	_hal.track_size = HalTrackSize;
	_hal.track_read = HalTrackRead;
	_hal.show = HalShow;
	_hal.watch = nullptr;
	_hal.peer_state = HalPeerState;
	_hal.peer_listen = HalPeerListen;
	_hal.peer_call = HalPeerCall;
	_hal.peer_close = HalPeerClose;
	_hal.peer_send = HalPeerSend;
	_hal.peer_recv = HalPeerRecv;
	_hal.peer_last = HalPeerLast;
	_hal.peer_keep = HalPeerKeep;
	_hal.airport_last = HalAirportLast;
	_hal.airport_keep = HalAirportKeep;
	_hal.zip_last = HalZipLast;
	_hal.zip_keep = HalZipKeep;
	_hal.favorites_last = HalFavoritesLast;
	_hal.favorites_keep = HalFavoritesKeep;
	_hal.peer_rtt = HalPeerRtt;
	_hal.peer_note = HalPeerNote;
	_hal.peer_noted = HalPeerNoted;
	_hal.peer_spread = HalPeerSpread;
	//Online play across the Internet and the Bluetooth keyboard are not
	//emulated yet: the firmware takes NULL as "this board cannot".
	_hal.peer_meet = nullptr;
	_hal.peer_candidates = nullptr;
	_hal.peer_punch = nullptr;
	_hal.kbd_enable = nullptr;
	_hal.kbd_state = nullptr;
	_hal.kbd_scan = nullptr;
	_hal.kbd_found = nullptr;
	_hal.kbd_pair = nullptr;
	_hal.kbd_forget = nullptr;
	_hal.kbd_read = nullptr;
	_hal.setting = HalSetting;
}

//A setting the console is never told, such as HARBOR's "aishub_key": from
//the environment as NESP32_AISHUB_KEY, or else from <Mesen home>/Nesp32/settings.txt,
//a line of name=value for each ('#' begins a comment).
bool Nesp32Device::HalSetting(void* ctx, const char* name, char* out, size_t max)
{
	(void)ctx;
	string key = name;
	string envName = "NESP32_";
	for(char c : key) {
		envName += (char)toupper((unsigned char)c);
	}
	string value;
	bool found = false;
#ifdef _WIN32
	char* env = nullptr;
	size_t envLen = 0;
	if(_dupenv_s(&env, &envLen, envName.c_str()) == 0 && env) {
		value = env;
		found = true;
	}
	free(env);
#else
	if(const char* env = getenv(envName.c_str())) {
		value = env;
		found = true;
	}
#endif
	if(!found) {
		string path = FolderUtilities::CombinePath(FolderUtilities::CombinePath(FolderUtilities::GetHomeFolder(), "Nesp32"), "settings.txt");
		std::ifstream in(path);
		string line;
		while(std::getline(in, line)) {
			size_t hash = line.find('#');
			if(hash != string::npos) {
				line = line.substr(0, hash);
			}
			size_t eq = line.find('=');
			if(eq == string::npos) {
				continue;
			}
			auto trim = [](string s) {
				size_t a = s.find_first_not_of(" \t\r\n");
				size_t b = s.find_last_not_of(" \t\r\n");
				return a == string::npos ? string() : s.substr(a, b - a + 1);
			};
			if(trim(line.substr(0, eq)) == key) {
				value = trim(line.substr(eq + 1));
				found = true;
			}
		}
	}
	if(!found || value.empty() || value.size() + 1 > max) {
		return false;
	}
	memcpy(out, value.c_str(), value.size() + 1);
	return true;
}

//The jukebox's files: <Mesen home>/Nesp32/tracks, in the order of their names
void Nesp32Device::LoadTracks()
{
	string folder = FolderUtilities::CombinePath(FolderUtilities::CombinePath(FolderUtilities::GetHomeFolder(), "Nesp32"), "tracks");
	FolderUtilities::CreateFolder(folder);
	_tracks = FolderUtilities::GetFilesInFolder(folder, { ".jbs", ".pcm", ".vgm", ".vgz" }, false);
	std::sort(_tracks.begin(), _tracks.end());
}

void Nesp32Device::InternalSetStateFromInput()
{
	//The board's two buttons are custom keys (Input > Port 2 > NESP32 > Configure)
	for(KeyMapping& keyMapping : _keyMappings) {
		SetPressedState(Buttons::A, KeyManager::IsKeyPressed(keyMapping.CustomKeys[0]));
		SetPressedState(Buttons::B, KeyManager::IsKeyPressed(keyMapping.CustomKeys[1]));
	}
}

//Once a frame: the board's buttons, and its services' time
void Nesp32Device::OnAfterSetState()
{
	static const uint8_t events[2] = { NESP_EVENT_BUTTON_A, NESP_EVENT_BUTTON_B };
	for(int i = 0; i < 2; i++) {
		bool pressed = IsPressed(i);
		if(pressed && !_wasPressed[i]) {
			nesp_stick_event(_stick.get(), events[i], 0);
		}
		_wasPressed[i] = pressed;
	}
	Advance(_emu->GetMasterClock());
}

//Brings the board up to a cycle: what the UART has heard goes to the link,
//its reply goes on the line, and the services get their tick.
void Nesp32Device::Advance(uint64_t now)
{
	if(now < _now) {
		//Time went back (a reset, or a saved state loaded): the line starts over
		TxReset();
		_tap.Reset();
		_searchFrom = (double)now;
		_heardAt = (double)now;
		_nextTick = now;
	}
	_now = now;

	_heard.clear();
	_tap.Decode(_searchFrom, (double)now, _bit, _heard);
	if(!_heard.empty()) {
		_heardAt = _heard.back().second;
		uint8_t data[1024];
		size_t len = std::min(_heard.size(), sizeof(data));
		for(size_t i = 0; i < len; i++) {
			data[i] = _heard[i].first;
		}
		uint8_t reply[NESP_FRAME_MAX];
		size_t replyLen = nesp_stick_feed(_stick.get(), data, len, reply);
		if(replyLen > 0) {
			double at = std::max(_heardAt + Turnaround, (double)now);
			TxSend(reply, replyLen, at);
			nesp_link_note_turnaround(&_stick->link, (uint32_t)((at - _heardAt) * 1000000.0 / NESP_CPU_HZ));
		}
		_nextTick = now; //a tick right after every reply, as the board's link task does
	}

	if(now >= _nextTick) {
		uint32_t nowMs = (uint32_t)(now * 1000 / NESP_CPU_HZ);
		bool soon = nesp_stick_tick(_stick.get(), nowMs);
		_nextTick = now + (soon ? BusyTickCycles : IdleTickCycles);
	}
}

uint8_t Nesp32Device::ReadRam(uint16_t addr)
{
	if(!IsCurrentPort(addr)) {
		return 0;
	}
	uint64_t now = _emu->GetMasterClock();
	Advance(now);
	//The console's '368 buffer inverts D0
	return (_wire.Level((double)now) ^ 1) & 0x01;
}

void Nesp32Device::WriteRam(uint16_t addr, uint8_t value)
{
	if(addr != 0x4016) {
		return;
	}
	uint64_t now = _emu->GetMasterClock();
	Advance(now);
	_tap.Write((double)now, value & 0x01);
}

// --- the hardware layer --------------------------------------------------

uint8_t Nesp32Device::HalBoard(void*) { return NESP_BOARD_HOST; }
const char* Nesp32Device::HalBoardName(void*) { return "Mesen"; }
uint8_t Nesp32Device::HalBattery(void*) { return 0xFF; }

bool Nesp32Device::HalTime(void*, uint32_t* seconds)
{
	*seconds = (uint32_t)std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	return true;
}

void Nesp32Device::HalWifi(void* ctx, nesp_wifi_t* out)
{
	*out = Self(ctx)->_wifi;
}

bool Nesp32Device::HalScanStart(void* ctx)
{
	Self(ctx)->_scanState = NESP_SCAN_DONE;
	return true;
}

uint8_t Nesp32Device::HalScanState(void* ctx, uint8_t* count)
{
	*count = Self(ctx)->_scanState == NESP_SCAN_DONE ? 1 : 0;
	return Self(ctx)->_scanState;
}

bool Nesp32Device::HalScanResult(void*, uint8_t index, nesp_network_t* out)
{
	if(index != 0) {
		return false;
	}
	memset(out, 0, sizeof(*out));
	out->rssi = -45;
	out->auth = NESP_AUTH_OPEN;
	out->channel = 6;
	out->name_len = (uint8_t)strlen(NetworkName);
	memcpy(out->name, NetworkName, out->name_len);
	return true;
}

bool Nesp32Device::HalJoin(void* ctx, const char* name, uint8_t nameLen, const char*, uint8_t)
{
	//Any network, any password: the computer's own connection is the one used
	Nesp32Device* self = Self(ctx);
	self->_wifi.state = NESP_WIFI_JOINED;
	self->_wifi.why = NESP_WHY_NONE;
	self->_wifi.name_len = std::min<uint8_t>(nameLen, NESP_NAME_MAX);
	memcpy(self->_wifi.name, name, self->_wifi.name_len);
	return true;
}

void Nesp32Device::HalForget(void* ctx)
{
	Nesp32Device* self = Self(ctx);
	self->_wifi.state = NESP_WIFI_IDLE;
	self->_wifi.name_len = 0;
}

void Nesp32Device::HalApp(void* ctx, const char* id, uint8_t)
{
	Nesp32Device* self = Self(ctx);
	string app(id, strnlen(id, NESP_APP_ID));
	if(app != self->_app) {
		self->_app = app;
		MessageManager::Log("[NESP32] App: " + app);
	}
}

bool Nesp32Device::HalFetch(void* ctx, const char* url, const char* agent, uint32_t id, const nesp_sink_t* sink)
{
	return Self(ctx)->_fetcher.Fetch(url, agent, id, sink);
}

void Nesp32Device::HalFetchStop(void* ctx)
{
	Self(ctx)->_fetcher.Stop();
}

void* Nesp32Device::HalAlloc(void*, size_t bytes) { return malloc(bytes); }
void Nesp32Device::HalRelease(void*, void* memory) { free(memory); }
void Nesp32Device::HalLock(void* ctx) { Self(ctx)->_lock.lock(); }
void Nesp32Device::HalUnlock(void* ctx) { Self(ctx)->_lock.unlock(); }

void Nesp32Device::HalLineRate(void* ctx, uint8_t period)
{
	Nesp32Device* self = Self(ctx);
	self->_bit = period;
	self->_searchFrom = (double)self->_now; //what the receiver held is dropped
}

size_t Nesp32Device::HalLineWrite(void* ctx, const uint8_t* data, size_t len)
{
	Nesp32Device* self = Self(ctx);
	size_t waiting = self->TxWaiting();
	size_t room = waiting < TxBuffer ? TxBuffer - waiting : 0;
	size_t count = std::min(len, room);
	if(count > 0) {
		self->TxSend(data, count, (double)self->_now);
	}
	return count;
}

bool Nesp32Device::HalLineIdle(void* ctx)
{
	return Self(ctx)->TxIdle();
}

//The UART on D0
void Nesp32Device::TxSend(const uint8_t* data, size_t len, double at)
{
	_wire.Send(data, len, at, _bit);
}

size_t Nesp32Device::TxWaiting()
{
	return _wire.Waiting((double)_now, _bit);
}

bool Nesp32Device::TxIdle()
{
	return _wire.BusyUntil() <= (double)_now;
}

void Nesp32Device::TxReset()
{
	_wire.Reset();
}

uint16_t Nesp32Device::HalTracks(void* ctx)
{
	return (uint16_t)Self(ctx)->_tracks.size();
}

uint32_t Nesp32Device::HalTrackSize(void* ctx, uint16_t track)
{
	Nesp32Device* self = Self(ctx);
	if(track >= self->_tracks.size()) {
		return 0;
	}
	VirtualFile file(self->_tracks[track]);
	return (uint32_t)file.GetSize();
}

size_t Nesp32Device::HalTrackRead(void* ctx, uint16_t track, uint32_t offset, uint8_t* out, size_t len)
{
	Nesp32Device* self = Self(ctx);
	if(track >= self->_tracks.size()) {
		return 0;
	}
	ifstream file(self->_tracks[track], ios::binary);
	if(!file) {
		return 0;
	}
	file.seekg(offset);
	file.read((char*)out, (std::streamsize)len);
	return (size_t)std::max<std::streamsize>(0, file.gcount());
}

void Nesp32Device::HalShow(void*, uint8_t, const char*) {}

uint8_t Nesp32Device::HalPeerState(void*) { return NESP_CONN_NONE; }
bool Nesp32Device::HalPeerListen(void*, uint16_t) { return false; }
bool Nesp32Device::HalPeerCall(void*, const uint8_t*, uint16_t) { return false; }
void Nesp32Device::HalPeerClose(void*) {}
size_t Nesp32Device::HalPeerSend(void*, const uint8_t*, size_t) { return 0; }
size_t Nesp32Device::HalPeerRecv(void*, uint8_t*, size_t) { return 0; }
bool Nesp32Device::HalPeerLast(void*, uint8_t*) { return false; }
void Nesp32Device::HalPeerKeep(void*, const uint8_t*) {}
uint32_t Nesp32Device::HalPeerRtt(void*) { return 0; }
void Nesp32Device::HalPeerNote(void*, const uint8_t*) {}
void Nesp32Device::HalPeerNoted(void* ctx, uint8_t* note) { memcpy(note, Self(ctx)->_noteIn, NESP_ARQ_NOTE); }
void Nesp32Device::HalPeerSpread(void*, uint32_t* counts) { memset(counts, 0, sizeof(uint32_t) * NESP_ARQ_NEEDS); }

bool Nesp32Device::HalAirportLast(void* ctx, char* code, int32_t* latitude, int32_t* longitude)
{
	Nesp32Device* self = Self(ctx);
	if(!self->_haveAirport) {
		return false;
	}
	memcpy(code, self->_airport, 4);
	*latitude = self->_airportPlace[0];
	*longitude = self->_airportPlace[1];
	return true;
}

void Nesp32Device::HalAirportKeep(void* ctx, const char* code, int32_t latitude, int32_t longitude)
{
	Nesp32Device* self = Self(ctx);
	self->_haveAirport = true;
	memcpy(self->_airport, code, 4);
	self->_airportPlace[0] = latitude;
	self->_airportPlace[1] = longitude;
}

bool Nesp32Device::HalZipLast(void* ctx, char* zip, int32_t* latitude, int32_t* longitude, char* name)
{
	Nesp32Device* self = Self(ctx);
	if(!self->_haveZip) {
		return false;
	}
	memcpy(zip, self->_zip, NESP_WEATHER_ZIP_LEN);
	*latitude = self->_zipPlace[0];
	*longitude = self->_zipPlace[1];
	memcpy(name, self->_zipName, NESP_WEATHER_PLACE_MAX);
	return true;
}

void Nesp32Device::HalZipKeep(void* ctx, const char* zip, int32_t latitude, int32_t longitude, const char* name)
{
	Nesp32Device* self = Self(ctx);
	self->_haveZip = true;
	memcpy(self->_zip, zip, NESP_WEATHER_ZIP_LEN);
	self->_zipPlace[0] = latitude;
	self->_zipPlace[1] = longitude;
	memcpy(self->_zipName, name, NESP_WEATHER_PLACE_MAX);
}

bool Nesp32Device::HalFavoritesLast(void* ctx, nesp_bsky_favorites_t* out)
{
	Nesp32Device* self = Self(ctx);
	if(!self->_haveFavorites) {
		return false;
	}
	*out = self->_favorites;
	return true;
}

void Nesp32Device::HalFavoritesKeep(void* ctx, const nesp_bsky_favorites_t* favorites)
{
	Nesp32Device* self = Self(ctx);
	self->_haveFavorites = true;
	self->_favorites = *favorites;
}
