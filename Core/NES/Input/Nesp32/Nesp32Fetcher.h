#pragma once
#include "pch.h"
#include <atomic>
#include <mutex>
#include <thread>

extern "C" {
#include "NES/Input/Nesp32/firmware/nesp_services.h"
}

//The NESP32's fetches (nesp_hal_t's fetch and fetch_stop): a real HTTP(S)
//request on a worker thread, whose reply goes to the service's sink, as the
//board's own fetch task does (firmware/main/fetch.c). A new fetch takes the
//place of the one under way; the sink is told nothing more of the old one.
class Nesp32Fetcher
{
private:
	std::atomic<uint32_t> _generation = { 0 };
	std::mutex _threadsLock;
	vector<std::thread> _threads;

	struct Job
	{
		Nesp32Fetcher* Owner;
		uint32_t Generation;
		string Url;
		string Agent;
		uint32_t Id;
		nesp_sink_t Sink;
		uint32_t MetaInt = 0;
		bool MetaIntSent = false;
		bool Stopped = false; //the sink said no more of the body is wanted
	};

	static void Run(Job job);
	static size_t OnHeader(char* data, size_t size, size_t count, void* userdata);
	static size_t OnBody(char* data, size_t size, size_t count, void* userdata);
	static int OnProgress(void* userdata, int64_t, int64_t, int64_t, int64_t);

	bool IsCurrent(uint32_t generation) const { return _generation.load() == generation; }
	void ReapThreads(bool all);

public:
	Nesp32Fetcher();
	~Nesp32Fetcher();

	bool Fetch(const char* url, const char* agent, uint32_t id, const nesp_sink_t* sink);
	void Stop();
};
