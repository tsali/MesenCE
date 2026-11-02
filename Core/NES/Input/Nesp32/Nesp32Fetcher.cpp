#include "pch.h"
#include "NES/Input/Nesp32/Nesp32Fetcher.h"
#include <cstring>

//The HTTP(S) client is the platform's own: WinHTTP on Windows (nothing to
//install), libcurl elsewhere.
#ifdef _WIN32
	#ifndef NOMINMAX
		#define NOMINMAX
	#endif
	#include <windows.h>
	#include <winhttp.h>
	#pragma comment(lib, "winhttp.lib")
#else
	#include <curl/curl.h>
#endif

Nesp32Fetcher::Nesp32Fetcher()
{
#ifndef _WIN32
	static std::once_flag curlInit;
	std::call_once(curlInit, []() { curl_global_init(CURL_GLOBAL_DEFAULT); });
#endif
}

Nesp32Fetcher::~Nesp32Fetcher()
{
	Stop();
	ReapThreads(true);
}

void Nesp32Fetcher::ReapThreads(bool all)
{
	vector<std::thread> threads;
	{
		std::lock_guard<std::mutex> lock(_threadsLock);
		if(!all && _threads.size() < 8) {
			return;
		}
		threads.swap(_threads);
	}
	//Every thread here belongs to a fetch that was replaced or stopped, so
	//each one ends at its next callback (OnBody/OnProgress see the
	//generation change).
	for(std::thread& t : threads) {
		if(t.joinable()) {
			t.join();
		}
	}
}

bool Nesp32Fetcher::Fetch(const char* url, const char* agent, uint32_t id, const nesp_sink_t* sink)
{
	if(url == nullptr || sink == nullptr) {
		return false;
	}

	uint32_t generation = ++_generation;
	ReapThreads(false);

	Job job;
	job.Owner = this;
	job.Generation = generation;
	job.Url = url;
	job.Agent = agent ? agent : "nesp32";
	job.Id = id;
	job.Sink = *sink;

	std::lock_guard<std::mutex> lock(_threadsLock);
	_threads.emplace_back(Run, std::move(job));
	return true;
}

void Nesp32Fetcher::Stop()
{
	++_generation;
}

#ifndef _WIN32
size_t Nesp32Fetcher::OnHeader(char* data, size_t size, size_t count, void* userdata)
{
	Job* job = (Job*)userdata;
	size_t len = size * count;
	string line(data, len);
	const char* name = "icy-metaint:";
	if(line.rfind("HTTP/", 0) == 0) {
		job->MetaInt = 0; //a new response (after a redirect): its own headers count
	} else if(line.size() > strlen(name) && strncasecmp(line.c_str(), name, strlen(name)) == 0) {
		job->MetaInt = (uint32_t)strtoul(line.c_str() + strlen(name), nullptr, 10);
	}
	return len;
}

#endif

size_t Nesp32Fetcher::OnBody(char* data, size_t size, size_t count, void* userdata)
{
	Job* job = (Job*)userdata;
	size_t len = size * count;
	if(!job->Owner->IsCurrent(job->Generation)) {
		return 0; //replaced or stopped: abort quietly
	}
	if(job->Sink.icy && !job->MetaIntSent) {
		job->MetaIntSent = true;
		job->Sink.icy(job->Sink.ctx, job->Id, job->MetaInt);
	}
	if(!job->Sink.body(job->Sink.ctx, job->Id, (const uint8_t*)data, len)) {
		job->Stopped = true;
		return 0;
	}
	return len;
}

#ifndef _WIN32
int Nesp32Fetcher::OnProgress(void* userdata, int64_t, int64_t, int64_t, int64_t)
{
	Job* job = (Job*)userdata;
	return job->Owner->IsCurrent(job->Generation) ? 0 : 1;
}

void Nesp32Fetcher::Run(Job job)
{
	CURL* curl = curl_easy_init();
	if(curl == nullptr) {
		if(job.Owner->IsCurrent(job.Generation)) {
			job.Sink.done(job.Sink.ctx, job.Id, false, "NO HTTP");
		}
		return;
	}

	struct curl_slist* headers = nullptr;
	if(job.Sink.icy) {
		headers = curl_slist_append(headers, "Icy-MetaData: 1");
	}

	char error[CURL_ERROR_SIZE] = {};
	curl_easy_setopt(curl, CURLOPT_URL, job.Url.c_str());
	curl_easy_setopt(curl, CURLOPT_USERAGENT, job.Agent.c_str());
	curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
	curl_easy_setopt(curl, CURLOPT_MAXREDIRS, 8L);
	curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 15L);
	//A stream never ends, so no overall timeout; give up on a stalled one.
	curl_easy_setopt(curl, CURLOPT_LOW_SPEED_LIMIT, 1L);
	curl_easy_setopt(curl, CURLOPT_LOW_SPEED_TIME, 30L);
	curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, OnHeader);
	curl_easy_setopt(curl, CURLOPT_HEADERDATA, &job);
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, OnBody);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &job);
	curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
	curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, OnProgress);
	curl_easy_setopt(curl, CURLOPT_XFERINFODATA, &job);
	curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);
	curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

	CURLcode result = curl_easy_perform(curl);
	long status = 0;
	curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
	curl_easy_cleanup(curl);
	curl_slist_free_all(headers);

	if(!job.Owner->IsCurrent(job.Generation)) {
		return; //nothing more is said of a fetch that was replaced or stopped
	}

	if(job.Sink.icy && !job.MetaIntSent) {
		job.Sink.icy(job.Sink.ctx, job.Id, job.MetaInt);
	}

	if(job.Stopped || (result == CURLE_OK && status >= 200 && status < 300)) {
		job.Sink.done(job.Sink.ctx, job.Id, true, "");
	} else if(result == CURLE_OK) {
		char why[32];
		snprintf(why, sizeof(why), "HTTP %ld", status);
		job.Sink.done(job.Sink.ctx, job.Id, false, why);
	} else if(result == CURLE_OPERATION_TIMEDOUT) {
		job.Sink.done(job.Sink.ctx, job.Id, false, "TIMED OUT");
	} else if(result == CURLE_COULDNT_RESOLVE_HOST) {
		job.Sink.done(job.Sink.ctx, job.Id, false, "NO SUCH HOST");
	} else {
		job.Sink.done(job.Sink.ctx, job.Id, false, "NO CONNECTION");
	}
}
#endif

#ifdef _WIN32
static std::wstring Widen(const string& text)
{
	if(text.empty()) {
		return std::wstring();
	}
	int len = MultiByteToWideChar(CP_UTF8, 0, text.c_str(), (int)text.size(), nullptr, 0);
	std::wstring out(len, L'\0');
	MultiByteToWideChar(CP_UTF8, 0, text.c_str(), (int)text.size(), &out[0], len);
	return out;
}

void Nesp32Fetcher::Run(Job job)
{
	const char* why = "NO CONNECTION";
	bool ok = false;
	DWORD status = 0;

	std::wstring url = Widen(job.Url);
	URL_COMPONENTS parts = {};
	parts.dwStructSize = sizeof(parts);
	wchar_t host[256] = {};
	wchar_t path[2048] = {};
	wchar_t extra[2048] = {};
	parts.lpszHostName = host;
	parts.dwHostNameLength = (DWORD)(sizeof(host) / sizeof(wchar_t));
	parts.lpszUrlPath = path;
	parts.dwUrlPathLength = (DWORD)(sizeof(path) / sizeof(wchar_t));
	parts.lpszExtraInfo = extra;
	parts.dwExtraInfoLength = (DWORD)(sizeof(extra) / sizeof(wchar_t));

	HINTERNET session = nullptr, connection = nullptr, request = nullptr;
	if(WinHttpCrackUrl(url.c_str(), 0, 0, &parts)) {
		session = WinHttpOpen(Widen(job.Agent).c_str(), WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
	} else {
		why = "BAD ADDRESS";
	}
	if(session) {
		//Resolve, connect, send, and wait for data: a stalled stream gives up after 30s
		WinHttpSetTimeouts(session, 15000, 15000, 15000, 30000);
		connection = WinHttpConnect(session, host, parts.nPort, 0);
	}
	if(connection) {
		std::wstring target = std::wstring(path) + extra;
		DWORD flags = parts.nScheme == INTERNET_SCHEME_HTTPS ? WINHTTP_FLAG_SECURE : 0;
		request = WinHttpOpenRequest(connection, L"GET", target.c_str(), nullptr, WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, flags);
	}
	if(request) {
		const wchar_t* headers = job.Sink.icy ? L"Icy-MetaData: 1\r\n" : WINHTTP_NO_ADDITIONAL_HEADERS;
		if(WinHttpSendRequest(request, headers, job.Sink.icy ? (DWORD)-1L : 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0)
			&& WinHttpReceiveResponse(request, nullptr)) {
			DWORD size = sizeof(status);
			WinHttpQueryHeaders(request, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX, &status, &size, WINHTTP_NO_HEADER_INDEX);

			wchar_t metaint[32] = {};
			DWORD metaintSize = sizeof(metaint);
			if(WinHttpQueryHeaders(request, WINHTTP_QUERY_CUSTOM, L"icy-metaint", metaint, &metaintSize, WINHTTP_NO_HEADER_INDEX)) {
				job.MetaInt = (uint32_t)wcstoul(metaint, nullptr, 10);
			}

			if(status >= 200 && status < 300) {
				char buffer[8192];
				ok = true;
				while(job.Owner->IsCurrent(job.Generation)) {
					DWORD got = 0;
					if(!WinHttpReadData(request, buffer, sizeof(buffer), &got)) {
						ok = false;
						why = "TIMED OUT";
						break;
					}
					if(got == 0) {
						break; //the whole reply came
					}
					if(OnBody(buffer, 1, got, &job) != got) {
						break; //replaced, or the sink wants no more
					}
				}
			}
		}
	}
	if(request) WinHttpCloseHandle(request);
	if(connection) WinHttpCloseHandle(connection);
	if(session) WinHttpCloseHandle(session);

	if(!job.Owner->IsCurrent(job.Generation)) {
		return; //nothing more is said of a fetch that was replaced or stopped
	}
	if(job.Sink.icy && !job.MetaIntSent) {
		job.Sink.icy(job.Sink.ctx, job.Id, job.MetaInt);
	}
	if(job.Stopped || ok) {
		job.Sink.done(job.Sink.ctx, job.Id, true, "");
	} else if(status != 0) {
		char text[32];
		snprintf(text, sizeof(text), "HTTP %lu", (unsigned long)status);
		job.Sink.done(job.Sink.ctx, job.Id, false, text);
	} else {
		job.Sink.done(job.Sink.ctx, job.Id, false, why);
	}
}
#endif
