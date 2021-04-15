#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#include <mutex>
#endif
#include "safe_queue.h"
#include "datatype.h"

struct threadDepthFileReader
{
	std::string &filepath;
	SafeQueue<depth_t> &depthBuf;
	std::vector<std::string > filelists;
	std::vector<vs_imu_data_t> imus;
	int cnt;
	int &exit;
	threadDepthFileReader(std::string &base, SafeQueue<depth_t> &_depthBuf, int &_exit) : filepath(base), depthBuf(_depthBuf), exit(_exit), cnt(0){}
	~threadDepthFileReader() {}


	bool scanningFiles();
	bool readNextDepthFile(depth_t &data);

};

bool startDepthFileReading(threadDepthFileReader &handler);
bool stopDepthFileReading(threadDepthFileReader &handler);