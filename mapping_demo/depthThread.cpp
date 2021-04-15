#include "depthThread.h"
#include <iostream>
#include <string>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <windows.h>
#include <chrono>
#include "datatype.h"
#include <thread>
#include <atomic>
#include <fstream>
/* opencv */
#   include <opencv2/core/core.hpp>
#   include <opencv2/highgui/highgui.hpp>
#   include <opencv2/imgproc/imgproc.hpp>

#include "fileio.h"
#include "config.h"

using namespace std;

#pragma warning(disable:4996)

static std::thread thread1;

bool threadDepthFileReader::scanningFiles()
{
	findCorrespondingFiles(filepath, filelists, "png");
	cnt = 0;

	// read imu
	std::string imutxtfile = filepath + "airsim_rec.txt";
	std::ifstream filereader(imutxtfile);
	imus.clear();
	if (filereader.is_open())
	{
		std::string line;
		int first_line = 1;
		while (!filereader.eof())
		{
			if (first_line == 1)
			{
				first_line = 0;
				std::getline(filereader, line);
				cout << line << std::endl;
				continue;
			}
			else
			{
				uint64_t ts;
				float px, py, pz, qw, qx, qy, qz;
				std::string filename;
				filereader >> ts >> px >> py >> pz >> qw >> qx >> qy >> qz >> filename;
				//cout << ts << ";" << px << ";" << py << ";" << pz << ";" << qw << ";" << qx << ";" << qy << ";" << qz << ";" << filename << std::endl;
				vs_imu_data_t imu;
				imu.time_stamp = ts;
				imu.lati = px;
				imu.longti = py;
				imu.alti = pz;
				imu.q0 = qw;
				imu.q1 = qx;
				imu.q2 = qy;
				imu.q3 = qz;
				imus.push_back(imu);
			}
		}
	}

	return true;
}

bool threadDepthFileReader::readNextDepthFile(depth_t &data)
{
	if (cnt < filelists.size())
	{

		std::string imgfile = filelists[cnt];
		std::string name = getFileName(imgfile);

		std::cout << name + ".txt" << std::endl;

		std::string depthfile = filepath + name + ".txt";

		std::ifstream filereader(depthfile);

		data.data.resize(IWIDTH*IHEIGHT);

		if (filereader.is_open())
		{
			for (int i = 0; i < IWIDTH * IHEIGHT; i++)
			{
				float d;
				filereader >> d;
				data.data[i] = d;
				//std::cout << d << "; " << std::endl;
			}
		}
		filereader.clear();

		cv::Mat img = cv::imread(imgfile, 1);
		img.copyTo(data.rawImage);

		// read imu and set to data
		memcpy(&data.imu, &imus[cnt], sizeof(vs_imu_data_t));

		//cout << data.imu.time_stamp << ";" << data.imu.lati << ";" << data.imu.q3 << std::endl;

		cnt = cnt + 1;

		return true;
	}
	else
		return false;
}


unsigned int depthreadingThread(unsigned int tid,  void* pData)
{
	threadDepthFileReader* pThreadParameter = reinterpret_cast<threadDepthFileReader*>(pData);
	SafeQueue<depth_t>& queue = pThreadParameter->depthBuf;

	depth_t currFrame;

	while (1)
	{
		// if exit request, then exit
		if (pThreadParameter->exit == 1)
			break;

		// if cosuming is to slow, then producer will wiat 
		if (queue.size() >= 30)
		{
			Sleep(1000);
			continue;
		}

		// if either, read one file
		std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
		bool end = pThreadParameter->readNextDepthFile(currFrame);
		std::chrono::time_point<std::chrono::system_clock> t2 = std::chrono::system_clock::now();
		std::chrono::duration<double> lapse = t2 - t1;
		std::cout << "png compression time: " << lapse.count() << endl;
		if (end == true)
			queue.enqueue(currFrame);
		else
			pThreadParameter->exit = 1;

		Sleep(100);
	}

	return 0;
}


//-----------------------------------------------------------------------------
bool startDepthFileReading(threadDepthFileReader& handler)
//-----------------------------------------------------------------------------
{
	handler.scanningFiles();
	std::cout << "INFO: Launching image encoding thread..." << std::endl;

	thread1 = std::thread(depthreadingThread, 0, &handler);

	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	return true;
}

bool stopDepthFileReading(threadDepthFileReader& handler)
{
	cout << "INFO: Terminating all encoding threads..." << endl;
	
	handler.exit = 1;

	if (thread1.joinable())
	{
		thread1.join();
	}

	cout << "INFO: Encoding Threads terminated successfully!" << endl;
	return true;
}