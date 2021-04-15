#pragma once
#include <stdint.h>
#include <mutex>
#include <vector>
#include "opencv2/core.hpp"
#include <iostream>

/* IMU data */
typedef struct __vs_imu_data {
	float         q0;
	float         q1;
	float         q2;
	float         q3;

	double        lati;
	double        longti;
	float         alti;
	float         height;

	float         vg_x;
	float         vg_y;
	float         vg_z;

	uint64_t      time_stamp;

	__vs_imu_data():q0(0),q1(0),q2(0),q3(0),lati(0),longti(0),alti(0),height(0),vg_x(0),vg_y(0),vg_z(0),time_stamp(0){}
	~__vs_imu_data() {}
} vs_imu_data_t, *vs_imu_data_handle_t;

struct point3d_t {
	float x;
	float y;
	float z;
};

struct depth_t {
	vs_imu_data_t imu;
	std::vector<float> data;
	cv::Mat rawImage;
};

struct mapvis_t {
	vs_imu_data_t imu;
	std::vector<point3d_t> pcl;
	cv::Mat rawImage;
};
