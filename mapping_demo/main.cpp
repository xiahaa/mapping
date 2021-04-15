// pangolin_test.cpp : Defines the entry point for the console application.
//

#include <pangolin/pangolin.h>
#include <pangolin\gl\gldraw.h>
#include <time.h>
#include <stdlib.h>
#include "opencv2\core\core.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <stdint.h>
#include "depthThread.h"
#include "mappingThread.h"
#include "opencv2/highgui/highgui.hpp"
#include <Eigen/Core>
#include <Eigen/src/Geometry/AlignedBox.h>
#include <Eigen/Dense>
#include "config.h"

#ifdef _DEBUG
#pragma comment(lib, "pangolin.lib")
#pragma comment(lib, "glew32.lib")
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "opencv_core310d.lib")
#pragma comment(lib, "opencv_highgui310d.lib")
#pragma comment(lib, "opencv_imgcodecs310d.lib")
#else
#pragma comment(lib, "pangolin.lib")
#pragma comment(lib, "glew32.lib")
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "opencv_core310.lib")
#pragma comment(lib, "opencv_highgui310.lib")
#pragma comment(lib, "opencv_imgcodecs310.lib")
#endif

template <typename T>
inline void quaternion_to_rotationMatrix(const T *q, T *rm)
{
	// quaternion to rotation matrix, q must be in x y z w
	T x2 = 2.0f*q[0] * q[0];
	T y2 = 2.0f*q[1] * q[1];
	T z2 = 2.0f*q[2] * q[2];

	T xy2 = 2.0f*q[0] * q[1];
	T xz2 = 2.0f*q[0] * q[2];
	T xw2 = 2.0f*q[0] * q[3];

	T yz2 = 2.0f*q[1] * q[2];
	T yw2 = 2.0f*q[1] * q[3];

	T zw2 = 2.0f*q[2] * q[3];

	rm[0] = 1.0f - y2 - z2;
	rm[1] = xy2 - zw2;
	rm[2] = xz2 + yw2;
	rm[3] = xy2 + zw2;
	rm[4] = 1.0f - x2 - z2;
	rm[5] = yz2 - xw2;
	rm[6] = xz2 - yw2;
	rm[7] = yz2 + xw2;
	rm[8] = 1.0f - x2 - y2;
}

void mapJet(double v, double vmin, double vmax, float &r, float& g, float& b)
{
	/*r = 255;
	g = 255;
	b = 255;*/

	if (v < vmin) {
		v = vmin;
	}

	if (v > vmax) {
		v = vmax;
	}

	v = (v - vmin) / (vmax - vmin);

	double dr, dg, db;

	if (v < 0.1242) {
		db = 0.504 + ((1. - 0.504) / 0.1242)*v;
		dg = dr = 0.;
	}
	else if (v < 0.3747) {
		db = 1.;
		dr = 0.;
		dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
	}
	else if (v < 0.6253) {
		db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
		dg = 1.;
		dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
	}
	else if (v < 0.8758) {
		db = 0.;
		dr = 1.;
		dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
	}
	else {
		db = 0.;
		dg = 0.;
		dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
	}

	r = dr;
	g = dg;
	b = db;
}

void pangolin_display(void *mbuf)
{
	SafeQueue<mapvis_t> *phand = (SafeQueue<mapvis_t> *)mbuf;

	pangolin::CreateWindowAndBind("Main", 640, 480);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Define Projection and initial ModelView matrix
	pangolin::OpenGlRenderState s_cam(
		pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
		pangolin::ModelViewLookAt(2, 2, -2, 0, 0, 0, pangolin::AxisX)
	);

	// Create Interactive View in window
	pangolin::Handler3D handler(s_cam);
	pangolin::View& d_cam = pangolin::CreateDisplay()
		.SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
		.SetHandler(&handler);
	mapvis_t mapdata;

	std::vector<point3d_t> traj;
	traj.clear();
	
	cv::namedWindow("rawView",cv::WINDOW_NORMAL);

	Eigen::Matrix<float, 4, 4> T_wf;
	Eigen::Matrix<float, 3, 3> K;
	K << F, 0, CU,
		0, F, CV,
		0, 0, 1;
	int w = 320;
	int h = 240;
	float scale = 1;
	Eigen::Matrix<float, 3, 3> Kinv;
	Kinv = K.inverse();

	std::vector<point3d_t> pcl;

	int num = 0;
	while (!pangolin::ShouldQuit())
	{
#if 0
		pangolin::GlBuffer vertex(pangolin::GlArrayBuffer, num, GL_FLOAT, 3, GL_STATIC_DRAW);
		pangolin::GlBuffer color(pangolin::GlArrayBuffer, num, GL_UNSIGNED_BYTE, 4, GL_STATIC_DRAW);

		vertex.Upload(pointcloud, 3 * sizeof(float)*num);
		color.Upload(colorcloud, 4 * sizeof(uchar)*num);
#endif
#if 0
		if ((clock() - prev) / CLOCKS_PER_SEC > 1)
		{
			prev = clock();
			x = float(rand()) / RAND_MAX * 2;
			y = float(rand()) / RAND_MAX * 2;
			z = -float(rand()) / RAND_MAX * 2;

			r = float(rand()) / RAND_MAX * 0.5 + 0.5;
			g = float(rand()) / RAND_MAX * 0.5 + 0.5;
			b = float(rand()) / RAND_MAX * 0.5 + 0.5;
			size = float(rand()) / RAND_MAX + 0.1;
			alpha = float(rand()) / RAND_MAX * 0.5 + 0.5;

			//printf("%f %f %f\n", r, g, b);
		}
#endif

		// Clear screen and activate view to render into
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		d_cam.Activate(s_cam);
	
		// Render OpenGL Cube
		//pangolin::glDrawColouredCube();
		//pangolin::glDrawColoredCubeCustermized(x, y, z, size, r, g, b, alpha);

		glColor4f(1, 1, 1, 0.8);
		glLineWidth(1);
		pangolin::glDraw_z0(1, 10);

		//glColor4f(0, 1, 0, 1.0);
		//glLineWidth(2);
		//pangolin::glDrawFrustrum(-160.0/250, -120.0/250, 1.0/250, 1.0/250, 320, 240, 1);
		//glLineWidth(1);

		if (phand->size() > 0)
		{
			mapdata = phand->dequeue();
			point3d_t pos;
			pos.x = mapdata.imu.lati; pos.y = mapdata.imu.longti; pos.z = mapdata.imu.alti;
			traj.push_back(pos);

			float q[4] = { mapdata.imu.q1, mapdata.imu.q2, mapdata.imu.q3, mapdata.imu.q0 };
			float r[9];
			quaternion_to_rotationMatrix(q, r);
			Eigen::Matrix3f R1(r);
			Eigen::Matrix3f R2;
			R2 << 0, 0, 1, 1, 0, 0, 0, 1, 0;
			Eigen::Matrix3f R3;
			R3 << 0, 1, 0, 1, 0, 0, 0, 0, -1;
			Eigen::Matrix3f R4 = R3*R1*R2;

			T_wf << R4(0, 0), R4(0, 1), R4(0, 2), (float)(mapdata.imu.lati),
				R4(1, 0), R4(1, 1), R4(1, 2), (float)(mapdata.imu.longti),
				R4(2, 0), R4(2, 1), R4(2, 2), (float)(mapdata.imu.alti),
				0, 0, 0, 1;

			pcl = mapdata.pcl;
			cv::imshow("rawView", mapdata.rawImage);
			cv::waitKey(100);
			for (int i = 0; i < pcl.size(); i++)
			{
				float z = pcl[i].z;
				float r = 1, g = 1, b = 1;
				mapJet(-z, 0, 10, r, g, b);
				pangolin::glDrawColoredCubeCustermized(pcl[i].x, pcl[i].y, pcl[i].z, 0.25, r, g, b, 0.8);
			}
		}

		glLineWidth(3);
		glColor4f(0, 0, 1, 1);
		pangolin::glDrawFrustrum(Kinv, w, h, T_wf, scale);
		glLineWidth(2);
		
		if (traj.size() > 0)
		{
			for (size_t j = 0; j < traj.size() - 1; j++) {
				glColor3f(0, 1, 0);
				glBegin(GL_LINES);
				auto p3 = traj[j], p4 = traj[j + 1];
				glVertex3d(p3.x, p3.y, p3.z);
				glVertex3d(p4.x, p4.y, p4.z);
				glEnd();
			}
		}
#if 0
		pangolin::RenderVboCbo(vertex, color);
#endif
		// Swap frames and Process Events
		pangolin::FinishFrame();

		Sleep(5);   // sleep 5 ms
	}
}


int main(int argc, char** argv)
{
	float x = 0, y = 0, z = 0;
	float r = 1, g = 1, b = 1;
	float alpha = 0.5;
	float size = 1;
	srand(time(NULL));

	clock_t prev = clock();
	SafeQueue<depth_t> dbuf;
	SafeQueue<mapvis_t> mbuf;
	int exit = 0;
	std::string base = "D:\\dtu\\sampledata\\airsim\\smalldemo\\2018-10-05-15-42-57\\front\\";

	threadDepthFileReader dephandle = threadDepthFileReader(base, dbuf, exit);
	threadMapping maphandle = threadMapping(dbuf, exit, mbuf);
	std::thread t1 = std::thread(pangolin_display, &mbuf);
	startDepthFileReading(dephandle);
	startMapping(maphandle);
	t1.join();


	//while (1)
	//{
	//	if (exit == 1)
	//		break;
	//}

	//stopDepthFileReading(dephandle);
	//stopMapping(maphandle);
	//t1.detach();

#if 0
	if (pointcloud != NULL)
		std::free(pointcloud);
	if (colorcloud != NULL)
		std::free(colorcloud);
#endif
	return 0;
}
