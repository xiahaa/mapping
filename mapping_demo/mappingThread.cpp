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

#include "config.h"
#include "mappingThread.h"

using namespace std;

#pragma warning(disable:4996)
#pragma warning(disable:4244)

static std::thread thread1;

bool mapper::map_init(int order)
{
	// prev curr center, memory allocation, mmeset to 0, look through table
	prev_cx_grid = 0; prev_cy_grid = 0; prev_cz_grid = 0;
	curr_cx_grid = 0; curr_cy_grid = 0; curr_cz_grid = 0;

	m_f = F; m_cu = CU; m_cv = CV;

	m_grid_precision_order = 0;
	m_grid_precision = 0;
	m_grid_coord_multipler = 0;

	m_lookthrough_tb_cnt = 0;

	if (m_bb_data_ptr != NULL)
		free(m_bb_data_ptr);
	else
	{
		m_bb_data_ptr = (bb_data_t *)calloc(BB_SIZE_XYZ, sizeof(bb_data_t));
		if (m_bb_data_ptr == NULL)
			return false;
	}

	if (m_cur_bb_data_ptr != NULL)
		free(m_cur_bb_data_ptr);
	else
	{
		m_cur_bb_data_ptr = (bb_data_t *)calloc(BB_SIZE_XYZ, sizeof(bb_data_t));
		if (m_cur_bb_data_ptr == NULL)
			return false;
	}	
	//m_cur_bb_data_ptr = NULL;
	//m_pre_bb_data_ptr = NULL;


	if (m_lookthrough_tb_ptr != NULL)
		free(m_lookthrough_tb_ptr);
	else
	{
		m_lookthrough_tb_ptr = (int *)calloc(DEPTH_MAP_WIDTH*DEPTH_MAP_HEIGHT*4, sizeof(int));
		if (m_lookthrough_tb_ptr == NULL)
			return false;
	}
	
	m_init = true;

	m_grid_precision_order = order;
	m_grid_precision = 1./(1 << m_grid_precision_order);
	m_grid_coord_multipler = (1 << m_grid_precision_order);

	return true;
}

bool mapper::map_release()
{
	// free all allocate memory
	if (m_bb_data_ptr != NULL)
		free(m_bb_data_ptr);

	if (m_lookthrough_tb_ptr != NULL)
		free(m_lookthrough_tb_ptr);

	if (m_cur_bb_data_ptr != NULL)
		free(m_cur_bb_data_ptr);

	m_init = true;

	return true;
}

bool mapper::update_bb_to_new_pose()
{
	int i, j, k;
	int bb_coord_x, bb_coord_y, bb_coord_z;
	int delta_x, delta_y, delta_z;
	int grid_coord_x, grid_coord_y, grid_coord_z;

	delta_x = curr_cx_grid - prev_cx_grid;
	delta_y = curr_cy_grid - prev_cy_grid;
	delta_z = curr_cz_grid - prev_cz_grid;

	if (delta_x == 0 && delta_y == 0 && delta_z == 0)// static
		return true;
	if (abs(delta_x) >= BB_SIZE_X || abs(delta_y) >= BB_SIZE_Y || abs(delta_z) >= BB_SIZE_Z) // completely out of bb
	{
		memset(m_bb_data_ptr, GRID_STATUS_UNKNOW, BB_SIZE_XYZ * sizeof(bb_data_t));
		return true;
	}

	/* x planar */
	if (delta_x>0)
	{
		grid_coord_x = prev_cx_grid - BB_SIZE_HALF_X;
		for (i = 0; i<delta_x; i++)
		{
			bb_coord_x = (grid_coord_x + i)&BB_MASK_X;
			memset(m_bb_data_ptr + (bb_coord_x << BB_SIZE_ORDER_YZ), GRID_STATUS_UNKNOW, BB_SIZE_YZ * sizeof(bb_data_t));
		}
	}
	else
	{
		delta_x = -delta_x;
		grid_coord_x = prev_cx_grid + BB_SIZE_HALF_X;
		for (i = 0; i<delta_x; i++)
		{
			bb_coord_x = (grid_coord_x - i)&BB_MASK_X;
			memset(m_bb_data_ptr + (bb_coord_x << BB_SIZE_ORDER_YZ), GRID_STATUS_UNKNOW, BB_SIZE_YZ * sizeof(bb_data_t));
		}
	}

	/* y row */
	if (delta_y > 0)
	{
		grid_coord_y = prev_cy_grid - BB_SIZE_HALF_Y;
		for (bb_coord_x = 0; bb_coord_x<BB_SIZE_X; bb_coord_x++)
		{
			for (j = 0; j<delta_y; j++)
			{
				bb_coord_y = (grid_coord_y + j)&BB_MASK_Y;
				memset(m_bb_data_ptr + (bb_coord_x << BB_SIZE_ORDER_YZ) + (bb_coord_y << BB_SIZE_ORDER_Z), GRID_STATUS_UNKNOW, BB_SIZE_Z * sizeof(bb_data_t));
			}
		}
	}
	else
	{
		grid_coord_y = prev_cy_grid + BB_SIZE_HALF_Y;
		delta_y = -delta_y;
		for (bb_coord_x = 0; bb_coord_x<BB_SIZE_X; bb_coord_x++)
		{
			for (j = 0; j<delta_y; j++)
			{
				bb_coord_y = (grid_coord_y - j)&BB_MASK_Y;
				memset(m_bb_data_ptr + (bb_coord_x << BB_SIZE_ORDER_YZ) + (bb_coord_y << BB_SIZE_ORDER_Z), GRID_STATUS_UNKNOW, BB_SIZE_Z * sizeof(bb_data_t));
			}
		}
	}

	// z
	if (delta_z > 0)
	{
		grid_coord_z = prev_cz_grid - BB_SIZE_HALF_Z;
		for (bb_coord_x = 0; bb_coord_x<BB_SIZE_X; bb_coord_x++)
		{
			for (bb_coord_y = 0; bb_coord_y<BB_SIZE_Y; bb_coord_y++)
			{
				for (k = 0; k<delta_z; k++)
				{
					bb_coord_z = (grid_coord_z + k)&BB_MASK_Z;
					m_bb_data_ptr[(bb_coord_x << BB_SIZE_ORDER_YZ) + (bb_coord_y << BB_SIZE_ORDER_Z) + bb_coord_z] = GRID_STATUS_UNKNOW;
				}
			}
		}
	}
	else
	{
		grid_coord_z = prev_cz_grid + BB_SIZE_HALF_Z;
		delta_z = -delta_z;
		for (bb_coord_x = 0; bb_coord_x<BB_SIZE_X; bb_coord_x++)
		{
			for (bb_coord_y = 0; bb_coord_y<BB_SIZE_Y; bb_coord_y++)
			{
				for (k = 0; k<delta_z; k++)
				{
					bb_coord_z = (grid_coord_z - k)&BB_MASK_Z;
					m_bb_data_ptr[(bb_coord_x << BB_SIZE_ORDER_YZ) + (bb_coord_y << BB_SIZE_ORDER_Z) + bb_coord_z] = GRID_STATUS_UNKNOW;
				}
			}
		}
	}

	return true;
}

int mapper::ray_tracing(const int sx, const int sy, const int sz, const int ex, const int ey, const int ez, const int end_status)
{
	int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
	int point[3];

	point[0] = sx;
	point[1] = sy;
	point[2] = sz;
	dx = ex - sx;
	dy = ey - sy;
	dz = ez - sz;
	x_inc = (dx < 0) ? -1 : 1;
	l = abs(dx);
	y_inc = (dy < 0) ? -1 : 1;
	m = abs(dy);
	z_inc = (dz < 0) ? -1 : 1;
	n = abs(dz);
	dx2 = l << 1;
	dy2 = m << 1;
	dz2 = n << 1;

	if ((l >= m) && (l >= n)) 
	{
		err_1 = dy2 - l;
		err_2 = dz2 - l;
		for (i = 0; i < l; i++) 
		{
			int address = ((point[0] & BB_MASK_X) << BB_SIZE_ORDER_YZ) + ((point[1] & BB_MASK_Y) << BB_SIZE_ORDER_Z) + (point[2] & BB_MASK_Z);
			bb_data_t status = m_bb_data_ptr[address];
			if (status > -GRID_STATUS_THRESHOLD)
				m_bb_data_ptr[address] = m_bb_data_ptr[address] - 1;

			if (err_1 > 0) 
			{
				point[1] += y_inc;
				err_1 -= dx2;
			}
			if (err_2 > 0) 
			{
				point[2] += z_inc;
				err_2 -= dx2;
			}
			err_1 += dy2;
			err_2 += dz2;
			point[0] += x_inc;
		}
	}
	else if ((m >= l) && (m >= n)) 
	{
		err_1 = dx2 - m;
		err_2 = dz2 - m;
		for (i = 0; i < m; i++) {
			int address = ((point[0] & BB_MASK_X) << BB_SIZE_ORDER_YZ) + ((point[1] & BB_MASK_Y) << BB_SIZE_ORDER_Z) + (point[2] & BB_MASK_Z);
			bb_data_t status = m_bb_data_ptr[address];
			if (status > -GRID_STATUS_THRESHOLD)
				m_bb_data_ptr[address] = m_bb_data_ptr[address] - 1;

			if (err_1 > 0)
			{
				point[0] += x_inc;
				err_1 -= dy2;
			}
			if (err_2 > 0) 
			{
				point[2] += z_inc;
				err_2 -= dy2;
			}
			err_1 += dx2;
			err_2 += dz2;
			point[1] += y_inc;
		}
	}
	else 
	{
		err_1 = dy2 - n;
		err_2 = dx2 - n;
		for (i = 0; i < n; i++) 
		{
			int address = ((point[0] & BB_MASK_X) << BB_SIZE_ORDER_YZ) + ((point[1] & BB_MASK_Y) << BB_SIZE_ORDER_Z) + (point[2] & BB_MASK_Z);
			bb_data_t status = m_bb_data_ptr[address];
			if (status > -GRID_STATUS_THRESHOLD)
				m_bb_data_ptr[address] = m_bb_data_ptr[address] - 1;			
			
			if (err_1 > 0)
			{
				point[1] += y_inc;
				err_1 -= dz2;
			}
			if (err_2 > 0) 
			{
				point[0] += x_inc;
				err_2 -= dz2;
			}
			err_1 += dy2;
			err_2 += dx2;
			point[2] += z_inc;
		}
	}

	int address = ((point[0] & BB_MASK_X) << BB_SIZE_ORDER_YZ) + ((point[1] & BB_MASK_Y) << BB_SIZE_ORDER_Z) + (point[2] & BB_MASK_Z);
	bb_data_t status = m_bb_data_ptr[address];

	if (end_status == 1)
	{
		// occupied
		if (status < GRID_STATUS_THRESHOLD)
			m_bb_data_ptr[address] = m_bb_data_ptr[address] + 1;
	}
	else
	{
		// free
		if (status > -GRID_STATUS_THRESHOLD)
			m_bb_data_ptr[address] = m_bb_data_ptr[address] - 1;
	}

	return 0;
}

// to grid, from phsical world to roll buffer id
bool mapper::map_update(const std::vector<float> &pdepth, const vs_imu_data_t &pimu)
{
	/*if (pdepth == NULL || pimu == NULL)
		return false;*/

	float x_ground;
	float y_ground;
	float z_ground;
	/* uav cur status */
	x_ground = pimu.lati;
	y_ground = pimu.longti;
	z_ground = pimu.alti;

	/* world to grid */
	curr_cx_grid = world2grid(x_ground);
	curr_cy_grid = world2grid(y_ground);
	curr_cz_grid = world2grid(z_ground);

	/* be careful with the format !!! */
	float q[4];
	float r[9];
	q[0] = pimu.q1;
	q[1] = pimu.q2;
	q[2] = pimu.q3;
	q[3] = pimu.q0;
	quaternion_to_rotationMatrix(q, r);

	// update center, clear old, 
	if (m_init == false)
	{
		/* move uav in bb */
		update_bb_to_new_pose();
	}
	else
	{
		memset(m_bb_data_ptr, GRID_STATUS_UNKNOW, sizeof(bb_data_t)*BB_SIZE_XYZ);
	}

	memset(m_cur_bb_data_ptr, GRID_STATUS_UNKNOW, BB_SIZE_XYZ * sizeof(bb_data_t));

	m_lookthrough_tb_cnt = 0;
	float inverse_f = 1.0f / m_f;

	// triangulation, set look through table
	for (int v = 0; v<DEPTH_MAP_HEIGHT; v++)
	{
		float v1 = v - m_cv;
		for (int u = 0; u < DEPTH_MAP_WIDTH; u++)
		{
			float depth = pdepth[v*DEPTH_MAP_WIDTH + u];
			if (depth > MIN_DEPTH && depth < MAX_DEPTH)
			{
				float z_camera = depth; // 
				float z_camera_inverse_f = z_camera * inverse_f;
				float x_camera = z_camera_inverse_f * (u - m_cu);
				float y_camera = z_camera_inverse_f * v1;

				/* to imu */
				float x_imu = z_camera;
				float y_imu = x_camera;
				float z_imu = y_camera;

				/* to world */
				float x_world =
					r[0] * x_imu +
					r[1] * y_imu +
					r[2] * z_imu +
					x_ground;
				float y_world =
					r[3] * x_imu +
					r[4] * y_imu +
					r[5] * z_imu +
					y_ground;
				float z_world =
					r[6] * x_imu +
					r[7] * y_imu +
					r[8] * z_imu +
					z_ground;

				//std::cout << x_world << ";" << y_world << ";" << z_world;

				/* to grid */
				int x_grid = world2grid(x_world);
				int y_grid = world2grid(y_world);
				int z_grid = world2grid(z_world);

				/* delta */
				int delta_x_grid = x_grid - curr_cx_grid;
				int delta_y_grid = y_grid - curr_cy_grid;
				int delta_z_grid = z_grid - curr_cz_grid;
				int abs_delta_x_grid = abs(delta_x_grid);
				int abs_delta_y_grid = abs(delta_y_grid);
				int abs_delta_z_grid = abs(delta_z_grid);

				/* insider */
				if (abs_delta_x_grid <= BB_SIZE_HALF_X && abs_delta_y_grid <= BB_SIZE_HALF_Y && abs_delta_z_grid <= BB_SIZE_HALF_Z)
				{
					/* to bb */
					int address = ((x_grid & BB_MASK_X) << BB_SIZE_ORDER_YZ) + ((y_grid & BB_MASK_Y) << BB_SIZE_ORDER_Z) + (z_grid & BB_MASK_Z);
					bb_data_t status = m_cur_bb_data_ptr[address];
					if (status != GRID_STATUS_OCCUPIED)
					{
						int m_look_through_cnt4 = m_lookthrough_tb_cnt << 2;
						m_lookthrough_tb_ptr[m_look_through_cnt4] = x_grid;
						m_lookthrough_tb_ptr[m_look_through_cnt4 + 1] = y_grid;
						m_lookthrough_tb_ptr[m_look_through_cnt4 + 2] = z_grid;
						m_lookthrough_tb_ptr[m_look_through_cnt4 + 3] = 1; // occupied
						m_lookthrough_tb_cnt++;
						m_cur_bb_data_ptr[address] = GRID_STATUS_OCCUPIED;
					}
				}
				else
				{
					int delta_x_grid_boundary, delta_y_grid_boundary, delta_z_grid_boundary;
					int x_grid_boundary, y_grid_boundary, z_grid_boundary;
#undef CALC_DELTA_GRID_BOUNDARY
#define CALC_DELTA_GRID_BOUNDARY(axis0, axis1, axis2, major_axis) \
                    { \
                        if(delta_##axis0##_grid>0) \
                            delta_##axis0##_grid_boundary = BB_SIZE_HALF_##major_axis+1; \
                        else \
                            delta_##axis0##_grid_boundary = -BB_SIZE_HALF_##major_axis-1; \
                        delta_##axis1##_grid_boundary = (delta_##axis0##_grid_boundary*delta_##axis1##_grid)/delta_##axis0##_grid; \
                        delta_##axis2##_grid_boundary = (delta_##axis0##_grid_boundary*delta_##axis2##_grid)/delta_##axis0##_grid; \
                    }

					if (abs_delta_x_grid >= abs_delta_y_grid && abs_delta_x_grid >= abs_delta_z_grid)
						CALC_DELTA_GRID_BOUNDARY(x, y, z, X)
					else if (abs_delta_y_grid >= abs_delta_z_grid && abs_delta_y_grid >= abs_delta_x_grid)
						CALC_DELTA_GRID_BOUNDARY(y, z, x, Y)
					else
						CALC_DELTA_GRID_BOUNDARY(z, x, y, Z)
#undef CALC_DELTA_GRID_BOUNDARY

					// to grid
					x_grid_boundary = curr_cx_grid + delta_x_grid_boundary;
					y_grid_boundary = curr_cy_grid + delta_y_grid_boundary;
					z_grid_boundary = curr_cz_grid + delta_z_grid_boundary;

					// to bb
					int address = ((x_grid_boundary & BB_MASK_X) << BB_SIZE_ORDER_YZ) + ((y_grid_boundary & BB_MASK_Y) << BB_SIZE_ORDER_Z) + (z_grid_boundary & BB_MASK_Z);
					bb_data_t status = m_cur_bb_data_ptr[address];
					if (status != GRID_STATUS_OCCUPIED)
					{
						int m_look_through_cnt4 = m_lookthrough_tb_cnt << 2;
						m_lookthrough_tb_ptr[m_look_through_cnt4] = x_grid_boundary;
						m_lookthrough_tb_ptr[m_look_through_cnt4 + 1] = y_grid_boundary;
						m_lookthrough_tb_ptr[m_look_through_cnt4 + 2] = z_grid_boundary;
						m_lookthrough_tb_ptr[m_look_through_cnt4 + 3] = 0; // free
						m_lookthrough_tb_cnt++;
						m_cur_bb_data_ptr[address] = GRID_STATUS_OCCUPIED;
					}
				}
			}
		}
	}

	// bresenham ray-tracing
	for (int i = 0; i<m_lookthrough_tb_cnt; i++)
	{
		int m_look_through_cnt3 = (i << 2);
		ray_tracing(curr_cx_grid, curr_cy_grid, curr_cz_grid,
			m_lookthrough_tb_ptr[m_look_through_cnt3],
			m_lookthrough_tb_ptr[m_look_through_cnt3 + 1],
			m_lookthrough_tb_ptr[m_look_through_cnt3 + 2],
			m_lookthrough_tb_ptr[m_look_through_cnt3 + 3]);
	}

	if (m_init == true)
	{
		m_init = false;
	}

	/* cur status -> pre status */
	prev_cx_grid = curr_cx_grid;
	prev_cy_grid = curr_cy_grid;
	prev_cz_grid = curr_cz_grid;

	return true;
}

// map visulization  
bool mapper::mapVis(std::vector<point3d_t> &pcl)
{
	pcl.clear();
	for (int i = -BB_SIZE_HALF_X; i <= BB_SIZE_HALF_X; i++)
	{
		for (int j = -BB_SIZE_HALF_Y; j <= BB_SIZE_HALF_Y; j++)
		{
			for (int k = -BB_SIZE_HALF_Z; k <= BB_SIZE_HALF_Z; k++)
			{
				int x_grid_real, y_grid_real, z_grid_real;
				x_grid_real = i + curr_cx_grid;
				y_grid_real = j + curr_cy_grid;
				z_grid_real = k + curr_cz_grid;

				int address = ((x_grid_real & BB_MASK_X) << BB_SIZE_ORDER_YZ) + ((y_grid_real & BB_MASK_Y) << BB_SIZE_ORDER_Z) + (z_grid_real & BB_MASK_Z);
				if (m_bb_data_ptr[address] > 0)
				{
					point3d_t pt;
					pt.x = x_grid_real * m_grid_precision + m_grid_precision * 0.5;
					pt.y = y_grid_real * m_grid_precision + m_grid_precision * 0.5;
					pt.z = z_grid_real * m_grid_precision + m_grid_precision * 0.5;
					pcl.push_back(pt);
				}
			}
		}
	}
	return true;
}


unsigned int mappingThread(unsigned int tid, void* pData)
{
	threadMapping* pThreadParameter = reinterpret_cast<threadMapping*>(pData);
	SafeQueue<depth_t>& dqueue = pThreadParameter->depthBuf;
	SafeQueue<mapvis_t>& mqueue = pThreadParameter->mapBuf;
	depth_t currFrame;
	std::vector<point3d_t> pcl;
	mapvis_t mapvis;
	while (1)
	{
		// if exit request, then exit
		if (pThreadParameter->exit == 1)
			break;

		// if cosuming is to slow, then producer will wiat 
		if (dqueue.size() == 0)
		{
			Sleep(1000);
			continue;
		}

		// if either, read one file
		currFrame = dqueue.dequeue();

		std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
		pThreadParameter->mp.map_update(currFrame.data, currFrame.imu);
		std::chrono::time_point<std::chrono::system_clock> t2 = std::chrono::system_clock::now();
		std::chrono::duration<double> lapse = t2 - t1;
		std::cout << "mapping time: " << lapse.count() << endl;
		//
		pThreadParameter->mp.mapVis(pcl);

		mapvis.imu = currFrame.imu;
		mapvis.rawImage = currFrame.rawImage;
		mapvis.pcl = pcl;

		mqueue.enqueue(mapvis);

		Sleep(100);
	}

	return 0;
}


//-----------------------------------------------------------------------------
bool startMapping(threadMapping& handler)
//-----------------------------------------------------------------------------
{
	handler.mp.set_map_order(2);

	std::cout << "INFO: Launching mapping thread..." << std::endl;

	thread1 = std::thread(mappingThread, 0, &handler);

	return true;
}

bool stopMapping(threadMapping& handler)
{
	cout << "INFO: Terminating mapping thread..." << endl;

	if (thread1.joinable())
	{
		thread1.join();
	}

	cout << "INFO: mapping Threads terminated successfully!" << endl;

	return true;
}