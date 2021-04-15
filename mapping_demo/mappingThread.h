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
#include "config.h"

#define MIN_DEPTH		0
#define MAX_DEPTH		100
#define DEPTH_MAP_WIDTH		IWIDTH
#define DEPTH_MAP_HEIGHT	IHEIGHT

#define GRID_STATUS_THRESHOLD	1

#define GRID_STATUS_UNKNOW		0
#define GRID_STATUS_OCCUPIED	GRID_STATUS_THRESHOLD
#define GRID_STATUS_FREE		-GRID_STATUS_THRESHOLD

#define BB_SIZE_ORDER		7
#define BB_SIZE_ORDER_X		BB_SIZE_ORDER
#define BB_SIZE_ORDER_Y		BB_SIZE_ORDER
#define BB_SIZE_ORDER_Z		BB_SIZE_ORDER
#define BB_SIZE_X		(1<<BB_SIZE_ORDER_X)
#define BB_SIZE_Y		(1<<BB_SIZE_ORDER_Y)
#define BB_SIZE_Z		(1<<BB_SIZE_ORDER_Z)
#define BB_MASK_X		(BB_SIZE_X-1)
#define BB_MASK_Y		(BB_SIZE_Y-1)
#define BB_MASK_Z		(BB_SIZE_Z-1)

#define BB_SIZE_ORDER_YZ (BB_SIZE_ORDER_Y+BB_SIZE_ORDER_Z)

#define BB_SIZE_YZ		(BB_SIZE_Y*BB_SIZE_Z)
#define BB_SIZE_XYZ		(BB_SIZE_X*BB_SIZE_Y*BB_SIZE_Z)

#define BB_SIZE_HALF_X	((BB_SIZE_X>>1) - 1 -1)
#define BB_SIZE_HALF_Y	((BB_SIZE_Y>>1) - 1 -1)
#define BB_SIZE_HALF_Z	((BB_SIZE_Z>>1) - 1 -1)

enum map_status {
	MAP_STATUS_FREE	=	0,
	MAP_STATUS_OCUUPIED,
	MAP_STATUS_UNKNOWN,
};

typedef char bb_data_t;

class mapper {
public:
	mapper() {
		m_bb_data_ptr = NULL;
		//m_pre_bb_data_ptr = NULL;
		m_cur_bb_data_ptr = NULL;
		m_lookthrough_tb_ptr = NULL;

		map_init(2);
	}

	~mapper() {
		map_release();
	}

	// map update
	bool map_update(const std::vector<float> &pdepth, const vs_imu_data_t &pimu);

	bool set_map_order(int order)
	{
		m_grid_precision_order = order;
		m_grid_precision = 1. / (1 << m_grid_precision_order);
		m_grid_coord_multipler = (1 << m_grid_precision_order);

		return true;
	}

	bool mapVis(std::vector<point3d_t> &pcl);

private:
	template <typename T>
	inline int world2grid(const T &world_coord)
	{
		// transform from world coordinate to grid coordinate
		return (world_coord < 0 ? (((int)(world_coord*m_grid_coord_multipler)) - 1) : ((int)(world_coord*m_grid_coord_multipler)));
	}

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

	// move bb center to new center and clear some old status
	bool update_bb_to_new_pose();
	// ray-tracing 
	int ray_tracing(const int sx, const int sy, const int sz, const int ex, const int ey, const int ez, const int end_status);
	// map init
	bool map_init(int order);
	// map release
	bool map_release();

public:
	int prev_cx_grid, prev_cy_grid, prev_cz_grid;
	int curr_cx_grid, curr_cy_grid, curr_cz_grid;

	bb_data_t *m_bb_data_ptr;
	//bb_data_t *m_pre_bb_data_ptr;
	bb_data_t *m_cur_bb_data_ptr;
	int		  *m_lookthrough_tb_ptr;
	int		   m_lookthrough_tb_cnt;

	bool	   m_init;

	float	   m_f, m_cu, m_cv;

private:
	unsigned int m_grid_precision_order;
	float		 m_grid_precision;
	float		 m_grid_coord_multipler;
};

struct threadMapping
{
	class mapper mp;
	SafeQueue<depth_t> &depthBuf;
	SafeQueue<mapvis_t> &mapBuf;
	int &exit;
	threadMapping(SafeQueue<depth_t> &_depthBuf, int &_exit, SafeQueue<mapvis_t> &_mapbuf) : depthBuf(_depthBuf), exit(_exit),
		mapBuf(_mapbuf) {}
	~threadMapping() {}
};

bool startMapping(threadMapping &handler);
bool stopMapping(threadMapping &handler);