#ifndef __POINT_CLOUD__
#define __POINT_CLOUD__

#include "vector.h"
#include "plane.h"

#include <nanoflann.hpp>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>

struct PointCloud
{
	std::vector<sVector3> points;

	inline size_t kdtree_get_point_count() const { return points.size(); }

	inline float kdtree_get_pt(const size_t idx, int dim) const
	{
		if(dim == 0)
			return points[idx]->x;
		else if(dim == 1)
			return points[idx]->y;
		else
			return points[idx]->z;
	}

	template<class BBOX>
	bool kdtree_get_bbox(BBOX&) const { return false; }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud>,
    PointCloud,
    3
> point_cloud_index;

struct PlaneCloud
{
	std::vector<sPlane> planes;

	inline size_t kdtree_get_point_count() const { return planes.size(); }

	inline float kdtree_get_pt(const size_t idx, int dim) const
	{
		if(dim == 0)
			return planes[idx]->center->x;
		else if(dim == 1)
			return planes[idx]->center->y;
		else
			return planes[idx]->center->z;
	}

	template<class BBOX>
	bool kdtree_get_bbox(BBOX&) const { return false; }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PlaneCloud>,
    PlaneCloud,
    3
> plane_cloud_index;


void PC_build_planes(const PointCloud &, std::vector<sPlane> &, size_t);

void PC_compute_surface_from_covaraince(
        Eigen::EigenSolver<Eigen::Matrix3f> &,
        const Eigen::Matrix3f &,
        Plane&);

#endif
