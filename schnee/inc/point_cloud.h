#ifndef __POINT_CLOUD__
#define __POINT_CLOUD__

#include "vector.h"
#include "plane.h"

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

void PC_build_planes(const PointCloud &, std::vector<sPlane> &, size_t);

void PC_compute_normal(Eigen::EigenSolver<Eigen::Matrix3f> &, const Eigen::Matrix3f &, Vector3 &);

#endif