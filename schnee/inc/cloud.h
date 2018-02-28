#ifndef __CLOUD__
#define __CLOUD__

#include "vector.h"
#include "plane.h"

#include <nanoflann.hpp>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>

/**
 * @brief A 3D Cloud of point
 * Structure adapted for the kd-tree
 */
struct PointCloud
{
	std::vector<sVector3> points;

	/**
	 * @brief Size accessor
	 * @return
	 */
	inline size_t kdtree_get_point_count() const { return points.size(); }

	/**
	 * @brief Return value of given point and given dimension (x: 0, y: 1, z: 2)
	 * @param idx
	 * @param dim
	 * @return
	 */
	inline float kdtree_get_pt(const size_t idx, int dim) const
	{
		if(dim == 0)
			return points[idx]->x;
		else if(dim == 1)
			return points[idx]->y;
		else
			return points[idx]->z;
	}

	/**
	 * Not Used
	 */
	template<class BBOX>
	bool kdtree_get_bbox(BBOX&) const { return false; }
};

/**
 * @brief Point cloud Tree
 */
typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud>,
    PointCloud,
    3
> point_cloud_index;

/**
 * @brief A 3D Cloud of plane
 * Adapted for the kd-tree
 */
struct PlaneCloud
{
	std::vector<sPlane> planes;

	/**
	 * @brief Number of plane in the cloud
	 * @return
	 */
	inline size_t kdtree_get_point_count() const { return planes.size(); }

	/**
	 * @brief Acess the given plane center dimension (x: 0, y: 1, z: 2)
	 * @param idx
	 * @param dim
	 * @return
	 */
	inline float kdtree_get_pt(const size_t idx, int dim) const
	{
		if(dim == 0)
			return planes[idx]->center->x;
		else if(dim == 1)
			return planes[idx]->center->y;
		else
			return planes[idx]->center->z;
	}

	/**
	 * Not Used
	 */
	template<class BBOX>
	bool kdtree_get_bbox(BBOX&) const { return false; }
};

/**
 * @brief Plane cloud Tree
 */
typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PlaneCloud>,
    PlaneCloud,
    3
> plane_cloud_index;


/**
 * @brief Create oriented planes from a set of points.
 * @param The points
 * @param The output planes
 * @param The number of neighbour to take for each plane computation, K
 */
void PTC_build_planes(const PointCloud &, std::vector<sPlane> &, size_t);

/**
 * @brief Calculate the min and max (x, y, z) of a plane cloud.
 * @param The cloud
 * @param out min x
 * @param out min y
 * @param out min z
 * @param out max x
 * @param out max y
 * @param out max z
 */
void PLC_get_bounds(const PlaneCloud&, float &, float &, float &, float &, float &, float &);

/**
 * @brief Extract the normal from a covariance matrix usign eigen vectors.
 * @param The eigen solver
 * @param The covariance matrix
 * @param out writer
 */
void PC_compute_surface_from_covaraince(
        Eigen::EigenSolver<Eigen::Matrix3f> &,
        const Eigen::Matrix3f &,
        Plane&);

#endif
