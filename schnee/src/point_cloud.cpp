#include "point_cloud.h"
#include "nanoflann.hpp"

#include <eigen3/Eigen/Eigen>

void PC_build_planes(const PointCloud & pc, std::vector<sPlane> & planes, size_t k)
{
	assert(k > 1);

	// Create nearest neighbour tree
	typedef nanoflann::KDTreeSingleIndexAdaptor<
	        nanoflann::L2_Simple_Adaptor<float, PointCloud>,
	        PointCloud,
	        3
	        > sc_kd_tree;

	sc_kd_tree index(3, pc,
	                 nanoflann::KDTreeSingleIndexAdaptorParams(10));
	index.buildIndex();

	// Process tangents

	// Nbhd variables
	const size_t            num_results = k + 1; // We will find the query point in the neighbour set
    std::vector<size_t>     ret_index(num_results);
    std::vector<float>      out_squared_dist(num_results);
	float                   kd_query[3];
	size_t                  nbhd_count;
	size_t                  current_index;
	sVector3                current_neighbour;
	float                   xdist, ydist, zdist;

	Eigen::Matrix<float, 3, 3> covariance_matrix;


	for(int i = 0; i < pc.points.size(); ++i)
	{
		const Vector3 & p = *(pc.points[i].get());
		// Find k nearest points
        kd_query[0] = p.x;
        kd_query[1] = p.y;
        kd_query[2] = p.z;
		nbhd_count = index.knnSearch(&kd_query[0], num_results, &ret_index[0], &out_squared_dist[0]);

		assert(nbhd_count == num_results);
		assert(ret_index[0] == i);

		sPlane plane = std::make_shared<Plane>();

		// Calculate o, centroid
		for(int j = 1; j < num_results; ++j)
		{
			current_index = ret_index[j];
			current_neighbour = pc.points[current_index];
			if(j == 1)
				plane->center = std::make_shared<Vector3>(*(current_neighbour.get()));
			else
				*(plane->center.get()) += *(current_neighbour.get());
		}
		*(plane->center.get()) /= k;

		// Calculate n, normal
		covariance_matrix.setZero();
		for(int j = 1; j < num_results; ++j)
		{
			current_index = ret_index[j];
			current_neighbour = pc.points[current_index];
			xdist = current_neighbour->x - plane->center->x;
			ydist = current_neighbour->y - plane->center->y;
			zdist = current_neighbour->z - plane->center->z;

			covariance_matrix(1, 1) += ydist * ydist;
			covariance_matrix(1, 2) += ydist * zdist;
			covariance_matrix(2, 2) += zdist * zdist;

			xdist *= xdist;
			ydist *= xdist;
			zdist *= xdist;

			covariance_matrix(0, 0) += xdist;
			covariance_matrix(0, 1) += ydist;
			covariance_matrix(0, 2) += zdist;
		}


		plane->normal = std::make_shared<Vector3>(Vector3::zup());
		planes.push_back(plane);
	}

	return;
}
