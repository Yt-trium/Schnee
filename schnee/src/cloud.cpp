#include "cloud.h"

#include <iostream>

void PC_build_planes(const PointCloud & pc, std::vector<sPlane> & planes, size_t k)
{
	assert(k > 1);

	// Create nearest neighbour tree
	point_cloud_index index(3, pc,
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

	Eigen::Matrix3f covariance_matrix;
    Eigen::EigenSolver<Eigen::Matrix3f> eigensolver;


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
        covariance_matrix (1, 0) = covariance_matrix (0, 1);
        covariance_matrix (2, 0) = covariance_matrix (0, 2);
        covariance_matrix (2, 1) = covariance_matrix (1, 2);

		// Solve normal
		PC_compute_surface_from_covaraince(eigensolver, covariance_matrix, *(plane.get()));

		planes.push_back(plane);
	}

	return;
}

void PC_compute_surface_from_covaraince(
        Eigen::EigenSolver<Eigen::Matrix3f> & solver,
        const Eigen::Matrix3f & covariance,
        Plane& output)
{
		solver.compute(covariance);
		assert(solver.info() == Eigen::Success);
        std::cout << "Eigen values: " << solver.eigenvalues() << "\n";
		std::cout << "Eigen vectors: \n" << solver.pseudoEigenvectors() << std::endl;
		//std::cout << "Eigen vectors: \n" << solver.pseudoEigenvectors().col(2) << std::endl;
		const float & lamb1 = solver.eigenvalues()(0).real();
		const float & lamb2 = solver.eigenvalues()(1).real();
		const float & lamb3 = solver.eigenvalues()(2).real();
		int ncol = 0, ucol = 1, vcol = 2;
		std::cout << "lamb1: " << lamb1 << "\n";
		std::cout << "lamb2: " << lamb2 << "\n";
		std::cout << "lamb3: " << lamb3 << "\n";
		if(lamb1 <= lamb2 && lamb1 <= lamb3)
		{
			ncol = 0;
			if(lamb2 <= lamb3)
			{
                ucol = 1;
                vcol = 2;
			}
			else
			{
                ucol = 2;
                vcol = 1;
			}
		}
		else if(lamb2 <= lamb3 && lamb2 <= lamb1)
		{
			ncol = 1;
			if(lamb3 <= lamb1)
			{
				ucol = 2;
				vcol = 0;
			}
			else
			{
                ucol = 0;
                vcol = 2;
			}
		}
		else
		{
			assert(lamb3 <= lamb1);
			assert(lamb3 <= lamb2);
			ncol = 2;
			if(lamb1 <= lamb2)
			{
				ucol = 0;
				vcol = 1;
			}
			else
			{
                ucol = 1;
                vcol = 0;
			}

		}

		auto u = solver.pseudoEigenvectors().col(ucol);
		auto v = solver.pseudoEigenvectors().col(vcol);
		auto n = solver.pseudoEigenvectors().col(ncol);
		output.normal = std::make_shared<Vector3>(n(0), n(1), n(2));
		output.u = std::make_shared<Vector3>(u(0), u(1), u(2));
		output.v = std::make_shared<Vector3>(v(0), v(1), v(2));
}
