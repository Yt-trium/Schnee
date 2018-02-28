#include "cloud.h"

#include <iostream>

void PTC_build_planes(const PointCloud & pc, std::vector<sPlane> & planes, size_t k)
{
	assert(k > 1);

	// Create nearest neighbour tree
	point_cloud_index index(3, pc,
	                 nanoflann::KDTreeSingleIndexAdaptorParams(10));
	index.buildIndex();


	// Nbhd variables
	const size_t            num_results = k + 1; // We will find the query point in the neighbour set
    std::vector<size_t>     ret_index(num_results);
    std::vector<float>      out_squared_dist(num_results);
	float                   kd_query[3];
	size_t                  nbhd_count;
	size_t                  current_index;
	sVector3                current_neighbour;
	float                   xdist, ydist, zdist;

	// CV
	Eigen::Matrix3f covariance_matrix;
	// Eigen vectors solver
    Eigen::EigenSolver<Eigen::Matrix3f> eigensolver;


	// Process tangents
	for(int i = 0; i < pc.points.size(); ++i)
	{
		const Vector3 & p = *(pc.points[i].get());
		// Find k nearest points
        kd_query[0] = p.x;
        kd_query[1] = p.y;
        kd_query[2] = p.z;
		nbhd_count = index.knnSearch(&kd_query[0], num_results, &ret_index[0], &out_squared_dist[0]);

		assert(nbhd_count == num_results);

		sPlane plane = std::make_shared<Plane>();

		// Calculate o, centroid
		for(int j = 0; j < num_results; ++j)
		{
			current_index = ret_index[j];
			if(current_index == i) continue;
			current_neighbour = pc.points[current_index];
			if(!plane->center)
				plane->center = std::make_shared<Vector3>(*(current_neighbour.get()));
			else
				*(plane->center.get()) += *(current_neighbour.get());
		}
		*(plane->center.get()) /= k;

		// Calculate n, normal
		// From the paper
		// Symetric matrix, do halg only
		covariance_matrix.setZero();
		for(int j = 1; j < num_results; ++j)
		{
			current_neighbour = pc.points[ret_index[j]];
			// OY : y - center
			Vector3 oy = *(current_neighbour.get()) - *(plane->center);

			for(int i = 0; i < 3; ++i)
			{
				float a = oy[i];
				for(int j = i; j < 3; ++j)
				{
					float b = oy[j];
					covariance_matrix(i, j) += a * b;
				}
			}
		}
		// Transpose
        covariance_matrix (1, 0) = covariance_matrix (0, 1);
        covariance_matrix (2, 0) = covariance_matrix (0, 2);
        covariance_matrix (2, 1) = covariance_matrix (1, 2);

		// Solve normal
		PC_compute_surface_from_covaraince(eigensolver, covariance_matrix, *(plane.get()));

		planes.push_back(plane);
	}
}

void PC_compute_surface_from_covaraince(
        Eigen::EigenSolver<Eigen::Matrix3f> & solver,
        const Eigen::Matrix3f & covariance,
        Plane& output)
{
		solver.compute(covariance);
		assert(solver.info() == Eigen::Success);
        //std::cout << "Eigen values: " << solver.eigenvalues() << "\n";
        //std::cout << "Eigen vectors: \n" << solver.pseudoEigenvectors() << std::endl;
		//std::cout << "Eigen vectors: \n" << solver.pseudoEigenvectors().col(2) << std::endl;
		const float & lamb1 = solver.eigenvalues()(0).real();
		const float & lamb2 = solver.eigenvalues()(1).real();
		const float & lamb3 = solver.eigenvalues()(2).real();
		int ncol = 0, ucol = 1;
        //std::cout << "lamb1: " << lamb1 << "\n";
        //std::cout << "lamb2: " << lamb2 << "\n";
        //std::cout << "lamb3: " << lamb3 << "\n";
		if(lamb1 <= lamb2 && lamb1 <= lamb3)
			ncol = 0;
		else if(lamb2 <= lamb1 && lamb2 <= lamb3)
			ncol = 1;
		else
			ncol = 2;

		ucol = (ncol + 1) % 3;

		auto u = solver.pseudoEigenvectors().col(ucol);
		auto n = solver.pseudoEigenvectors().col(ncol);
		output.normal = std::make_shared<Vector3>(n(0), n(1), n(2));
        //std::cout << "Normal: " << *(output.normal.get()) << "\n";
		output.u = std::make_shared<Vector3>(u(0), u(1), u(2));
		output.v = std::make_shared<Vector3>(Vector3::cross(*(output.u), *(output.normal)));
}

void PLC_get_bounds(const PlaneCloud & plc,
                    float & xmin, float & ymin, float & zmin,
                    float & xmax, float & ymax, float & zmax)
{
	const std::vector<sPlane> planes = plc.planes;
	sVector3 current = planes[0]->center;

	xmin = current->x;
	ymin = current->y;
	zmin = current->z;
	xmax = current->x;
	ymax = current->y;
	zmax = current->z;

	for(int i = 1; i < planes.size(); ++i)
	{
		current = planes[i]->center;

		if(current->x < xmin) xmin = current->x;
		else if(current->x > xmax) xmax = current->x;

		if(current->y < ymin) ymin = current->y;
		else if(current->y > ymax) ymax = current->y;

		if(current->z < zmin) zmin = current->z;
		else if(current->z > zmax) zmax = current->z;
	}
}
