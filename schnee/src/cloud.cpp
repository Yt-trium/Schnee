#include "cloud.h"

#include <iostream>

void PTC_build_planes(const PointCloud & pc, std::vector<sPlane> & planes, size_t k)
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
		if(lamb1 <= lamb2 && lamb2 <= lamb3)
		{
			ncol = 0;
			ucol = 1;
			vcol = 2;
		}
		else if(lamb1 <= lamb3 && lamb3 <= lamb2)
		{
			ncol = 0;
			ucol = 2;
			vcol = 1;
		}
		else if(lamb2 <= lamb3 && lamb3 <= lamb1)
		{
			ncol = 1;
			ucol = 2;
			vcol = 0;
		}
		else if(lamb2 <= lamb1 && lamb1 <= lamb3)
		{
			ncol = 1;
			ucol = 0;
			vcol = 2;
		}
		else if(lamb3 <= lamb1 && lamb1 <= lamb2)
		{
			ncol = 2;
			ucol = 0;
			vcol = 1;
		}
		else if(lamb3 <= lamb2 && lamb2 <= lamb1)
		{
			ncol = 2;
			ucol = 1;
			vcol = 0;
		}

		auto u = solver.pseudoEigenvectors().col(ucol);
		auto v = solver.pseudoEigenvectors().col(vcol);
		auto n = solver.pseudoEigenvectors().col(ncol);
		output.normal = std::make_shared<Vector3>(n(0), n(1), n(2));
		std::cout << "Normal: " << *(output.normal.get()) << "\n";
		output.u = std::make_shared<Vector3>(u(0), u(1), u(2));
		output.v = std::make_shared<Vector3>(Vector3::cross(*(output.normal.get()), *(output.u.get())));
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

void PLC_compute_signed_distances(const PlaneCloud & plc,
                                  const plane_cloud_index & cloud,
                                  std::vector<float> & distances,
                                  const Vector3 & bboxmin, const Vector3 & bboxmax,
                                  float csize,
                                  size_t count_x, size_t count_y, size_t count_z)
{
	assert(distances.size() == 0);
	float nb_cell_x = count_x,
	        nb_cell_y = count_y,
	        nb_cell_z = count_z;
	float nb_cell_xy = nb_cell_x * nb_cell_y;
	float compute;
	size_t index;

	distances.resize(count_x * count_y * count_z);

	assert(bboxmin.x + csize * nb_cell_x >= bboxmax.x);
	assert(bboxmin.y + csize * nb_cell_y >= bboxmax.y);
	assert(bboxmin.z + csize * nb_cell_z >= bboxmax.z);

	// Nbhd variables
	const size_t            num_results = 1;
    std::vector<size_t>     ret_index(num_results);
    std::vector<float>      out_squared_dist(num_results);
	float                   kd_query[3];
	size_t                  nbhd_count;
	//sVector3                current_neighbour;
	Vector3 cell_pos;

	for(float z = 0; z < nb_cell_z; z+=1)
	{
        kd_query[2] = cell_pos.z = bboxmin.z + z * csize;
        for(float y = 0; y < nb_cell_y; y+=1)
        {
            kd_query[1] = cell_pos.y = bboxmin.y + y * csize;
            for(float x = 0; x < nb_cell_x; x+=1)
            {
                kd_query[0] = cell_pos.z = bboxmin.x + x * csize;
				index = z * (nb_cell_xy) + y * nb_cell_x + x;

				// Find closest
                nbhd_count = cloud.knnSearch(&kd_query[0], num_results, &ret_index[0], &out_squared_dist[0]);
                assert(nbhd_count == num_results);

                const sPlane & current_neighbour = plc.planes[ret_index[0]];
				const Plane & current_plane = *(current_neighbour.get());
				const Vector3 & current_center = *(current_plane.center.get());
				const Vector3 & current_normal = *(current_plane.normal.get());

				// Compute distance
				compute = Vector3::dot(cell_pos - current_center, current_normal);
				distances[index] = compute;
            }
        }
	}

}
