#include "marching_cubes.h"

#include "file_saver.h"

void MC_create_cubes(const PlaneCloud & plc,
                     const plane_cloud_index & index,
                     std::vector<sGrid> & grids, float cell_size)
{
	// Create only cubes intersecting the mesh

	// Get bounding box
	Vector3 bbox_min;
	Vector3 bbox_max;
	PLC_get_bounds(plc,
	               bbox_min.x, bbox_min.y, bbox_min.z,
	               bbox_max.x, bbox_max.y, bbox_max.z);
	std::cout << "BBOX MIN " << bbox_min << "\n";
	std::cout << "BBOX MAX " << bbox_max << "\n";

	// Calculate "cells"
	float nb_cell_x = std::ceil((bbox_max.x - bbox_min.x ) / cell_size + 1);
	float nb_cell_y = std::ceil((bbox_max.y - bbox_min.y ) / cell_size + 1);
	float nb_cell_z = std::ceil((bbox_max.z - bbox_min.z ) / cell_size + 1);
	std::cout << "NBCELLS : " << nb_cell_x << " / " << nb_cell_y << " / " << nb_cell_z << "\n";

	// Debug "cells"
	std::vector<sVector3> corners(nb_cell_x * nb_cell_y * nb_cell_z);
	std::vector<sVector3> cell_centers;

	Vector3 cell_center;
	assert(bbox_min.x + cell_size * nb_cell_x >= bbox_max.x);
	assert(bbox_min.y + cell_size * nb_cell_y >= bbox_max.y);
	assert(bbox_min.z + cell_size * nb_cell_z >= bbox_max.z);
	std::cout << "TOTAL NB CELLS: " << corners.size() << "\n";
	std::cout << "TOTAL NB CELLS2: " <<
	             (nb_cell_z - 1) * (nb_cell_x * nb_cell_y) + (nb_cell_y - 1) * nb_cell_x + nb_cell_x << "\n";
	std::cout << std::endl;
	assert(corners.size() ==
	             (nb_cell_z - 1) * (nb_cell_x * nb_cell_y) + (nb_cell_y - 1) * nb_cell_x + nb_cell_x);

	float hcsize = cell_size * 0.5f;

	Vector3 directions[8] = {
	    Vector3(1, 1, 1) * hcsize,
	    Vector3(1, 1, -1) * hcsize,
	    Vector3(1, -1, 1) * hcsize,
	    Vector3(1, -1, -1) * hcsize,
	    Vector3(-1, 1, 1) * hcsize,
	    Vector3(-1, 1, -1) * hcsize,
	    Vector3(-1, -1, 1) * hcsize,
	    Vector3(-1, -1, -1) * hcsize
	};
	const size_t            num_results = 1;
    std::vector<size_t>     ret_index(num_results);
    std::vector<float>      out_squared_dist(num_results);
	size_t                  nbhd_count;
	float                 kd_query[3] =
	    {cell_center.x, cell_center.y, cell_center.z};

	for(float z = 0; z < nb_cell_z; z+=1)
	{
        cell_center.z = kd_query[2] = bbox_min.z + z * cell_size;
        for(float y = 0; y < nb_cell_y; y+=1)
        {
			cell_center.y = kd_query[1] = bbox_min.y + y * cell_size;
            for(float x = 0; x < nb_cell_x; x+=1)
            {
                cell_center.x = kd_query[0] = bbox_min.x + x * cell_size;
                corners[z * (nb_cell_x * nb_cell_y) + y * nb_cell_x + x] = std::make_shared<Vector3>(cell_center);

				// Test if any point in cell
                nbhd_count = index.knnSearch(&kd_query[0], num_results, &ret_index[0], &out_squared_dist[0]);
                assert(nbhd_count == num_results);


				sGrid grid = std::make_shared<Grid>();
				for(int i = 0; i < 8; i++)
                    grid->corners.emplace_back(new Vector3(cell_center + directions[i]));
				grids.push_back(grid);
            }
        }
	}

    FS_OFF_save_points("/tmp/out.cells.corners.off", corners);

}

bool MC_is_point_in_cell(const Vector3 & point, const Vector3 & cell, const float & radius)
{
	return point <= cell + radius && point >= cell - radius;
}
