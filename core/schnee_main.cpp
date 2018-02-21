#include "vector.h"
#include "plane.h"
#include "file_loader.h"
#include "file_saver.h"
#include "cloud.h"
#include "orientationfixer.h"
#include "marching_cubes.h"

#include <iostream>
#include <string>
#include <vector>
#include <cassert>
#include <cmath>
#include <deque>

void dump_mem_usage()
{
	FILE* f = fopen("/proc/self/statm","rt");
	if (!f) return;
	char str[300];
	size_t n = fread(str, 1, 200, f);
	str[n] = 0;
	printf("MEM: %s\n", str);
	fclose(f);
}


int main(int argc, const char * argv[])
{
	if(argc < 6)
	{
		std::cout << "USAGE:\n";
		std::cout << argv[0] << " off_input_file off_out_file k density noise" << std::endl;
		exit(2);
	}

	// In off file
	std::string pin = argv[1];
	std::string pout = argv[2];
	int k = std::stoi(argv[3]);
	float denity = std::stof(argv[4]);
	float noise = std::stof(argv[5]);
	assert(k > 1);
    assert(density > 0.0f);
	assert(noise >= 0.0f);
	std::cout << "IN FILE: " << pin << "\n";
	std::cout << "OUT FILE: " << pout << "\n";
	std::cout << "K: " << k << std::endl;
    std::cout << "DENSITY: " << density << std::endl;
	std::cout << "NOISE: " << noise << std::endl;

	// Create empty point cloud
	PointCloud pc;

	// Get points
	if(!FL_OFF_load_points(pin, pc.points))
		exit(3);

	// Buil planes
	std::vector<sPlane> planes;
	PTC_build_planes(pc, planes, k);

	// Fix planes orientation
	PlaneCloud plc;
	plc.planes = planes;
	plane_cloud_index index(3, plc, nanoflann::KDTreeSingleIndexAdaptorParams(10));
	index.buildIndex();

    orientationFixer(plc,index,k);

	// marching cubes
	std::vector<sGrid> grids;
	Vector3 bbox_min, bbox_max;
	PLC_get_bounds(plc,
	               bbox_min.x, bbox_min.y, bbox_min.z,
	               bbox_max.x, bbox_max.y, bbox_max.z);
	sGrid grid = std::make_shared<Grid>(bbox_min, bbox_max, density);
	grid->create_cells();

	// Calculate signed distance function
	std::vector<sCellPoint> cell_corners;
	grid->getUniquePoints(cell_corners);
	MC_compute_signed_distance(cell_corners, plc, index, density, noise);

	// Debug
	FS_OFF_save_planes("/tmp/out.planes.faces.off", planes, 0.05f);
	FS_OFF_save_planes_normals("/tmp/out.planes.normals.off", planes, 5, 0.06f);
	FS_OFF_save_cell_points("/tmp/out.cells.values.off", cell_corners);
	//FS_OFF_save_grid_distances("/tmp/out.grid.distances.off", corners, distances);
	FS_OFF_save_cells_position("/tmp/out.grid.corners.off", grid->cells());

	return 0;
}

