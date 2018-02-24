#include "cloud.h"
#include "file_loader.h"
#include "file_saver.h"
#include "marching_cubes.h"
#include "mesh.h"
#include "orientationfixer.h"
#include "plane.h"
#include "vector.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <ctime>
#include <deque>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, const char * argv[])
{
    if(argc < 3)
	{
		std::cout << "USAGE:\n";
        std::cout << argv[0] << " off_input_file off_out_file k density noise iso_level" << "\n";
        std::cout << "by default: k=8, density=mesh size/point count, noise=0, iso_level=0" << std::endl;
		exit(2);
	}
	int start_s=clock();

	// In off file
	std::string pin = argv[1];
	std::string pout = argv[2];
    int k = 8;
    float density = -1.0f, noise = 0.0f, isolevel = 0.0f;

    if(argc > 3) k = std::stoi(argv[3]);
    if(argc > 4) density = std::stof(argv[4]);
    if(argc > 5) noise = std::stof(argv[5]);
	if(argc > 6) isolevel = std::stof(argv[6]);
	assert(k > 1);
	assert(noise >= 0.0f);
	assert(isolevel >= 0.0f);
	std::cout << "IN FILE: " << pin << "\n";
	std::cout << "OUT FILE: " << pout << "\n";
    std::cout << "K: " << k << std::endl;
    std::cout << "NOISE: " << noise << std::endl;
    std::cout << "ISO LEVEL: " << isolevel << std::endl;

	// Create empty point cloud
	PointCloud pc;

	// Get points
	int start_loading_file = clock();
	if(!FL_OFF_load_points(pin, pc.points))
		exit(3);
	int end_loading_file = clock();

    assert(pc.points.size() > 0);


	// Buil planes
	std::vector<sPlane> planes;
	int start_building_planes = clock();
	PTC_build_planes(pc, planes, k);
	int end_building_planes = clock();

	// Fix planes orientation
	PlaneCloud plc;
	plc.planes = planes;
	int start_kd_planes = clock();
	plane_cloud_index index(3, plc, nanoflann::KDTreeSingleIndexAdaptorParams(20));
	index.buildIndex();
	int end_kd_planes = clock();

	int start_mst = clock();
    orientationFixer(plc,index,k);
	int end_mst = clock();

	// marching cubes
	std::vector<sGrid> grids;
	Vector3 bbox_min, bbox_max;
	PLC_get_bounds(plc,
	               bbox_min.x, bbox_min.y, bbox_min.z,
	               bbox_max.x, bbox_max.y, bbox_max.z);
    Vector3 size = bbox_max - bbox_min;
    // Calcul density if undefined
    if(density == -1.0f)
    {
        density = (8 * size.x * size.y * size.z) / (float) planes.size();
    }
    std::cout << "DENSITY: " << density << std::endl;
    std::cout << "POINT COUNT: " << planes.size() << std::endl;
    assert(density > 0.0f);
    sGrid grid = std::make_shared<Grid>(bbox_min, bbox_max, density);
    std::cout << "GRID SIZE: " << grid->sizeX() << "x" << grid->sizeY() << "x" <<
                 grid->sizeZ() << " -> " << grid->sizeX() * grid->sizeY() * grid->sizeZ() << std::endl;
    std::cout << "GRID REAL SIZE: " << size << std::endl;
	int start_grid_cells = clock();
    grid->create_cells();
	int end_grid_cells = clock();

	// Calculate signed distance function
	std::vector<sCellPoint> cell_corners = grid->uniquePoints();
	MC_compute_signed_distance(cell_corners, plc, index, density, noise);
	FS_OFF_save_cell_points("/tmp/out.cells.values.off", cell_corners, isolevel);

	// Compute marching cubes
	mesh::Mesh generated_mesh;
	int start_compute_mesh = clock();
	grid->compute_mesh(plc, index, density, noise, isolevel, generated_mesh);
	int end_compute_mesh = clock();

	// Debug
	FS_OFF_save_planes("/tmp/out.planes.faces.off", planes, 0.05f);
	FS_OFF_save_planes_normals("/tmp/out.planes.normals.off", planes, 9, 0.09f);
	//FS_OFF_save_grid_distances("/tmp/out.grid.distances.off", corners, distances);

	// Export
	FS_OFF_save_mesh(pout, generated_mesh);

	int stop_s=clock();
	std::cout << "\n";
	std::cout << "READ FILE EXECUTION TIME: " << '\t' << (end_loading_file-start_loading_file)/double(CLOCKS_PER_SEC) << " seconds" << std::endl;
	std::cout << "BUILD PLANES EXECUTION TIME: " << '\t'<< (end_building_planes-start_building_planes)/double(CLOCKS_PER_SEC) << " seconds" << std::endl;
	std::cout << "BUILD KD TREE PLANES EXECUTION TIME: " << '\t' << (end_kd_planes-start_kd_planes)/double(CLOCKS_PER_SEC) << " seconds" << std::endl;
	std::cout << "FIXING NORMALS ORIENTATION EXECUTION TIME: " << '\t' << (end_mst-start_mst)/double(CLOCKS_PER_SEC) << " seconds" << std::endl;
	std::cout << "CREATING CELLS EXECUTION TIME: " << '\t' << (end_grid_cells-start_grid_cells)/double(CLOCKS_PER_SEC) << " seconds" << std::endl;
	std::cout << "COMPUTE MESH EXECUTION TIME: " << '\t' << (end_grid_cells-start_grid_cells)/double(CLOCKS_PER_SEC) << " seconds" << std::endl;
	std::cout << "\n";
	std::cout << "TOTAL EXECUTION TIME: " << '\t' << (stop_s-start_s)/double(CLOCKS_PER_SEC) << " seconds" << std::endl;
	return 0;
}

