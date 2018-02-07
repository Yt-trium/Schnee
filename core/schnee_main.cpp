#include "vector.h"
#include "plane.h"
#include "file_loader.h"
#include "nanoflann.hpp"

#include <iostream>
#include <string>
#include <vector>

struct PointCloud
{
	std::vector<sVector3> points;
};

int main(int argc, const char * argv[])
{
	if(argc < 3)
	{
		std::cout << argv[0] << " off_input_file k" << std::endl;
		exit(2);
	}

	// In off file
	std::string pin = argv[1];
	int k = std::stoi(argv[2]);

	// Create empty point cloud
	PointCloud pc;
	std::vector<sVector3> & pc_points = pc.points;

	// Get points
	if(!FL_OFF_load_points(pin, pc_points))
	{
		pc_points.clear();
		exit(3);
	}

	// Create nearest neighbour tree
	//typedef nanoflann::KDTreeSingleIndexAdaptor

	std::vector<sPlane> planes;
	// Process tangents
	for(int i = 0; i < pc_points.size(); ++i)
	{
		// Find k nearest points
		// UNIMPLEMENTED
		sPlane plane = std::make_shared<Plane>();
		plane->center = pc_points[i];
		plane->normal = std::make_shared<Vector3>(Vector3::zup());
		planes.push_back(plane);
	}



	return 0;
}
