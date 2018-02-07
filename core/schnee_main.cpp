#include "vector.h"
#include "plane.h"
#include "file_loader.h"

#include <iostream>
#include <string>
#include <vector>

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

	// Get points
	std::vector<sVector3> points;
	if(!FL_OFF_load_points(pin, points))
	{
		points.clear();
		exit(3);
	}

	std::vector<sPlane> planes;
	// Process tangents
	for(int i = 0; i < points.size(); ++i)
	{
		// Find k nearest points
		// UNIMPLEMENTED
		sPlane plane = std::make_shared<Plane>();
		plane->center = points[i];
		plane->normal = std::make_shared<Vector3>(Vector3::zup());
		planes.push_back(plane);
	}



	return 0;
}
