#include "file_loader.h"

#include <fstream>
#include <stdexcept>
#include <iostream>

bool FL_OFF_load_points(const std::string & path, std::vector<sVector3> & output)
{
	std::ifstream reader(path);
	// Checkup
	if(!reader.is_open()) throw std::runtime_error("Unable to open the file");

	// Reading variables
	std::string reader_str;
	int reader_int;
	float reader_float;

	// Check file type
	reader >> reader_str;
	if(reader_str.compare("OFF"))
	{
		reader.close();
		std::cout << path << " is not an off file" << std::endl;
		return false;
	}

	int vert_count;
	reader >> vert_count >> reader_int >> reader_int; // Line ends with 0
    //std::cout << "Vert count: " << vert_count << std::endl;

	// Read verts
	float x, y, z;
	for(int i = 0; i < vert_count; i++)
	{
		reader >> x >> y >> z;
		output.emplace_back(new Vector3(x, y, z));
	}

	// Cleaning
	reader.close();

	return true;
}

