#include "file_saver.h"

#include <fstream>
#include <cassert>

bool FS_OFF_save_points(const std::string & path, const std::vector<sVector3> & points)
{
	std::ofstream writer(path);

	assert(writer.is_open());

	writer << "OFF\n";
	writer << points.size() << " 0 0\n";

	auto it = points.begin();
	sVector3 current;

	writer << std::fixed;

	while(it != points.end())
	{
		current = (*it);
		writer << current->x << " " << current->y << " " << current->z << "\n";

		it++;
	}

	writer.close();

	return true;
}
