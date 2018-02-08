#include "file_saver.h"

#include <fstream>
#include <sstream>
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

bool FS_OFF_save_planes(const std::string & path, const std::vector<sPlane> & planes, float size)
{
	std::ofstream writer(path);

	assert(writer.is_open());


	writer << "OFF\n";
	writer << planes.size() * 4 << " " << planes.size() << " 0\n";

	auto it = planes.begin();
	size_t index = 0;
	Vector3 topleft, topright, bottomleft, bottomright;
	Vector3 & u = *(planes[0]->u.get());
	Vector3 & v = *(planes[0]->v.get());
	Vector3 & n = *(planes[0]->normal.get());
	Vector3 & p = *(planes[0]->center.get());

	std::stringstream verts, faces;

	writer << std::fixed;

	while(it != planes.end())
	{
		u = *((*it)->u.get());
		v = *((*it)->v.get());
		n = *((*it)->normal.get());
		p = *((*it)->center.get());

		u = Vector3::cross(v, n);

		topright = p + u * size + v * size;
		bottomright = p - u * size + v * size;
		bottomleft = p - u * size - v * size;
		topleft = p + u * size - v * size;

		/*
		topright = p + n * size;
		bottomright = p;
		bottomleft = p;
		bottomleft.x += size / 2;
		topleft = topright;
		topleft.x += size / 2;
		*/

		// vert buffer
		verts << topright.x << " " << topright.y << " " << topright.z << "\n";
		verts << bottomright.x << " " << bottomright.y << " " << bottomright.z << "\n";
		verts << bottomleft.x << " " << bottomleft.y << " " << bottomleft.z << "\n";
		verts << topleft.x << " " << topleft.y << " " << topleft.z << "\n";

        // face buffer
        faces << "4 " << index << " " << ++index << " " << ++index << " " << ++index << "\n";
		index++;
		it++;
	}

	writer << verts.rdbuf();
	writer << faces.rdbuf();


	writer.close();


	return true;

}
