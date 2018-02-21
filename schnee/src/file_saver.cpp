#include "file_saver.h"
#include "marching_cubes.h"

#include <fstream>
#include <sstream>
#include <cassert>
#include <vector>
#include <algorithm>

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

	std::stringstream verts, faces;

	writer << std::fixed;

	while(it != planes.end())
	{
		const Vector3 & u = *((*it)->u.get());
		const Vector3 & v = *((*it)->v.get());
		const Vector3 & n = *((*it)->normal.get());
		const Vector3 & p = *((*it)->center.get());

		//u = Vector3::cross(v, n);
		//v = Vector3::cross(*((*it)->u.get()), n);

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
		verts << topleft.x << " " << topleft.y << " " << topleft.z << "\n";
		verts << bottomleft.x << " " << bottomleft.y << " " << bottomleft.z << "\n";
		verts << bottomright.x << " " << bottomright.y << " " << bottomright.z << "\n";

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

bool FS_OFF_save_planes_normals(const std::string & path,
                                const std::vector<sPlane> & planes,
                                size_t count, float size)
{
	std::ofstream writer(path);

	assert(writer.is_open());

	writer << "OFF\n";
	writer << planes.size() * (count + 1) << " 0 0\n";

	auto it = planes.begin();
	Vector3 point;

	writer << std::fixed;

	float dist_mult = size / (float) count;

	while(it != planes.end())
	{
		const Vector3 & n = *((*it)->normal.get());
		const Vector3 & p = *((*it)->center.get());
		for(float i = 0; i <= count; i += 1)
		{
			point = p + n * (dist_mult * i);
            writer << point.x << " " << point.y << " " << point.z << "\n";
		}

		it++;
	}

	writer.close();

	return true;
}

bool FS_OFF_save_vector_values(const std::string & path, const std::vector<sVector3> & corners,
                                const std::vector<float> & distances)
{
	assert(distances.size() == corners.size());
	std::ofstream writer(path);

	assert(writer.is_open());

	writer << "COFF\n";
	writer << corners.size() << " 0 0\n";

	// Get min and max distances for grading
	float min = distances[0], max = distances[0];
	float d;
	for(int i = 0; i < distances.size(); i++)
	{
		d = distances[i];
		if(d > max) max = d;
		else if(d < min) min = d;
	}

	assert(min != max);
	assert(min == min);
	assert(max == max);

	writer << std::fixed;

	float color;
	for(int i = 0; i < corners.size(); i++)
	{
		d = distances[i];
		const Vector3 & p = *(corners[i].get());
        writer << p.x << " " << p.y << " " << p.z << " ";
		if(d == d)
		{
            color = (d - min) / (max - min);
            writer << color << " " << color << " " << color << " " << 1.0f << "\n";
		}
		else
		{
            writer << 1.0f << " " << 0.0f << " " << 0.0f << " " << 1.0f << "\n";

		}
	}

	writer.close();

	return true;

}

bool FS_OFF_save_cells_position(const std::string & path, const std::vector<sCell> & cells)
{
	std::ofstream writer(path);

	assert(writer.is_open());

	writer << "OFF\n";

	size_t count_point = 0;
	std::stringstream verts;
    verts << std::fixed;
	std::vector<Vector3 *> writenPoints;

	Vector3 * va, * vb;
	for(int i = 0; i < cells.size(); ++i)
	{
		const sCell & c = cells[i];
		for(int j = 0; j < 12; ++j)
		{
			const sCellEdge & e = c->edges.at(j);
			va = e->va.get();
			vb = e->vb.get();

			if(std::find(writenPoints.begin(), writenPoints.end(), va) == writenPoints.end())
			{
				writenPoints.push_back(va);
				count_point++;
				verts << va->x << " " << va->y << " " << va->z << "\n";
			}

			if(std::find(writenPoints.begin(), writenPoints.end(), vb) == writenPoints.end())
			{
				writenPoints.push_back(vb);
				count_point++;
				verts << vb->x << " " << vb->y << " " << vb->z << "\n";
			}
		}
	}

	writer << count_point << " 0 0\n";
	writer << std::fixed;
	writer << verts.rdbuf();

	writer.close();

	return true;

}

bool FS_OFF_save_cell_points(const std::string & path, const std::vector<sCellPoint>& corners)
{
	std::ofstream writer(path);

	assert(writer.is_open());

	writer << "COFF\n";
	writer << corners.size() << " 0 0\n";

	// Get min and max distances for grading
	float min = corners[0]->fd, max = corners[0]->fd;
	float d;
	for(int i = 0; i < corners.size(); i++)
	{
		d = corners[i]->fd;
		if(d > max) max = d;
		else if(d < min) min = d;
	}

	assert(min != max);
	assert(min == min);
	assert(max == max);

	writer << std::fixed;

	float color;
	for(int i = 0; i < corners.size(); i++)
	{
		d = corners[i]->fd;
		const Vector3 & p = *(corners[i].get());
        writer << p.x << " " << p.y << " " << p.z << " ";
		if(d == d)
		{
            color = (d - min) / (max - min);
            writer << color << " " << color << " " << color << " " << 1.0f << "\n";
		}
		else
		{
            writer << 1.0f << " " << 0.0f << " " << 0.0f << " " << 1.0f << "\n";

		}
	}

	writer.close();

	return true;

}
