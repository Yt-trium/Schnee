#include "file_saver.h"
#include "marching_cubes.h"

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iterator>
#include <sstream>
#include <vector>

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
		const Vector3 & n = *((*it)->normal.get());
		Vector3 v = Vector3::cross(u, n);
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

bool FS_OFF_save_cells_position(const std::string & path, const std::vector<sCell> & cells)
{
	std::ofstream writer(path);

	assert(writer.is_open());

	writer << "OFF\n";

	std::stringstream verts;
    verts << std::fixed;
	std::vector<Vector3 *> writenPoints;

    Vector3 * va;
	for(int i = 0; i < cells.size(); ++i)
	{
		const sCell & c = cells[i];
        for(int j = 0; j < c->corners.size(); ++j)
		{
            const sCellPoint & cp = c->corners.at(j);
            va = cp.get();

			if(std::find(writenPoints.begin(), writenPoints.end(), va) == writenPoints.end())
			{
				writenPoints.push_back(va);
				verts << va->x << " " << va->y << " " << va->z << "\n";
			}
		}
	}

    writer << writenPoints.size() << " 0 0\n";
	writer << std::fixed;
	writer << verts.rdbuf();

	writer.close();

	return true;

}

bool FS_OFF_save_cell_points(const std::string & path, const std::vector<sCellPoint>& corners, float isolevel)
{
	std::ofstream writer(path);

	assert(writer.is_open());

	writer << "COFF\n";
	writer << corners.size() << " 0 0\n";

	writer << std::fixed;

	for(int i = 0; i < corners.size(); i++)
	{
		float d = corners[i]->fd;
		const Vector3 & p = *(corners[i].get());
        writer << p.x << " " << p.y << " " << p.z << " ";
		if(d != d)
		{
            writer << 0.0f << " " << 0.0f << " " << 0.0f << " " << 1.0f << "\n";
		}
		else if(d < isolevel)
		{
            writer << 1.0f << " " << 0.0f << " " << 0.0f << " " << 1.0f << "\n";
		}
		else
		{
            writer << 0.0f << " " << 1.0f << " " << 0.0f << " " << 1.0f << "\n";
		}
	}

	writer.close();

	return true;

}

bool FS_OFF_save_mesh(const std::string & path, const mesh::Mesh & mesh)
{
    std::ofstream writer(path);
    assert(writer.is_open());


    writer << "OFF\n";

    std::stringstream verts, faces;
    verts << std::fixed;
    faces << std::fixed;

    std::vector<Vector3 *> writenPoints;
    size_t vert_index;
    Vector3 * va = 0;

    // For each face
    for(size_t f = 0; f < mesh.faces.size(); ++f)
    {
        const mesh::sFace & face = mesh.faces[f];

        faces << face->points.size();
        // For each point
        for(size_t e = 0; e < face->points.size(); ++e)
        {
            va = face->points[e].get();

            vert_index = std::distance(writenPoints.begin(),
                                       std::find(writenPoints.begin(), writenPoints.end(), va));

            if(vert_index >= writenPoints.size()) // new point
            {
                writenPoints.push_back(va);
                verts << va->x << " " << va->y << " " << va->z << "\n";
                faces << " " << writenPoints.size() - 1;
            }
            else // existing
            {
                faces << " " << vert_index;
            }

        }
        faces << "\n";
    }


    writer << writenPoints.size() << " " << mesh.faces.size() << " 0\n";
    writer << std::fixed;
    writer << verts.rdbuf();
    writer << faces.rdbuf();
    writer.close();

    return true;
}
