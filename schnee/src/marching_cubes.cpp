#include "LookUpTable.h"

#include "marching_cubes.h"
#include "plane.h"

Grid::Grid(const Vector3 & bbox_min, const Vector3 & bbox_max, float cell_size) :
    _csize(cell_size), _hcsize(cell_size * 0.5f),
    _bbox_min(bbox_min - cell_size), _bbox_max(bbox_max)
{
	_size_x = std::ceil((_bbox_max.x - _bbox_min.x ) / _csize );
	_size_y = std::ceil((_bbox_max.y - _bbox_min.y ) / _csize );
	_size_z = std::ceil((_bbox_max.z - _bbox_min.z ) / _csize );
	_size_xy = _size_x * _size_y;

	// Checks
	assert(_bbox_min.x + _csize * _size_x >= _bbox_max.x);
	assert(_bbox_min.y + _csize * _size_y >= _bbox_max.y);
	assert(_bbox_min.z + _csize * _size_z >= _bbox_max.z);
	assert(_size_x * _size_y * _size_z ==
	             (_size_z - 1) * (_size_x * _size_y) + (_size_y - 1) * _size_x + _size_x);
}

Grid::Grid(const Grid & other) : 
	_csize(other._csize), _hcsize(other._hcsize),
	_bbox_min(other._bbox_min), _bbox_max(other._bbox_max),
	_size_x(other._size_x), _size_y(other._size_y), _size_z(other._size_z), _size_xy(other._size_xy)
{
}

void Grid::create_cells()
{
    assert(_size_x * _size_z * _size_y < 80000); // For performance issues
	assert(_cells.size() == 0);
	assert(_corners.size() == 0);

	Vector3 cell_center;
	float h_csize = _csize * 0.5f;

	// Corners direcctions
	Vector3 directions[8] = {
        // FRONT - Z
        // BACK + Z
        // 0: front Bottom left
        Vector3(-1, -1, -1) * h_csize,
        // 1: front Bottom right
        Vector3(1, -1, -1) * h_csize,
        // 2: back Bottom right
        Vector3(1, -1, 1) * h_csize,
        // 3: back Bottom left
        Vector3(-1, -1, 1) * h_csize,
        // 4: front Top left
        Vector3(-1, 1, -1) * h_csize,
        // 5: front Top right
        Vector3(1, 1, -1) * h_csize,
        // 6: back Top right
        Vector3(1, 1, 1) * h_csize,
        // 7: back Top left
        Vector3(-1, 1, 1) * h_csize
	};

	bool topx, topy, topz;

	sCell current_cell;

	for(float z = 0; z < _size_z; z+=1)
	{
        cell_center.z = _bbox_min.z + z * _csize + h_csize;
		topz = z + 1 == _size_z;
        for(float y = 0; y < _size_y; y+=1)
        {
            cell_center.y = _bbox_min.y + y * _csize + h_csize;
            topy = y + 1 == _size_y;
            for(float x = 0; x < _size_x; x+=1)
            {
                cell_center.x = _bbox_min.x + x * _csize + h_csize;
                topx = x + 1 == _size_x;

				// Create cell
				current_cell = create_corner(x, y, z, cell_center, directions);
                _cells.push_back(current_cell);

				// Fill up unique corners list
				add_unique_corners(current_cell, topx, topy, topz);
            }
        }
	}
}

static void edge_cornders_indices(const int & i, int & a, int & b)
{
	/**
     * Cube description:
     *         7 ________ 6           _____6__             ________
     *         /|       /|         7/|       /|          /|       /|
     *       /  |     /  |        /  |     /5 |        /  6     /  |
     *   4 /_______ /    |      /__4____ /    10     /_______3/    |
     *    |     |  |5    |     |    11  |     |     |     |  |   2 |
     *    |    3|__|_____|2    |     |__|__2__|     | 4   |__|_____|
     *    |    /   |    /      8   3/   9    /      |    /   |    /
     *    |  /     |  /        |  /     |  /1       |  /     5  /
     *    |/_______|/          |/___0___|/          |/_1_____|/
     *   0          1        0          1
	 */
    a = i % 8;
    b = (i + 1) % 4;
    if(i == 11)
        b = 7;
    else if(i > 7)
        b += 3;
    else if(i > 3)
        b += 4;
	assert(a >= 0 && a < 8);
	assert(b >= 0 && b < 8);
	assert(a != b);
}

void Grid::compute_mesh(const PlaneCloud & plc, const plane_cloud_index & pci,
                        float density, float noise, float isolevel, mesh::Mesh & out)
{
	// Compute signed distance
	MC_compute_signed_distance(_corners, plc, pci, density, noise);

	// Common
	const char * cell_case;
	int et;
	bool cell_ok;
	mesh::sFace newface;
	sVector3 newpoint;

	// KD Tree
	const size_t            num_results = 1;
    std::vector<size_t>     ret_index(num_results);
    std::vector<float>      out_squared_dist(num_results);
	float                   kd_query[3];
	size_t                  nbhd_count;
	float                   ignore_threshold = (density + noise) * (density + noise);

	// Map storing created vertices
	point_map edges_right; // Edges 0, 2, 4, 6
	point_map edges_top; // Edges 8, 9, 10, 11
	point_map edges_depth; // Edges 1, 3, 5, 7

	// GO
	for(int i = 0; i < _cells.size(); ++i)
	{
		sCell & cur_cell = _cells.at(i);

		// Check if cell intersect the surface
#if 0
		kd_query[0] = (cur_cell->corners[0]->x + cur_cell->corners[6]->x) * 0.5f;
		kd_query[1] = (cur_cell->corners[0]->y + cur_cell->corners[6]->y) * 0.5f;
		kd_query[2] = (cur_cell->corners[0]->z + cur_cell->corners[6]->z) * 0.5f;

        nbhd_count = pci.knnSearch(&kd_query[0], num_results, &ret_index[0], &out_squared_dist[0]);
        assert(nbhd_count == num_results);

		// Ignore if not on the surface
		if(out_squared_dist[0] >= ignore_threshold)
			continue;
#endif

		cell_ok = true;

		// Calculate cell case
		for(int j = 0, b = 1; j < 8; ++j, b *= 2)
		{
			if(cur_cell->corners[j]->fd != cur_cell->corners[j]->fd)
			{
                // Undefined distance
				cell_ok = false;
				break;
			}
			if(cur_cell->corners[j]->fd < isolevel)
			{
				cur_cell->situation |= b;
			}
        }

		if(!cell_ok) continue;

		assert(int(cur_cell->situation) >= 0);
		assert(int(cur_cell->situation) <= 255);

		// Look up edges
        cell_case = casesClassic[cur_cell->situation];

		// Read triangles
		for(int t = 0; t < 16; t += 3)
		{
			if(cell_case[t] == -1) break;
			newface = std::make_shared<mesh::Face>();

			// for each edge of the cell for this face
			for(int e = 0; e < 3; e++)
			{
				// edge indice
				et = cell_case[t + e];

				newpoint = get_face_vertex(i, et, cur_cell, edges_right, edges_top, edges_depth);
				assert(newpoint);

				// Push vertex
				newface->points.push_back(newpoint);
			}

			// Push face
			out.faces.push_back(newface);
		}
	}
}

sVector3 Grid::get_face_vertex(const int & cell_index, const int & edge_index, const sCell & cur_cell,
                           point_map & edges_right, point_map & edges_top, point_map & edges_depth)
{
    // See if the point has already been created
	// index formula z * _size_xy + y * _size_x + x
	int index;
	point_map * map;
	switch (edge_index) {
		case 0:
			map = &edges_right; index = cell_index; break;
		case 1:
			map = &edges_depth; index = cell_index + 1; break; // X + 1
		case 2:
			map = &edges_right; index = cell_index + _size_xy; break; // Z + 1
		case 3:
			map = &edges_depth; index = cell_index; break;
		case 4:
			map = &edges_right; index = cell_index + _size_x; break; // Y + 1
		case 5:
			map = &edges_depth; index = cell_index + 1 + _size_x; break; // Y + 1, X + 1
		case 6:
			map = &edges_right; index = cell_index + _size_xy + _size_x; break; // Z + 1, Y + 1
		case 7:
			map = &edges_depth; index = cell_index + _size_x; break; // Y + 1
		case 8:
			map = &edges_top; index = cell_index; break;
		case 9:
			map = &edges_top; index = cell_index + 1; break; // X + 1
		case 10:
			map = &edges_top; index = cell_index + 1 + _size_xy; break; // Z + 1, X + 1
		case 11:
			map = &edges_top; index = cell_index + _size_xy; break; // Z + 1
			break;
		default:
			assert(false);
			break;
	}

	// Search in map
	bool exists = map->count(index) > 0;

	if(!exists)
	{
		// Create the point
        int c1, c2;
        // Get corners indices of this edge
        edge_cornders_indices(edge_index, c1, c2);

        // Create point
        sVector3 out = std::make_shared<Vector3>(
                  (*(cur_cell->corners[c1]) + *(cur_cell->corners[c2])) * 0.5f
                  );
        // Update maps
        (*map)[index] = out;
		return out;
	}
	else
	{
		return (*map)[index];
	}
}


sCell Grid::create_corner(
        const float & x, const float & y, const float & z,
        const Vector3 & cell_center, const Vector3 * directions)
{
    sCell current_cell = std::make_shared<Cell>();
    current_cell->corners.resize(8);
    // X

    if(x == 0)
    {
        // Create left face
        if(y == 0)
        {
            current_cell->corners[0] = std::make_shared<CellPoint>(
                                           cell_center + directions[0]);
            current_cell->corners[3] = std::make_shared<CellPoint>(
                                           cell_center + directions[3]);
        }
        if(z == 0)
            current_cell->corners[4] = std::make_shared<CellPoint>(
                                           cell_center + directions[4]);
        current_cell->corners[7] = std::make_shared<CellPoint>(
                                       cell_center + directions[7]);
    }
    else
    {
        // Connect left face
        const sCell & other_cell = cell(x - 1, y, z);
        current_cell->corners[4] = other_cell->corners[5];
        current_cell->corners[0] = other_cell->corners[1];
        current_cell->corners[3] = other_cell->corners[2];
        current_cell->corners[7] = other_cell->corners[6];
    }

    // Y

    if(y == 0)
    {
        // Create bottom face
        if(z == 0)
            current_cell->corners[1] = std::make_shared<CellPoint>(
                                           cell_center + directions[1]);
        current_cell->corners[2] = std::make_shared<CellPoint>(
                                       cell_center + directions[2]);
    }
    else
    {
        const sCell & other_cell = cell(x, y - 1, z);
        current_cell->corners[1] = other_cell->corners[5];
        current_cell->corners[2] = other_cell->corners[6];
        if(x == 0)
        {
            current_cell->corners[3] = other_cell->corners[7];
            current_cell->corners[0] = other_cell->corners[4];
        }
    }

    // Z

    if(z == 0)
    {
        current_cell->corners[5] = std::make_shared<CellPoint>(
                                       cell_center + directions[5]);
    }
    else
    {
        // current +Z connected to other -Z
        const sCell & other_cell = cell(x, y, z - 1);
        current_cell->corners[5] = other_cell->corners[6];
        if(x == 0 && y == 0)
        {
            current_cell->corners[0] = other_cell->corners[3];
            current_cell->corners[1] = other_cell->corners[2];
            current_cell->corners[4] = other_cell->corners[7];
        }
        else if(y == 0)
            current_cell->corners[1] = other_cell->corners[2];
        else if(x == 0)
            current_cell->corners[4] = other_cell->corners[7];

    }
    current_cell->corners[6] = std::make_shared<CellPoint>(
                                   cell_center + directions[6]);

	return current_cell;
}

void Grid::add_unique_corners(const sCell & current_cell, const bool& topx, const bool& topy, const bool& topz)
{
    _corners.push_back(current_cell->corners[0]);

    // Borders
    if(topx)
        _corners.push_back(current_cell->corners[1]);

    if(topy)
    {
        _corners.push_back(current_cell->corners[4]);
        if(topx)
            _corners.push_back(current_cell->corners[5]);
    }

    if(topz)
    {
        _corners.push_back(current_cell->corners[3]);
        if(topx && topy)
		{
            _corners.push_back(current_cell->corners[6]);
            _corners.push_back(current_cell->corners[7]);
            _corners.push_back(current_cell->corners[2]);
		}
        else if(topx)
            _corners.push_back(current_cell->corners[2]);
		else if(topy)
            _corners.push_back(current_cell->corners[7]);
    }
}

void MC_compute_signed_distance(std::vector<sCellPoint> & points,
                                const PlaneCloud & plc, const plane_cloud_index & index,
                                float density, float noise)
{
	// Nbhd variables
	const size_t            num_results = 1;
    std::vector<size_t>     ret_index(num_results);
    std::vector<float>      out_squared_dist(num_results);
	float                   kd_query[3];
	size_t                  nbhd_count;
    Vector3                 po, z;

	for(int i = 0; i < points.size(); ++i)
	{
		sCellPoint & p = points[i];

		kd_query[0] = p->x;
		kd_query[1] = p->y;
		kd_query[2] = p->z;

		// Compute
        nbhd_count = index.knnSearch(&kd_query[0], num_results, &ret_index[0], &out_squared_dist[0]);
        assert(nbhd_count == num_results);

        const Plane & current_plane = *(plc.planes[ret_index[0]].get());

		// i: index of closest plane
        // z = oi - ((p - oi) . ni) * ni

		po = *(p.get()) - *(current_plane.center);
        z = *(current_plane.center) -
                (Vector3::dot(po, *(current_plane.normal)) *
                *(current_plane.normal));

        if(z.distanceTo(*(current_plane.center)) >= density + noise)
        {
        	p->fd = 0.0 / 0.0;
		}
        else
        {
            // f(p) = (p - o) . n
            p->fd = Vector3::dot(po
                                 , *(current_plane.normal));
        }
	}
}
