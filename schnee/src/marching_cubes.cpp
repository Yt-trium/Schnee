#include "marching_cubes.h"
#include "plane.h"

Grid::Grid(const Vector3 & bbox_min, const Vector3 & bbox_max, float cell_size) :
    _csize(cell_size), _hcsize(cell_size * 0.5f),
    _bbox_min(bbox_min - cell_size), _bbox_max(bbox_max)
{
	_size_x = std::ceil((_bbox_max.x - _bbox_min.x ) / _csize );
	_size_y = std::ceil((_bbox_max.y - _bbox_min.y ) / _csize );
	_size_z = std::ceil((_bbox_max.z - _bbox_min.z ) / _csize );

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
	_size_x(other._size_x), _size_y(other._size_y), _size_z(other._size_z)
{
}

void Grid::create_cells()
{
    assert(_size_x * _size_z * _size_y < 20000); // For performance issues

	Vector3 cell_center;
	float h_csize = _csize * 0.5f;
	float squared_csize = _csize * _csize;
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
	const size_t            size_xy = _size_x * _size_y;
	size_t                  cell_index;

	sCell current_cell;
	sCell other_cell;

	for(float z = 0; z < _size_z; z+=1)
	{
        cell_center.z = _bbox_min.z + z * _csize + h_csize;
        for(float y = 0; y < _size_y; y+=1)
        {
            cell_center.y = _bbox_min.y + y * _csize + h_csize;
            for(float x = 0; x < _size_x; x+=1)
            {
                cell_center.x = _bbox_min.x + x * _csize + h_csize;

				current_cell = std::make_shared<Cell>();
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
                    cell_index = z * (size_xy) + y * _size_x + x - 1;
                    assert(cell_index < _cells.size());
                    other_cell = _cells.at(cell_index);
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
                    cell_index = z * (size_xy) + (y - 1) * _size_x + x;
                    assert(cell_index < _cells.size());
                    other_cell = _cells.at(cell_index);
                    assert(bool(other_cell));
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
                    cell_index = (z - 1) * (size_xy) + y * _size_x + x;
                    assert(cell_index < _cells.size());
                    other_cell = _cells.at(cell_index);
                    assert(bool(other_cell));
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

                _cells.push_back(current_cell);
            }
        }
	}

}

void Grid::getUniquePoints(std::vector<sCellPoint> & out) const
{
	int index;
	int size_xy = _size_x * _size_y;
	sCell curcell;

	assert(_cells.size() == _size_x * _size_y * _size_z);

	bool topx, topy, topz;

	for(int z = 0; z < _size_z; z+=1)
	{
		topz = z + 1 == _size_z;
        for(int y = 0; y < _size_y; y+=1)
        {
            topy = y + 1 == _size_y;
            for(int x = 0; x < _size_x; x+=1)
			{
                topx = x + 1 == _size_x;

				index = z * size_xy + y * _size_x + x;
				curcell = _cells[index];
                out.push_back(curcell->corners[0]);

				// If on borders
				if(topx)
                    out.push_back(curcell->corners[1]);

				if(topy)
                {
                    out.push_back(curcell->corners[4]);
                    if(topx)
                        out.push_back(curcell->corners[5]);
                }

				if(topz)
				{
                    out.push_back(curcell->corners[3]);
                    if(topx && topy)
                        out.push_back(curcell->corners[6]);
                    else if(topx)
                        out.push_back(curcell->corners[2]);
                    else if(topy)
                        out.push_back(curcell->corners[6]);
                }
			}
		}
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
    Vector3                 z; // Projection of p on plane

	float ignore_threshold = pow((density + noise) * 2, 2);

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

        // z = o - ((p - o) . n) * n
        z = *(current_plane.center) -
                Vector3::dot((*(p.get()) - *(current_plane.center)), *(current_plane.normal)) *
                *(current_plane.normal);

        if(z.distanceTo(*(current_plane.center)) > density + noise)
        {
        	p->fd = 0.0 / 0.0;
		}
        else
        {
            // f(p) = (p - o) . n
            p->fd = Vector3::dot(*(p.get()) - *(current_plane.center.get())
                                 , *(current_plane.normal.get()));
        }
	}

}

bool MC_is_point_in_cell(const Vector3 & point, const Vector3 & cell, const float & radius)
{
	return point <= cell + radius && point >= cell - radius;
}

