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

static void MC_create_loop_edge(
        sCell & cell, size_t start, size_t end,
        const Vector3 & cell_center,
        const Vector3 * directions,
        const std::pair<int, int> * connections
        )
{
	sCellEdge current_edge;
    for(int i = start; i < end; i++)
    {
        current_edge = std::make_shared<CellEdge>();
		if(i == start)
            current_edge->va = std::make_shared<CellPoint>(cell_center + directions[connections[i].first]);
		else
			current_edge->va = cell->edges[i -1]->vb;
		if(i == end -1)
			current_edge->vb = cell->edges[start]->va;
		else
            current_edge->vb = std::make_shared<CellPoint>(cell_center + directions[connections[i].second]);
        cell->edges[i] = current_edge;
    }
}

static void MC_create_edge(
        sCell & cell, const size_t & edge,
        const Vector3 & cell_center,
        const Vector3 * directions,
        const std::pair<int, int> * connections
        )
{
	sCellEdge cedge;
	cedge = std::make_shared<CellEdge>();
	cedge->va = std::make_shared<CellPoint>(cell_center + directions[connections[edge].first]);
	cedge->vb = std::make_shared<CellPoint>(cell_center + directions[connections[edge].second]);
	cell->edges[edge] = cedge;
}

static void MC_create_edge_from_a(
        sCell & new_cell,
        const size_t & edge,
        const size_t & edge_for_a, const size_t & edge_for_b)
{
	sCellEdge sedge = std::make_shared<CellEdge>();
	sedge->va = new_cell->edges[edge_for_a]->va;
	sedge->vb = new_cell->edges[edge_for_b]->va;
	new_cell->edges[edge] = sedge;
}

static void MC_create_edge_from_b(
        sCell & new_cell,
        const size_t & edge,
        const size_t & edge_for_a, const size_t & edge_for_b)
{
	sCellEdge sedge = std::make_shared<CellEdge>();
	sedge->va = new_cell->edges[edge_for_a]->vb;
	sedge->vb = new_cell->edges[edge_for_b]->vb;
	new_cell->edges[edge] = sedge;
}

static void MC_link_edges(
        sCell & new_cell,
        sCell & parent_cell,
        size_t start, size_t end,
        size_t start_parent)
{
	for(int i = start, j = start_parent; i < end; i++, j++)
	{
		new_cell->edges[i] = parent_cell->edges[j];
	}
}

static void MC_link_edge(
        sCell & new_cell,
        sCell & parent_cell,
        size_t a, size_t b)
{
	new_cell->edges[a] = parent_cell->edges[b];
}

void Grid::create_cells()
{
    assert(_size_x * _size_z * _size_y < 20000); // For performance issues
	// Edge vertices connections
	static std::pair<int, int> edges_cons[] = {
	    // FRONT + Z
	    {1, 0}, // left
	    {1, 2}, // Bottom
	    {2, 3}, // right
	    {0, 3}, // top
	    // BACK - Z
	    {5, 4}, // left
	    {5, 6}, // bottom
	    {6, 7}, // right
	    {4, 7}, // top
	    // LEFT - X
	    {0, 4}, // top
	    {1, 5}, // bottom
	    // RIGHT + X
	    {3, 7}, // top
	    {2, 6} // bottom
	};

	// Create only cubes intersecting the mesh

	// Debug "cells"
	std::vector<sVector3> corners;
	std::vector<sVector3> cell_centers;

	Vector3 cell_center;

	float h_csize = _csize * 0.5f;
	float squared_csize = _csize * _csize;

	Vector3 directions[8] = {
	    // FRONT + Z
	    // Left top 0
	    Vector3(-1, 1, 1) * h_csize,
	    // Left bottom 1
	    Vector3(-1, -1, 1) * h_csize,
	    // Right bottom 2
	    Vector3(1, -1, 1) * h_csize,
	    // Right top 3
	    Vector3(1, 1, 1) * h_csize,
	    // BACK - Z
	    // Left top 4
	    Vector3(-1, 1, -1) * h_csize,
	    // Left bottom 5
	    Vector3(-1, -1, -1) * h_csize,
	    // Right bottom 6
	    Vector3(1, -1, -1) * h_csize,
	    // Right top 7
	    Vector3(1, 1, -1) * h_csize
	};
	const size_t            num_results = 1;
	const size_t            size_xy = _size_x * _size_y;
    std::vector<size_t>     ret_index(num_results);
    std::vector<float>      out_squared_dist(num_results);
	size_t                  nbhd_count;
	size_t                  cell_index;
	float                 kd_query[3] =
	    {cell_center.x, cell_center.y, cell_center.z};

	sCell current_cell;
	sCell other_cell;
	sCellEdge current_edge;

	for(float z = 0; z < _size_z; z+=1)
	{
        cell_center.z = kd_query[2] = _bbox_min.z + z * _csize + h_csize;
        for(float y = 0; y < _size_y; y+=1)
        {
			cell_center.y = kd_query[1] = _bbox_min.y + y * _csize + h_csize;
            for(float x = 0; x < _size_x; x+=1)
            {
                cell_center.x = kd_query[0] = _bbox_min.x + x * _csize + h_csize;

				current_cell = std::make_shared<Cell>();
				current_cell->edges.resize(12);

				// Connect edges to other cells

				// Back of this cell is equal to the front of the previous Z cell
				if(z > 0)
				{
                    cell_index = (z - 1) * (size_xy) + y * _size_x + x;
					assert(cell_index < _cells.size());
					other_cell = _cells.at(cell_index);
					MC_link_edges(current_cell, other_cell, 4, 8, 0);
				}

				// Bottom of this cell is equal to the top of the previous Y cell
				if(y > 0)
				{
                    cell_index = z * (size_xy) + (y - 1) * _size_x + x;
					assert(cell_index < _cells.size());
					other_cell = _cells.at(cell_index);
					// Front
					MC_link_edge(current_cell, other_cell, 1, 3);
					// Back
					// No need to link if there is a Z link
					if(z == 0)
                        MC_link_edge(current_cell, other_cell, 5, 7);
					// Left
					MC_link_edge(current_cell, other_cell, 9, 8);
					// Right
					MC_link_edge(current_cell, other_cell, 11, 10);
				}

				// Left of this cell is equal to the right of the previous X cell
				if(x > 0)
				{
                    cell_index = z * (size_xy) + y * _size_x + x - 1;
					assert(cell_index < _cells.size());
					other_cell = _cells.at(cell_index);
                    MC_link_edge(current_cell, other_cell, 0, 2);
					// If there is Z link, 4 already exists
					if(z == 0)
                        MC_link_edge(current_cell, other_cell, 4, 6);
                    MC_link_edge(current_cell, other_cell, 8, 10);
					// If there is Y link, 9 already exists
					if(y == 0)
                        MC_link_edge(current_cell, other_cell, 9, 11);
				}

				// Other non linked new edges
				if(z == 0 && y == 0 && x == 0)
				{
					//MC_create_loop_edge(current_cell, 0, 4, cell_center, directions, edges_cons);
					//MC_create_loop_edge(current_cell, 4, 8, cell_center, directions, edges_cons);
					// Front
					MC_create_edge(current_cell, 4, cell_center, directions, edges_cons);
					MC_create_edge(current_cell, 6, cell_center, directions, edges_cons);
					MC_create_edge_from_a(current_cell, 5, 4, 6);
					MC_create_edge_from_b(current_cell, 7, 4, 6);
					// Back
					MC_create_edge(current_cell, 0, cell_center, directions, edges_cons);
					MC_create_edge(current_cell, 2, cell_center, directions, edges_cons);
					MC_create_edge_from_a(current_cell, 1, 0, 2);
					MC_create_edge_from_b(current_cell, 3, 0, 2);

					// Left
					MC_create_edge_from_a(current_cell, 8, 3, 7);
					MC_create_edge_from_a(current_cell, 9, 1, 5);
					// Right
					MC_create_edge_from_b(current_cell, 10, 3, 7);
					MC_create_edge_from_b(current_cell, 11, 1, 5);
				}
				else if(y == 0 && x == 0)
				{
					MC_create_edge(current_cell, 3, cell_center, directions, edges_cons);
					MC_create_edge(current_cell, 1, cell_center, directions, edges_cons);

					MC_create_edge_from_a(current_cell, 0, 1, 3);
					MC_create_edge_from_b(current_cell, 2, 1, 3);

					MC_create_edge_from_a(current_cell, 8, 3, 7);
					MC_create_edge_from_a(current_cell, 9, 1, 5);
					MC_create_edge_from_b(current_cell, 10, 3, 7);
					MC_create_edge_from_b(current_cell, 11, 1, 5);
				}
				else if(y == 0 && z == 0)
				{
					MC_create_edge(current_cell, 10, cell_center, directions, edges_cons);
					MC_create_edge(current_cell, 11, cell_center, directions, edges_cons);

					MC_create_edge_from_a(current_cell, 2, 11, 10);
					MC_create_edge_from_b(current_cell, 6, 11, 10);

					MC_create_edge_from_a(current_cell, 1, 9, 11);
					MC_create_edge_from_a(current_cell, 3, 8, 10);
					MC_create_edge_from_b(current_cell, 7, 8, 10);
					MC_create_edge_from_b(current_cell, 5, 9, 11);
				}
				else if(z == 0 && x == 0)
				{
					MC_create_edge(current_cell, 8, cell_center, directions, edges_cons);
					MC_create_edge(current_cell, 10, cell_center, directions, edges_cons);

					MC_create_edge_from_a(current_cell, 3, 8, 10);
					MC_create_edge_from_b(current_cell, 7, 8, 10);

					MC_create_edge_from_a(current_cell, 0, 1, 3);
					MC_create_edge_from_b(current_cell, 2, 1, 3);
					MC_create_edge_from_b(current_cell, 4, 9, 8);
					MC_create_edge_from_b(current_cell, 6, 11, 10);
				}
				else if(y == 0)
				{
					MC_create_edge(current_cell, 2, cell_center, directions, edges_cons);

					MC_create_edge_from_a(current_cell, 11, 2, 6);
					MC_create_edge_from_b(current_cell, 10, 2, 6);
					MC_create_edge_from_a(current_cell, 1, 9, 11);
					MC_create_edge_from_b(current_cell, 3, 0, 2);
				}
				else if(x == 0)
				{
					MC_create_edge(current_cell, 3, cell_center, directions, edges_cons);

					MC_create_edge_from_a(current_cell, 8, 3, 7);
					MC_create_edge_from_a(current_cell, 0, 9, 8);
					MC_create_edge_from_b(current_cell, 2, 1, 3);
					MC_create_edge_from_b(current_cell, 10, 2, 6);
				}
				else if(z == 0)
				{
					MC_create_edge(current_cell, 10, cell_center, directions, edges_cons);

					MC_create_edge_from_a(current_cell, 2, 11, 10);
					MC_create_edge_from_a(current_cell, 3, 8, 10);
					MC_create_edge_from_b(current_cell, 6, 11, 10);
					MC_create_edge_from_b(current_cell, 7, 8, 10);
				}
				else
				{
					sCellPoint newPoint = std::make_shared<CellPoint>(cell_center +
					                                              directions[3]);
					sCellEdge newEdge = std::make_shared<CellEdge>();
					newEdge->va = newPoint;
					newEdge->vb = current_cell->edges[7]->vb;
					current_cell->edges[10] = newEdge;
					MC_create_edge_from_a(current_cell, 3, 8, 10);
					MC_create_edge_from_b(current_cell, 2, 1, 3);

				}

				_cells.push_back(current_cell);

# if 0
				// Test if any point in cell
                nbhd_count = index.knnSearch(&kd_query[0], num_results, &ret_index[0], &out_squared_dist[0]);
                assert(nbhd_count == num_results);

				if(out_squared_dist[0] > squared_csize)
					continue;
# endif

				//sGrid grid = std::make_shared<Grid>();
				//for(int i = 0; i < 8; i++)
                    //grid->corners.emplace_back(new Vector3(cell_center + directions[i]));
				//grids.push_back(grid);
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
				out.push_back(curcell->edges[4]->va);

				// If on borders
				if(topx)
				{
					out.push_back(curcell->edges[6]->va);
				}

				if(topy)
				{
					out.push_back(curcell->edges[4]->vb);
					if(topx)
					{
                        out.push_back(curcell->edges[6]->vb);
					}
				}

				if(topz)
				{
					out.push_back(curcell->edges[0]->va);
					if(topy)
                        out.push_back(curcell->edges[0]->vb);
					if(topx)
					{
                        out.push_back(curcell->edges[2]->va);
						if(topy)
                            out.push_back(curcell->edges[2]->vb);
					}
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

