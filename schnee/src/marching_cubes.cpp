#include "marching_cubes.h"

#include "file_saver.h"

Grid::Grid(const PlaneCloud & plc, float cell_size) :
    _plc(plc), _csize(cell_size), _hcsize(cell_size * 0.5f)
{
	PLC_get_bounds(_plc,
	               _bbox_min.x, _bbox_min.y, _bbox_min.z,
	               _bbox_max.x, _bbox_max.y, _bbox_max.z);
	std::cout << "BBOX MIN " << _bbox_min << "\n";
	std::cout << "BBOX MAX " << _bbox_max << "\n";
	_size_x = std::ceil((_bbox_max.x - _bbox_min.x ) / _csize + 1);
	_size_y = std::ceil((_bbox_max.y - _bbox_min.y ) / _csize + 1);
	_size_z = std::ceil((_bbox_max.z - _bbox_min.z ) / _csize + 1);
	std::cout << "NBCELLS : " << _size_x << " / " << _size_y << " / " << _size_z << "\n";

	// Checks
	assert(_bbox_min.x + _csize * _size_x >= _bbox_max.x);
	assert(_bbox_min.y + _csize * _size_y >= _bbox_max.y);
	assert(_bbox_min.z + _csize * _size_z >= _bbox_max.z);
	std::cout << "TOTAL NB CELLS2: " <<
	             (_size_z - 1) * (_size_x * _size_y) + (_size_y - 1) * _size_x + _size_x << "\n";
	std::cout << std::endl;
	assert(_size_x * _size_y * _size_z ==
	             (_size_z - 1) * (_size_x * _size_y) + (_size_y - 1) * _size_x + _size_x);

}

void Grid::create_cells(const plane_cloud_index & index)
{
	static std::pair<int, int> edges_cons[] = {
	    // FRONT
	    {0, 1}, // left
	    {1, 2}, // Bottom
	    {2, 3}, // right
	    {3, 0}, // top
	    // BACK
	    {5, 4}, // left
	    {5, 6}, // bottom
	    {6, 7}, // right
	    {7, 4}, // top
	    // LEFT
	    {0, 4}, // top
	    {1, 5}, // bottom
	    // RIGHT
	    {3, 7}, // top
	    {6, 2} // bottom
	};

	// Create only cubes intersecting the mesh

	// Debug "cells"
	std::vector<sVector3> corners;
	std::vector<sVector3> cell_centers;

	Vector3 cell_center;

	float h_csize = _csize * 0.5f;
	float squared_csize = _csize * _csize;

	Vector3 directions[8] = {
	    // FRONT
	    // Left top 0
	    Vector3(-1, 1, 1) * h_csize,
	    // Left bottom 1
	    Vector3(-1, -1, 1) * h_csize,
	    // Right bottom 2
	    Vector3(1, -1, 1) * h_csize,
	    // Right top 3
	    Vector3(1, 1, 1) * h_csize,
	    // BACK
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
    std::vector<size_t>     ret_index(num_results);
    std::vector<float>      out_squared_dist(num_results);
	size_t                  nbhd_count;
	float                 kd_query[3] =
	    {cell_center.x, cell_center.y, cell_center.z};

	sCell current_cell;
	sCellEdge current_edge;

	for(float z = 0; z < _size_z; z+=1)
	{
        cell_center.z = kd_query[2] = _bbox_min.z + z * _csize;
        for(float y = 0; y < _size_y; y+=1)
        {
			cell_center.y = kd_query[1] = _bbox_min.y + y * _csize;
            for(float x = 0; x < _size_x; x+=1)
            {
                cell_center.x = kd_query[0] = _bbox_min.x + x * _csize;

				current_cell = std::make_shared<Cell>();

				for(int i = 0; i < 8; i++)
				{
					current_edge = std::make_shared<CellEdge>();
					current_edge->va = std::make_shared<Vector3>(cell_center + directions[edges_cons[i].first]);
					current_edge->vb = std::make_shared<Vector3>(cell_center + directions[edges_cons[i].second]);
					current_cell->edges.push_back(current_edge);
				}
				/*
				// Left front edge
                current_edge = std::make_shared<CellEdge>();
                current_edge->va = std::make_shared<Vector3>(cell_center + directions[0]);
                current_edge->vb = std::make_shared<Vector3>(cell_center + directions[1]);
				current_cell->edges.push_back(current_edge);
				// Bottom front edge
                current_edge = std::make_shared<CellEdge>();
                current_edge->va = std::make_shared<Vector3>(cell_center + directions[1]);
                current_edge->vb = std::make_shared<Vector3>(cell_center + directions[2]);
				current_cell->edges.push_back(current_edge);
				// Right front edge
                current_edge = std::make_shared<CellEdge>();
                current_edge->va = std::make_shared<Vector3>(cell_center + directions[2]);
                current_edge->vb = std::make_shared<Vector3>(cell_center + directions[3]);
				current_cell->edges.push_back(current_edge);
				// Top front edge
                current_edge = std::make_shared<CellEdge>();
                current_edge->va = std::make_shared<Vector3>(cell_center + directions[3]);
                current_edge->vb = std::make_shared<Vector3>(cell_center + directions[0]);
				current_cell->edges.push_back(current_edge);
				// Back left edge
                current_edge = std::make_shared<CellEdge>();
                current_edge->va = std::make_shared<Vector3>(cell_center + directions[5]);
                current_edge->vb = std::make_shared<Vector3>(cell_center + directions[4]);
				current_cell->edges.push_back(current_edge);
				// Back bottom edge
                current_edge = std::make_shared<CellEdge>();
                current_edge->va = std::make_shared<Vector3>(cell_center + directions[5]);
                current_edge->vb = std::make_shared<Vector3>(cell_center + directions[6]);
				current_cell->edges.push_back(current_edge);
				// Back right edge
                current_edge = std::make_shared<CellEdge>();
                current_edge->va = std::make_shared<Vector3>(cell_center + directions[6]);
                current_edge->vb = std::make_shared<Vector3>(cell_center + directions[7]);
				current_cell->edges.push_back(current_edge);
				// Back top edge
                current_edge = std::make_shared<CellEdge>();
                current_edge->va = std::make_shared<Vector3>(cell_center + directions[7]);
                current_edge->vb = std::make_shared<Vector3>(cell_center + directions[4]);
				current_cell->edges.push_back(current_edge);
				// Left top edge
                current_edge = std::make_shared<CellEdge>();
                current_edge->va = std::make_shared<Vector3>(cell_center + directions[0]);
                current_edge->vb = std::make_shared<Vector3>(cell_center + directions[4]);
				current_cell->edges.push_back(current_edge);
				// Left bottom edge
                current_edge = std::make_shared<CellEdge>();
                current_edge->va = std::make_shared<Vector3>(cell_center + directions[1]);
                current_edge->vb = std::make_shared<Vector3>(cell_center + directions[5]);
				current_cell->edges.push_back(current_edge);
				// Right top edge
                current_edge = std::make_shared<CellEdge>();
                current_edge->va = std::make_shared<Vector3>(cell_center + directions[3]);
                current_edge->vb = std::make_shared<Vector3>(cell_center + directions[7]);
				current_cell->edges.push_back(current_edge);
				// Right bottom edge
                current_edge = std::make_shared<CellEdge>();
                current_edge->va = std::make_shared<Vector3>(cell_center + directions[6]);
                current_edge->vb = std::make_shared<Vector3>(cell_center + directions[2]);
				current_cell->edges.push_back(current_edge);
				*/
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

bool MC_is_point_in_cell(const Vector3 & point, const Vector3 & cell, const float & radius)
{
	return point <= cell + radius && point >= cell - radius;
}

