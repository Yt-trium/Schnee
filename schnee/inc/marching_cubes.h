#ifndef __MCUBES__
#define __MCUBES__

#include "vector.h"
#include "cloud.h"
#include "plane.h"

#include <vector>
#include <memory>

class CellEdge
{
public:
	/**
	 * @brief Start and end of the edge
	 */
	sVector3 va, vb;

	/**
	 * @brief Signed distance value of start and end of the edge
	 */
	float fa, bb;

};

typedef std::shared_ptr<CellEdge> sCellEdge;

class Cell
{
public:

	/**
	 * @brief edges
	 * Size : 12
	 *
	 * From the same point of view, these are the edges index
	 * FRONT:
	 * 0 : left
	 * 1 : bottom
	 * 2 : right
	 * 3 : top
	 * BACK:
	 * 4 : left
	 * 5 : bottom
	 * 6 : right
	 * 7 : top
	 * LEFT:
	 * 8 : top
	 * 9 : bottom
	 * RIGHT :
	 * 10 : top
	 * 11 : bottom
	 */
	std::vector<sCellEdge> edges;

};

typedef std::shared_ptr<Cell> sCell;

class Grid
{
public:
	Grid(const PlaneCloud &, float);

	void create_cells(const plane_cloud_index &);

	const std::vector<sCell> & cells() const { return _cells; }

private:
	const PlaneCloud & _plc;
	Vector3 _bbox_min;
	Vector3 _bbox_max;
	float _csize;
	float _hcsize;
	int _size_x, _size_y, _size_z;
	std::vector<sCell> _cells;
};

typedef std::shared_ptr<Grid> sGrid;

bool MC_is_point_in_cell(const Vector3&, const Vector3&, const float&);



#endif
