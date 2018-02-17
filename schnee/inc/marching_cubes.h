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
	 * FRONT: + Z
	 * 0 : left     (top left       -> bottom left)
	 * 1 : bottom   (bottom left    -> bottom right)
	 * 2 : right    (bottom right   -> top right)
	 * 3 : top      (top right      -> top left)
	 * BACK: - Z
	 * 4 : left     (top left       -> bottom left)
	 * 5 : bottom   (bottom left    -> bottom right)
	 * 6 : right    (bottom right   -> top right)
	 * 7 : top      (top right      -> top left)
	 * LEFT: - X
	 * 8 : top      (top front left     -> top back left)
	 * 9 : bottom   (bottom front left  -> bottom back left)
	 * RIGHT : + Y
	 * 10 : top     (top front right    -> top back right)
	 * 11 : bottom  (bottom front right -> bottom back right)
	 */
	std::vector<sCellEdge> edges;

};

typedef std::shared_ptr<Cell> sCell;

class Grid
{
public:
	Grid(const Vector3&, const Vector3 &, float);

	void create_cells();

	const std::vector<sCell> & cells() const { return _cells; }

	int sizeX() const { return _size_x; }
	int sizeY() const { return _size_y; }
	int sizeZ() const { return _size_z; }

private:
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
