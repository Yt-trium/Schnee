#ifndef __MCUBES__
#define __MCUBES__

#include "cloud.h"
#include "mesh.h"
#include "plane.h"
#include "vector.h"

#include <deque>
#include <memory>
#include <vector>

#define DF_UNDEFINED -10.0f

class CellPoint : public Vector3
{
public:

	/**
	 * @brief Signed distance value
	 */
	float fd;

	CellPoint(const Vector3 & v) : Vector3(v), fd(DF_UNDEFINED)
	{

	}
};

typedef std::shared_ptr<CellPoint> sCellPoint;

class Cell
{
public:

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
    std::vector<sCellPoint> corners;

	int situation = 0;

};

typedef std::shared_ptr<Cell> sCell;

class Grid
{
public:
	Grid(const Vector3&, const Vector3 &, float);
	Grid(const Grid &);

	void create_cells();

	void compute_mesh(const PlaneCloud &, const plane_cloud_index &, float, float, float, mesh::Mesh &);

	const sCell & cell(const float & x, const float & y, const float & z) const
	{
		assert(z * _size_xy + y * _size_x + x < _cells.size());
		assert(bool(_cells.at(z * _size_xy + y * _size_x + x)));
		return _cells.at(z * _size_xy + y * _size_x + x);
	}

	const std::vector<sCell> & cells() const { return _cells; }

    const std::vector<sCellPoint> & uniquePoints() const
	{
		return _corners;
	}

    std::vector<sCellPoint> uniquePoints()
	{
		return _corners;
	}

	int sizeX() const { return _size_x; }
	int sizeY() const { return _size_y; }
	int sizeZ() const { return _size_z; }

private:
	Vector3 _bbox_min;
	Vector3 _bbox_max;
	float _csize;
	float _hcsize;
	int _size_x, _size_y, _size_z, _size_xy;
	std::vector<sCell> _cells;
	std::vector<sCellPoint> _corners; // List of unique corners in the grid

	sCell create_corner(
	        const float &, const float &, const float &,
	        const Vector3 &, const Vector3 *);

	void add_unique_corners(const sCell &, const bool&, const bool&, const bool&);
};

typedef std::shared_ptr<Grid> sGrid;

void MC_compute_signed_distance(std::vector<sCellPoint> &,
                                const PlaneCloud &, const plane_cloud_index &, float, float);


#endif
