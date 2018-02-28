#ifndef __MCUBES__
#define __MCUBES__

#include "cloud.h"
#include "mesh.h"
#include "plane.h"
#include "vector.h"

#include <deque>
#include <map>
#include <memory>
#include <vector>

#define DF_UNDEFINED -10.0f

typedef std::map<int, sVector3> point_map;

/**
 * @brief A cell corner in the grid.
 */
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

/**
 * @brief A Cell in the grid
 */
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
	 * @brief Corners of the cell.
	 */
    std::vector<sCellPoint> corners;

	/**
	 * @brief Based on the look table.
	 * Tells which corner is in the surface.
	 */
	int situation = 0;
};

typedef std::shared_ptr<Cell> sCell;

/**
 * @brief A 3D Grid.
 */
class Grid
{
public:
	/**
	 * @brief Create a grid with 2 corners and a size for each cell
	 */
	Grid(const Vector3&, const Vector3 &, float);
	Grid(const Grid &);

	/**
	 * @brief Build grid cells.
	 * Share cell corners among adjacent cells.
	 */
	void create_cells();

	/**
	 * @brief Create a mesh based on a set of plane.
	 */
	void compute_mesh(const PlaneCloud &, const plane_cloud_index &, float, float, float, mesh::Mesh &);

	/**
	 * @brief Return the 3D point of an edge in the grid
	 * @return
	 */
	sVector3 get_face_vertex(const int &, const int &, const sCell &,
	                     point_map &, point_map &, point_map &);

	/**
	 * @brief Access a cell in the grid
	 * @param x
	 * @param y
	 * @param z
	 * @return
	 */
	const sCell & cell(const float & x, const float & y, const float & z) const
	{
		assert(z * _size_xy + y * _size_x + x < _cells.size());
		assert(bool(_cells.at(z * _size_xy + y * _size_x + x)));
		return _cells.at(z * _size_xy + y * _size_x + x);
	}

	const std::vector<sCell> & cells() const { return _cells; }

    const std::vector<sCellPoint> & uniquePoints() const { return _corners; }

    std::vector<sCellPoint> uniquePoints() { return _corners; }

	int sizeX() const { return _size_x; }
	int sizeY() const { return _size_y; }
	int sizeZ() const { return _size_z; }

private:
	Vector3 _bbox_min;
	Vector3 _bbox_max;
	/**
	 * @brief Cell size
	 */
	float _csize;

	/**
	 * @brief Half cell size
	 */
	float _hcsize;

	/**
	 * @brief Number of cell in each direction
	 */
	int _size_x, _size_y, _size_z, _size_xy;

	/**
	 * @brief All cells
	 */
	std::vector<sCell> _cells;

	/**
	 * @brief List of all unique points in the cell
	 */
	std::vector<sCellPoint> _corners;

	/**
	 * @brief Create a corner with its associated cell.
	 * @return
	 */
	sCell create_corner(
	        const float &, const float &, const float &,
	        const Vector3 &, const Vector3 *);

	/**
	 * @brief Add the corners of the given cell in the unique_corners list of the grid.
	 * Only add points depending on the position of the cell.
	 */
	void add_unique_corners(const sCell &, const bool&, const bool&, const bool&);
};

typedef std::shared_ptr<Grid> sGrid;

/**
 * @brief Compute the signed distance for each corner of the grid based on a plane cloud.
 */
void MC_compute_signed_distance(std::vector<sCellPoint> &,
                                const PlaneCloud &, const plane_cloud_index &, float, float);


#endif
