#ifndef __FILE_SAVER__
#define __FILE_SAVER__

#include "marching_cubes.h"
#include "mesh.h"
#include "plane.h"
#include "vector.h"

#include <string>
#include <vector>

/**
 * @brief Save points into an OFF file
 * @param       File path
 * @param       Point list
 * @return True for success
 */
bool FS_OFF_save_points(const std::string &, const std::vector<sVector3> &);

/**
 * @brief Save planes in a file.
 * Create a Quad for each plane.
 * @return
 */
bool FS_OFF_save_planes(const std::string &, const std::vector<sPlane> &, float = 1.0f);

/**
 * @brief Save planes normals as a suite of point
 * @return
 */
bool FS_OFF_save_planes_normals(const std::string &, const std::vector<sPlane> &, size_t = 5, float = 1.0f);

/**
 * @brief Save cells corners
 * @return
 */
bool FS_OFF_save_cells_position(const std::string &, const std::vector<sCell>&);

/**
 * @brief Save colored cells corners.
 * Use the signed distance function to color corners.
 * @return
 */
bool FS_OFF_save_cell_points(const std::string &, const std::vector<sCellPoint>&, float);

/**
 * @brief Save a mesh.
 * @return
 */
bool FS_OFF_save_mesh(const std::string &, const mesh::Mesh&);

#endif
