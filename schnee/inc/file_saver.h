#ifndef __FILE_SAVER__
#define __FILE_SAVER__

#include "vector.h"
#include "plane.h"

#include <string>
#include <vector>

/**
 * @brief Save points into an OFF file
 * @param       File path
 * @param       Point list
 * @return True for success
 */
bool FS_OFF_save_points(const std::string &, const std::vector<sVector3> &);

bool FS_OFF_save_planes(const std::string &, const std::vector<sPlane> &, float = 1.0f);

bool FS_OFF_save_planes_normals(const std::string &, const std::vector<sPlane> &, size_t = 5, float = 1.0f);

#endif
