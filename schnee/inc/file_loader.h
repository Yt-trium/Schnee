#ifndef __FILE_LOADER__
#define __FILE_LOADER__

#include "vector.h"

#include <string>
#include <vector>

/**
 * @brief Load points from a file
 * @param       File path
 * @param out   Out point list
 * @return True for success
 */
bool FL_OFF_load_points(const std::string &, std::vector<sVector3> &);

#endif
