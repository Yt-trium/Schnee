#ifndef __FILE_SAVER__
#define __FILE_SAVER__

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

#endif
