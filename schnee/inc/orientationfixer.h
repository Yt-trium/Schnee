#ifndef ORIENTATIONFIXER_H
#define ORIENTATIONFIXER_H

#include <cmath>

#include "vector.h"
#include "cloud.h"
#include "plane.h"
#include "kruskal.h"

/**
 * @brief orientationFixer
 * @param plc
 * @param index
 * @param k
 */
void orientationFixer(PlaneCloud &plc, const plane_cloud_index &index, const int &k);

#endif // ORIENTATIONFIXER_H
