#ifndef ORIENTATIONFIXER_H
#define ORIENTATIONFIXER_H

#include "vector.h"

#include "cloud.h"
#include "plane.h"

void orientationFixer(PlaneCloud &plc, const plane_cloud_index &index, const int &k);

#endif // ORIENTATIONFIXER_H
