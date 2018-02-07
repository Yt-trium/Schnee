#ifndef __PLANE__
#define __PLANE__

#include "vector.h"

#include <memory>

/**
 * @brief A 3D Plane reprensation.
 * Based on a point (center) and a normal.
 */
class Plane
{
	public sVector3 center;
	public sVector3 normal;
};

typedef std::shared_ptr<Plane> sPlane;

#endif
