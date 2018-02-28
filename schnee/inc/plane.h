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
public:
	sVector3 center;
	sVector3 normal;
	/**
	 * @brief Direction vector of the plane. Used only for debug.
	 */
	sVector3 u;
	sVector3 v;
};

typedef std::shared_ptr<Plane> sPlane;

#endif
