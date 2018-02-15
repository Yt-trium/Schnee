#ifndef __MCUBES__
#define __MCUBES__

#include "vector.h"
#include "cloud.h"
#include "plane.h"

#include <vector>
#include <memory>

class Grid
{

public:

	std::vector<sVector3> corners;
	std::vector<float> values;

};

typedef std::shared_ptr<Grid> sGrid;

void MC_create_cubes(const PlaneCloud&,
                     const plane_cloud_index &,
                     std::vector<sGrid> &, float);

bool MC_is_point_in_cell(const Vector3&, const Vector3&, const float&);



#endif
