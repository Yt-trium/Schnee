#ifndef __MESH__
#define __MESH__

#include "vector.h"

#include <memory>
#include <vector>

namespace mesh {

class Face
{
  public:
    std::vector<sVector3> points;
};

typedef std::shared_ptr<Face> sFace;

class Mesh
{
  public:
    std::vector<sFace> faces;
};

}

#endif
