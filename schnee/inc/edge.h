#ifndef EDGE_H
#define EDGE_H

#include "plane.h"

/**
 * @brief Edge in a non-oriented graph
 */
class Edge
{
public:
    sPlane p1;
    sPlane p2;
    float w;
};

#endif // EDGE_H
