#ifndef EDGE_H
#define EDGE_H

#include "plane.h"

class Edge
{
public:
    Edge();

    Plane p1;
    Plane p2;
    float w;

};

#endif // EDGE_H
