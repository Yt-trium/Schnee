#define CATCH_CONFIG_MAIN

#include <vector>

#include "catch.hpp"

#include "kruskal.h"
#include "cloud.h"

TEST_CASE( "Kruskal" ) {
    Kruskal k;
    std::vector<Kruskal::KEdge> r;
    k.setVertices(4);
    k.setEdges(5);

    k.addEdge(0,1,10);
    k.addEdge(0,2,6);
    k.addEdge(0,3,5);
    k.addEdge(1,3,15);
    k.addEdge(2,3,4);

    r = k.MST();

    REQUIRE( r.size() == 3 );

    SECTION( "edge 1" ) {
        REQUIRE( r.at(0).src == 2 );
        REQUIRE( r.at(0).dst == 3 );
        REQUIRE( r.at(0).weight == 4 );
    }
    SECTION( "edge 2" ) {
        REQUIRE( r.at(1).src == 0 );
        REQUIRE( r.at(1).dst == 3 );
        REQUIRE( r.at(1).weight == 5 );
    }
    SECTION( "edge 3" ) {
        REQUIRE( r.at(2).src == 0 );
        REQUIRE( r.at(2).dst == 1 );
        REQUIRE( r.at(2).weight == 10 );
    }
}

/*
TEST_CASE( "PC_build_planes", "[point_cloud.h]" ) {
    PointCloud pc;
    std::vector<sPlane> planes;

    pc.points.emplace_back(new Vector3(1, 0, 0));
    pc.points.emplace_back(new Vector3(2, 0, 0));
    pc.points.emplace_back(new Vector3(3, 0, 0));
    pc.points.emplace_back(new Vector3(4, 0, 0));
    pc.points.emplace_back(new Vector3(8, 0, 0));
    pc.points.emplace_back(new Vector3(10, 0, 0));

    PTC_build_planes(pc, planes, 2);

    for(int i = 0; i < planes.size(); i++)
    {
        std::cout << planes[i]->center->x << " "
                  << planes[i]->center->y << " "
                  << planes[i]->center->z << std::endl;
    }
}

// Example test with std::vector
TEST_CASE( "vectors can be sized and resized", "[vector]" ) {

    std::vector<int> v( 5 );

    REQUIRE( v.size() == 5 );
    REQUIRE( v.capacity() >= 5 );

    SECTION( "resizing bigger changes size and capacity" ) {
        v.resize( 10 );

        REQUIRE( v.size() == 10 );
        REQUIRE( v.capacity() >= 10 );
    }
    SECTION( "resizing smaller changes size but not capacity" ) {
        v.resize( 0 );

        REQUIRE( v.size() == 0 );
        REQUIRE( v.capacity() >= 5 );
    }
    SECTION( "reserving bigger changes capacity but not size" ) {
        v.reserve( 10 );

        REQUIRE( v.size() == 5 );
        REQUIRE( v.capacity() >= 10 );
    }
    SECTION( "reserving smaller does not change size or capacity" ) {
        v.reserve( 0 );

        REQUIRE( v.size() == 5 );
        REQUIRE( v.capacity() >= 5 );
    }
}
*/
