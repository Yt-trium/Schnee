#define CATCH_CONFIG_MAIN

#include <vector>
#include <queue>

#include "catch.hpp"

#include "kruskal.h"
#include "cloud.h"
#include "marching_cubes.h"
#include "vector.h"

TEST_CASE("Grid edges")
{
	// Test that in a 2 by 2 grid, vertices and edges are correctly shared

	Vector3 min(0, 0, 0);
	Vector3 max(10, 10, 10);
	float cell_size = 15;
	sGrid grid = std::make_shared<Grid>(min, max, cell_size);

	REQUIRE(grid->sizeX() == 2);
	REQUIRE(grid->sizeY() == 2);
	REQUIRE(grid->sizeZ() == 2);

	const std::vector<sCell> & cells = grid->cells();
	REQUIRE(cells.size() == 0);
	grid->create_cells();
	REQUIRE(cells.size() == 2 * 2 * 2);

	SECTION("Test point values")
	{
		sCell cell = cells[0];
        REQUIRE(*(cell->corners[0].get()) == Vector3(-cell_size, -cell_size, -cell_size));

	}

    SECTION("Test common points in (0, 0, 0)")
	{
		sCell cell = cells[0];
        REQUIRE(cell->corners[0] != cell->corners[1]);
        REQUIRE(cell->corners[1] != cell->corners[2]);
        REQUIRE(cell->corners[2] != cell->corners[3]);
        REQUIRE(cell->corners[3] != cell->corners[4]);
        REQUIRE(cell->corners[4] != cell->corners[5]);
        REQUIRE(cell->corners[5] != cell->corners[6]);
        REQUIRE(cell->corners[6] != cell->corners[7]);
    }

    SECTION("Test common corners with (0, 0, 0) and (1, 0, 0)")
	{
		sCell cell = cells[0];
		sCell other = cells[1];

        REQUIRE(cell->corners[1] == other->corners[0]);
        REQUIRE(cell->corners[5] == other->corners[4]);
        REQUIRE(cell->corners[6] == other->corners[7]);
        REQUIRE(cell->corners[2] == other->corners[3]);
	}

    SECTION("Test common corners with (0, 0, 0) and (0, 1, 0)")
	{
		sCell cell = cells[0];
		sCell other = cells[2];

        REQUIRE(cell->corners[4] == other->corners[0]);
        REQUIRE(cell->corners[5] == other->corners[1]);
        REQUIRE(cell->corners[6] == other->corners[2]);
        REQUIRE(cell->corners[7] == other->corners[3]);
	}

    SECTION("Test common corners with (0, 0, 0) and (0, 0, 1)")
	{
		sCell cell = cells[0];
		sCell other = cells[4];

        REQUIRE(cell->corners[2] == other->corners[1]);
        REQUIRE(cell->corners[3] == other->corners[0]);
        REQUIRE(cell->corners[6] == other->corners[5]);
        REQUIRE(cell->corners[7] == other->corners[4]);
	}

    SECTION("Test common corners with (0, 0, 0) and (1, 1, 0)")
	{
		sCell cell = cells[0];
		sCell other = cells[3];

        REQUIRE(cell->corners[5] == other->corners[0]);
        REQUIRE(cell->corners[6] == other->corners[3]);
    }

    SECTION("Test common corners with (0, 0, 0) and (0, 1, 1)")
	{
		sCell cell = cells[0];
		sCell other = cells[6];

        REQUIRE(cell->corners[6] == other->corners[1]);
        REQUIRE(cell->corners[7] == other->corners[0]);
    }

    SECTION("Test common corners with (0, 0, 0) and (1, 0, 1)")
	{
		sCell cell = cells[0];
		sCell other = cells[5];

        REQUIRE(cell->corners[2] == other->corners[0]);
        REQUIRE(cell->corners[6] == other->corners[4]);
    }

    SECTION("Test common corners with (1, 0, 0) and (1, 1, 0)")
	{
		sCell cell = cells[1];
		sCell other = cells[3];

        REQUIRE(cell->corners[4] == other->corners[0]);
        REQUIRE(cell->corners[5] == other->corners[1]);
        REQUIRE(cell->corners[6] == other->corners[2]);
        REQUIRE(cell->corners[7] == other->corners[3]);
	}

    SECTION("Test common corners with (1, 0, 0) and (0, 1, 0)")
	{
		sCell cell = cells[1];
		sCell other = cells[2];

        REQUIRE(cell->corners[4] == other->corners[1]);
        REQUIRE(cell->corners[7] == other->corners[2]);
    }

    SECTION("Test common corners with (1, 0, 0) and (1, 1, 1)")
	{
		sCell cell = cells[1];
		sCell other = cells[7];

        REQUIRE(cell->corners[6] == other->corners[1]);
        REQUIRE(cell->corners[7] == other->corners[0]);
    }

    SECTION("Test common corners with (1, 0, 0) and (1, 0, 1)")
	{
		sCell cell = cells[1];
		sCell other = cells[5];

        REQUIRE(cell->corners[2] == other->corners[1]);
        REQUIRE(cell->corners[3] == other->corners[0]);
        REQUIRE(cell->corners[6] == other->corners[5]);
        REQUIRE(cell->corners[7] == other->corners[4]);
	}

    SECTION("Test common corners with (1, 0, 0) and (0, 0, 1)")
	{
		sCell cell = cells[1];
		sCell other = cells[4];

        REQUIRE(cell->corners[3] == other->corners[1]);
        REQUIRE(cell->corners[7] == other->corners[5]);
    }

    SECTION("Test common corners with (0, 1, 0) and (1, 1, 0)")
	{
		sCell cell = cells[2];
		sCell other = cells[3];

        REQUIRE(cell->corners[1] == other->corners[0]);
        REQUIRE(cell->corners[2] == other->corners[3]);
        REQUIRE(cell->corners[5] == other->corners[4]);
        REQUIRE(cell->corners[6] == other->corners[7]);
	}

    // Needs to be finished here

    SECTION("Test common corners with (0, 1, 0) and (0, 1, 1)")
	{
		sCell cell = cells[2];
		sCell other = cells[6];

        REQUIRE(cell->corners[2] == other->corners[1]);
        REQUIRE(cell->corners[3] == other->corners[0]);
        REQUIRE(cell->corners[6] == other->corners[5]);
        REQUIRE(cell->corners[7] == other->corners[4]);
	}

    SECTION("Test common corners with (0, 1, 0) and (0, 0, 1)")
	{
		sCell cell = cells[2];
		sCell other = cells[4];

        REQUIRE(cell->corners[2] == other->corners[5]);
        REQUIRE(cell->corners[3] == other->corners[4]);
	}
    SECTION("Test common corners with (0, 1, 0) and (1, 1, 1)")
	{
		sCell cell = cells[2];
		sCell other = cells[7];

        REQUIRE(cell->corners[2] == other->corners[0]);
        REQUIRE(cell->corners[6] == other->corners[4]);
	}

    SECTION("Test common corners with (1, 1, 0) and (0, 1, 1)")
	{
		sCell cell = cells[3];
		sCell other = cells[6];

        REQUIRE(cell->corners[3] == other->corners[1]);
        REQUIRE(cell->corners[7] == other->corners[5]);
	}

    SECTION("Test common corners with (1, 1, 0) and (1, 1, 1)")
	{
		sCell cell = cells[3];
		sCell other = cells[7];

        REQUIRE(cell->corners[2] == other->corners[1]);
        REQUIRE(cell->corners[3] == other->corners[0]);
        REQUIRE(cell->corners[6] == other->corners[5]);
        REQUIRE(cell->corners[7] == other->corners[4]);
	}

    SECTION("Test common corners with (1, 1, 0) and (1, 0, 1)")
	{
		sCell cell = cells[3];
		sCell other = cells[5];

        REQUIRE(cell->corners[2] == other->corners[5]);
        REQUIRE(cell->corners[3] == other->corners[4]);
	}

    SECTION("Test common corners with (0, 1, 1) and (1, 0, 1)")
	{
		sCell cell = cells[6];
		sCell other = cells[5];

        REQUIRE(cell->corners[2] == other->corners[7]);
        REQUIRE(cell->corners[1] == other->corners[4]);
	}

    SECTION("Test common corners with (0, 1, 1) and (1, 1, 1)")
	{
		sCell cell = cells[6];
		sCell other = cells[7];

        REQUIRE(cell->corners[1] == other->corners[0]);
        REQUIRE(cell->corners[2] == other->corners[3]);
        REQUIRE(cell->corners[5] == other->corners[4]);
        REQUIRE(cell->corners[6] == other->corners[7]);
	}

    SECTION("Test common corners with (0, 1, 1) and (0, 0, 1)")
	{
		sCell cell = cells[6];
		sCell other = cells[4];

        REQUIRE(cell->corners[0] == other->corners[4]);
        REQUIRE(cell->corners[1] == other->corners[5]);
        REQUIRE(cell->corners[2] == other->corners[6]);
        REQUIRE(cell->corners[3] == other->corners[7]);
	}

}

TEST_CASE("Grid unique points all unique")
{
	Vector3 min(0, 0, 0);
	Vector3 max(100, 100, 100);
	float cell_size = 15;
	sGrid grid = std::make_shared<Grid>(min, max, cell_size);
	grid->create_cells();

	// Get corners
	const std::vector<sCellPoint> & points = grid->uniquePoints();

	// Check if all points are unique
	std::vector<CellPoint *> uniquePoints;
	for(int i = 0; i < points.size(); ++i)
	{
		CellPoint * p = points[i].get();
		if(std::find(uniquePoints.begin(), uniquePoints.end(), p) == uniquePoints.end())
		{
            uniquePoints.push_back(p);
		}
	}

	REQUIRE(points.size() == uniquePoints.size());
}

TEST_CASE("Grid unique points")
{

	Vector3 min(0, 0, 0);
	Vector3 max(100, 100, 100);
	float cell_size = 15;
	sGrid grid = std::make_shared<Grid>(min, max, cell_size);
	grid->create_cells();

	const std::vector<sCell> & cells = grid->cells();
	// Count number of unique points
	size_t nbpoint = 0;
	std::vector<CellPoint *> uniquePoints;

    CellPoint * va;
	for(int i = 0; i < cells.size(); ++i)
	{
		const sCell & c = cells[i];
        for(int j = 0; j < c->corners.size(); ++j)
		{
            va = c->corners.at(j).get();

			if(std::find(uniquePoints.begin(), uniquePoints.end(), va) == uniquePoints.end())
			{
				uniquePoints.push_back(va);
			}
        }
	}

	// Get result
	const std::vector<sCellPoint> & points = grid->uniquePoints();

    REQUIRE(points.size() == uniquePoints.size());
}

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

    REQUIRE(k.addEdge(1,3,1) == false);
    REQUIRE(k.addEdge(3,1,1) == false);
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
