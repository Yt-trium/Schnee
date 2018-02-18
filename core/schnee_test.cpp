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
		REQUIRE(*(cell->edges[4]->va.get()) == Vector3(0, 0, 0));

	}

	SECTION("Test common edges in (0, 0, 0)")
	{
		sCell cell = cells[0];
		REQUIRE(cell->edges[0] != cell->edges[8]);
		REQUIRE(cell->edges[1] != cell->edges[0]);
		REQUIRE(cell->edges[3] != cell->edges[10]);
		REQUIRE(cell->edges[4] != cell->edges[7]);
		REQUIRE(cell->edges[5] != cell->edges[4]);
		REQUIRE(cell->edges[6] != cell->edges[11]);
		REQUIRE(cell->edges[7] != cell->edges[10]);
		REQUIRE(cell->edges[8] != cell->edges[3]);
		REQUIRE(cell->edges[9] != cell->edges[0]);
		REQUIRE(cell->edges[10] != cell->edges[7]);
		REQUIRE(cell->edges[11] != cell->edges[6]);
	}

	SECTION("Test common edges with (0, 0, 0) and (1, 0, 0)")
	{
		sCell cell = cells[0];
		sCell other = cells[1];

		REQUIRE(cell->edges[6] == other->edges[4]);
		REQUIRE(cell->edges[2] == other->edges[0]);
		REQUIRE(cell->edges[10] == other->edges[8]);
		REQUIRE(cell->edges[11] == other->edges[9]);
	}

	SECTION("Test common edges with (0, 0, 0) and (0, 1, 0)")
	{
		sCell cell = cells[0];
		sCell other = cells[2];

		REQUIRE(cell->edges[7] == other->edges[5]);
		REQUIRE(cell->edges[3] == other->edges[1]);
		REQUIRE(cell->edges[8] == other->edges[9]);
		REQUIRE(cell->edges[10] == other->edges[11]);
	}

	SECTION("Test common edges with (0, 0, 0) and (0, 0, 1)")
	{
		sCell cell = cells[0];
		sCell other = cells[4];

		REQUIRE(cell->edges[1] == other->edges[5]);
		REQUIRE(cell->edges[2] == other->edges[6]);
		REQUIRE(cell->edges[0] == other->edges[4]);
		REQUIRE(cell->edges[3] == other->edges[7]);
	}

	SECTION("Test common edges with (0, 0, 0) and (1, 1, 0)")
	{
		sCell cell = cells[0];
		sCell other = cells[3];

		REQUIRE(cell->edges[10] == other->edges[9]);
	}

	SECTION("Test common edges with (0, 0, 0) and (0, 1, 1)")
	{
		sCell cell = cells[0];
		sCell other = cells[6];

		REQUIRE(cell->edges[3] == other->edges[5]);
	}

	SECTION("Test common edges with (0, 0, 0) and (1, 0, 1)")
	{
		sCell cell = cells[0];
		sCell other = cells[5];

		REQUIRE(cell->edges[2] == other->edges[4]);
	}

	SECTION("Test common edges with (1, 0, 0) and (1, 1, 0)")
	{
		sCell cell = cells[1];
		sCell other = cells[3];

		REQUIRE(cell->edges[3] == other->edges[1]);
		REQUIRE(cell->edges[10] == other->edges[11]);
		REQUIRE(cell->edges[7] == other->edges[5]);
		REQUIRE(cell->edges[8] == other->edges[9]);
	}

	SECTION("Test common edges with (1, 0, 0) and (0, 1, 0)")
	{
		sCell cell = cells[1];
		sCell other = cells[2];

		REQUIRE(cell->edges[8] == other->edges[11]);
	}

	SECTION("Test common edges with (1, 0, 0) and (1, 1, 1)")
	{
		sCell cell = cells[1];
		sCell other = cells[7];

		REQUIRE(cell->edges[3] == other->edges[5]);
	}

	SECTION("Test common edges with (1, 0, 0) and (1, 0, 1)")
	{
		sCell cell = cells[1];
		sCell other = cells[5];

		REQUIRE(cell->edges[3] == other->edges[7]);
		REQUIRE(cell->edges[2] == other->edges[6]);
		REQUIRE(cell->edges[0] == other->edges[4]);
		REQUIRE(cell->edges[1] == other->edges[5]);
	}

	SECTION("Test common edges with (1, 0, 0) and (0, 0, 1)")
	{
		sCell cell = cells[1];
		sCell other = cells[4];

		REQUIRE(cell->edges[0] == other->edges[6]);
	}

	SECTION("Test common edges with (0, 1, 0) and (1, 1, 0)")
	{
		sCell cell = cells[2];
		sCell other = cells[3];

		REQUIRE(cell->edges[6] == other->edges[4]);
		REQUIRE(cell->edges[2] == other->edges[0]);
		REQUIRE(cell->edges[10] == other->edges[8]);
		REQUIRE(cell->edges[11] == other->edges[9]);
	}

	SECTION("Test common edges with (0, 1, 0) and (0, 1, 1)")
	{
		sCell cell = cells[2];
		sCell other = cells[6];

		REQUIRE(cell->edges[3] == other->edges[7]);
		REQUIRE(cell->edges[2] == other->edges[6]);
		REQUIRE(cell->edges[0] == other->edges[4]);
		REQUIRE(cell->edges[1] == other->edges[5]);
	}

	SECTION("Test common edges with (0, 1, 0) and (0, 0, 1)")
	{
		sCell cell = cells[2];
		sCell other = cells[4];

		REQUIRE(cell->edges[1] == other->edges[7]);
	}
	SECTION("Test common edges with (0, 1, 0) and (1, 1, 1)")
	{
		sCell cell = cells[2];
		sCell other = cells[7];

		REQUIRE(cell->edges[2] == other->edges[4]);
	}

	SECTION("Test common edges with (1, 1, 0) and (0, 1, 1)")
	{
		sCell cell = cells[3];
		sCell other = cells[6];

		REQUIRE(cell->edges[0] == other->edges[6]);
	}

	SECTION("Test common edges with (1, 1, 0) and (1, 1, 1)")
	{
		sCell cell = cells[3];
		sCell other = cells[7];

		REQUIRE(cell->edges[3] == other->edges[7]);
		REQUIRE(cell->edges[2] == other->edges[6]);
		REQUIRE(cell->edges[0] == other->edges[4]);
		REQUIRE(cell->edges[1] == other->edges[5]);
	}

	SECTION("Test common edges with (1, 1, 0) and (1, 0, 1)")
	{
		sCell cell = cells[3];
		sCell other = cells[5];

		REQUIRE(cell->edges[1] == other->edges[7]);
	}

	SECTION("Test common edges with (0, 1, 1) and (1, 0, 1)")
	{
		sCell cell = cells[6];
		sCell other = cells[5];

		REQUIRE(cell->edges[11] == other->edges[8]);
	}

	SECTION("Test common edges with (0, 1, 1) and (1, 1, 1)")
	{
		sCell cell = cells[6];
		sCell other = cells[7];

		REQUIRE(cell->edges[6] == other->edges[4]);
		REQUIRE(cell->edges[2] == other->edges[0]);
		REQUIRE(cell->edges[10] == other->edges[8]);
		REQUIRE(cell->edges[11] == other->edges[9]);
	}

	SECTION("Test common edges with (0, 1, 1) and (0, 0, 1)")
	{
		sCell cell = cells[6];
		sCell other = cells[4];

		REQUIRE(cell->edges[9] == other->edges[8]);
		REQUIRE(cell->edges[1] == other->edges[3]);
		REQUIRE(cell->edges[5] == other->edges[7]);
		REQUIRE(cell->edges[11] == other->edges[10]);
	}

	SECTION("Test all uncommon cells")
	{

		std::pair<int, int> uncommons [] =
		{
		    {0, 7}, {1, 6}, {2, 5}, {3, 4}
		};

		for(int c = 0; c < 4; c++)
		{
            sCell cell = cells[uncommons[c].first];
            sCell other = cells[uncommons[c].second];

            for(int i = 0; i < 12; i ++)
            {
                for(int j = 0; j < 12; j++)
                {
                    REQUIRE(cell->edges[i] != other->edges[j]);
                }
            }
		}
	}

	SECTION("Test common points in all cells")
	{
		for(int i = 0; i < cells.size(); i++)
		{
			std::cout << "Testing cell number " << i << "\n";
            REQUIRE(cells[i]->edges[0]->va == cells[i]->edges[1]->va);
            REQUIRE(cells[i]->edges[0]->vb == cells[i]->edges[8]->va);
            REQUIRE(cells[i]->edges[1]->va == cells[i]->edges[0]->va);
            REQUIRE(cells[i]->edges[1]->vb == cells[i]->edges[2]->va);
            REQUIRE(cells[i]->edges[3]->va == cells[i]->edges[8]->va);
            REQUIRE(cells[i]->edges[3]->vb == cells[i]->edges[10]->va);
            REQUIRE(cells[i]->edges[4]->va == cells[i]->edges[9]->vb);
            REQUIRE(cells[i]->edges[4]->vb == cells[i]->edges[7]->va);
            REQUIRE(cells[i]->edges[5]->va == cells[i]->edges[4]->va);
            REQUIRE(cells[i]->edges[5]->vb == cells[i]->edges[6]->va);
            REQUIRE(cells[i]->edges[6]->va == cells[i]->edges[11]->vb);
            REQUIRE(cells[i]->edges[6]->vb == cells[i]->edges[10]->vb);
            REQUIRE(cells[i]->edges[7]->va == cells[i]->edges[8]->vb);
            REQUIRE(cells[i]->edges[7]->vb == cells[i]->edges[6]->vb);
            REQUIRE(cells[i]->edges[8]->va == cells[i]->edges[3]->va);
            REQUIRE(cells[i]->edges[8]->vb == cells[i]->edges[7]->va);
            REQUIRE(cells[i]->edges[9]->va == cells[i]->edges[0]->va);
            REQUIRE(cells[i]->edges[9]->vb == cells[i]->edges[5]->va);
            REQUIRE(cells[i]->edges[10]->va == cells[i]->edges[2]->vb);
            REQUIRE(cells[i]->edges[10]->vb == cells[i]->edges[7]->vb);
            REQUIRE(cells[i]->edges[11]->va == cells[i]->edges[1]->vb);
            REQUIRE(cells[i]->edges[11]->vb == cells[i]->edges[6]->va);
        }
    }

}

TEST_CASE("Grid unique points all unique")
{
	Vector3 min(0, 0, 0);
	Vector3 max(100, 100, 100);
	float cell_size = 15;
	sGrid grid = std::make_shared<Grid>(min, max, cell_size);
	grid->create_cells();

	// Get result
	std::queue<sCellPoint> points;
	grid->getUniquePoints(points);

	// Check if all points are unique
	size_t size = points.size();
	std::vector<sCellPoint> uniquePoints;
	while(points.size() > 0)
	{
		sCellPoint p = points.front();
		points.pop();
		if(std::find(uniquePoints.begin(), uniquePoints.end(), p) == uniquePoints.end())
		{
            uniquePoints.push_back(p);
		}
	}

	REQUIRE(size == uniquePoints.size());
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

	CellPoint * va, * vb;
	for(int i = 0; i < cells.size(); ++i)
	{
		const sCell & c = cells[i];
		for(int j = 0; j < 12; ++j)
		{
			const sCellEdge & e = c->edges.at(j);
			va = e->va.get();
			vb = e->vb.get();

			if(std::find(uniquePoints.begin(), uniquePoints.end(), va) == uniquePoints.end())
			{
				uniquePoints.push_back(va);
				nbpoint++;
			}

			if(std::find(uniquePoints.begin(), uniquePoints.end(), vb) == uniquePoints.end())
			{
				uniquePoints.push_back(vb);
				nbpoint++;
			}
		}
	}

	// Get result
	std::queue<sCellPoint> points;
	grid->getUniquePoints(points);

	REQUIRE(points.size() == nbpoint);
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
