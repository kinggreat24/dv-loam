#include <Fade_2D.h>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>

using namespace GEOM_FADE2D;
using namespace std;


int example4_main()
{
	std::cout<<"example4: Zones - Defined areas in a triangulation\n";
	std::cout<<"* Define zones in different ways:\n";
	std::cout<<"  0) area inside a ConstraintGraph2"<<endl;
	std::cout<<"  1) area outside a ConstraintGraph2"<<endl;
	std::cout<<"  2) area grown from a seed point"<<endl;
	std::cout<<"  3) global (all triangles)"<<endl;
	std::cout<<"  4) area of arbitrary triangles\n\n";

	// 1) Insert 4 points
	Fade_2D dt;
	dt.insert(Point2(-100,-100));
	dt.insert(Point2(+100,-100));
	dt.insert(Point2(+100,+270));
	dt.insert(Point2(-100,+270));

	// 2) Prepare and insert two constraint graphs
	std::vector<Point2> vConstraintPoints;
	vConstraintPoints.push_back(Point2(80,0));
	vConstraintPoints.push_back(Point2(49.8,62.5));
	vConstraintPoints.push_back(Point2(-17.8,78));
	vConstraintPoints.push_back(Point2(-72,34.7));
	vConstraintPoints.push_back(Point2(-72,-34.7));
	vConstraintPoints.push_back(Point2(-17.8,-78));
	vConstraintPoints.push_back(Point2(49.8,-62.5));

	std::vector<Segment2> vSegments1;
	std::vector<Segment2> vSegments2;
	for(size_t i=0;i<vConstraintPoints.size();++i)
	{
		Point2& p0(vConstraintPoints[i]);
		Point2& p1(vConstraintPoints[(i+1)%vConstraintPoints.size()]);
		vSegments1.push_back(Segment2(p0,p1));

		Point2 p0a(p0.x(),p0.y()+170);
		Point2 p1a(p1.x(),p1.y()+170);
		vSegments2.push_back(Segment2(p0a,p1a));
	}

	ConstraintGraph2* pCG1=dt.createConstraint(vSegments1,CIS_CONSTRAINED_DELAUNAY);
	ConstraintGraph2* pCG2=dt.createConstraint(vSegments2,CIS_CONSTRAINED_DELAUNAY);

	// 3) Apply and show the triangulation and constraints
	dt.applyConstraintsAndZones();
	dt.show("example4_constraints.ps");

	// Verify that the ConstraintGraph2 objects pCG1 and pCG2 are closed
	if(!pCG1->isPolygon() || !pCG2->isPolygon() )
	{
		std::cout<<"pCG1 and pCG2 must be closed polygons, stop"<<std::endl;
		return 1;
	}

	// 4) Create Zone2 objects in different ways, then retrieve the
	//    triangles and visualize the zones

	//    a) Use pCG and ZL_INSIDE to define a zone as the area inside pCG1
	Zone2* pZoneInside(dt.createZone(pCG1,ZL_INSIDE));
	pZoneInside->show("example4_zoneInside.ps",true,true); // all triangles=true, constraints=true

	//    b) Use pCG and ZL_OUTSIDE to define a zone as the area outside pCG1
	Zone2* pZoneOutside(dt.createZone(pCG1,ZL_OUTSIDE));
	pZoneOutside->show("example4_zoneOutside.ps",true,true);

	//    c) Use ConstraintGraph2 objects and an arbitrary seed point to
	//       grow a zone such that the growing stops at edges of the
	//       specified ConstraintGraph2 objects.
	vector<ConstraintGraph2*> vCG;
	vCG.push_back(pCG1);
	vCG.push_back(pCG2);
	Point2 seedPoint(-99,-99); // near the lower left corner
	Zone2* pZoneGrow(dt.createZone(vCG,ZL_GROW,seedPoint));
	pZoneGrow->show("example4_zoneGrow.ps",true,true);

	//    d) Use ZL_GLOBAL to create a zone containing all triangles
	Zone2* pZoneGlobal(dt.createZone(NULL,ZL_GLOBAL));
	pZoneGlobal->show("example4_zoneGlobal.ps",true,true);


	//    e) Zones do not need to be connected. Create a zone from
	//       a random set of triangles
	vector<Triangle2*> vT;
	dt.getTrianglePointers(vT);
	random_shuffle(vT.begin(),vT.end());
	vT.resize(vT.size()/2);
	Zone2* pZoneRandomTriangles(dt.createZone(vT));
	pZoneRandomTriangles->show("example4_zoneRandomTriangles.ps",true,true);
	return 0;
}


