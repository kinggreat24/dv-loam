#include <Fade_2D.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
using namespace GEOM_FADE2D;
using namespace std;


int example6_main()
{
	std::cout<<"\n";
	std::cout<<"example6: Mesh Generator - Delaunay meshing\n";
	std::cout<<"* Create a Zone2 object and refine the mesh\n";
	std::cout<<"* Visualize before and after\n\n";

	// 1) Insert 8 points
	Fade_2D dt;
	vector<Point2> vPoints;
	vPoints.push_back(Point2(40,1));
	vPoints.push_back(Point2(60,1));
	vPoints.push_back(Point2(70,70));
	vPoints.push_back(Point2(30,70));
	vPoints.push_back(Point2(0,0));
	vPoints.push_back(Point2(100,0));
	vPoints.push_back(Point2(100,100));
	vPoints.push_back(Point2(0,100));
	dt.insert(vPoints);

	// 2) Use the points 0,1,2,3 and 4,5,6,7 to create constraint graphs
	std::vector<Segment2> vSegments1;
	std::vector<Segment2> vSegments2;
	for(size_t i=0;i<4;++i)
	{
		Point2& p0(vPoints[i]);
		Point2& p1(vPoints[(i+1)%4]);
		vSegments1.push_back(Segment2(p0,p1));

		Point2& p0a(vPoints[4+i]);
		Point2& p1a(vPoints[4+(i+1)%4]);
		vSegments2.push_back(Segment2(p0a,p1a));

	}
	ConstraintGraph2* pCG1=dt.createConstraint(vSegments1,CIS_CONSTRAINED_DELAUNAY);
	ConstraintGraph2* pCG2=dt.createConstraint(vSegments2,CIS_CONSTRAINED_DELAUNAY);

	// 3) Apply and show the triangulation and constraint
	dt.applyConstraintsAndZones();
	dt.show("example6_constraint.ps");

	// 4) Create a Zone2 using ZL_GROW
	Point2 seedPoint(1,1);
	vector<ConstraintGraph2*> vCG;
	vCG.push_back(pCG1);
	vCG.push_back(pCG2);
	Zone2* pGrowZone=dt.createZone(vCG,ZL_GROW,seedPoint);

	// 5) Extract the triangles of pZone
	std::vector<Triangle2*> vTriangles;
	pGrowZone->getTriangles(vTriangles);
	std::cout<<"pZone consists of "<<vTriangles.size()<<" triangles"<<std::endl;
	pGrowZone->show("example6_originalZone.ps",false,true); // all triangles=false, show constraints=true

	// Meshing, the parameters are
	// * zone pointer, pZone
	// * minimum interior angle per triangle, <30
	// * minimum length (no refinement below that edge length), some reasonable value
	// * maximum length (largest allowed output edge length), some reasonable value
	// * allow that constraint edges are splitted, normally true

	// Note: The mesh refinement algorithms expect zones with zoneLocation ZL_INSIDE
	// or ZL_BOUNDED. Before version v1.39 other types where also accepted but that
	// was not fully correct. An easy to use method has been implemented to convert
	// other types of zones to bounded zones like shown below:
	Zone2* pBoundedZone(pGrowZone->convertToBoundedZone());

	dt.refine(pBoundedZone,27,0.01,15,true);

	std::vector<Triangle2*> vTriangles2;
	pBoundedZone->getTriangles(vTriangles2);
	std::cout<<"\nAfter refinement: \nZone consists of "<<vTriangles2.size()<<" triangles"<<std::endl;
	pBoundedZone->show("example6_refinedZone.ps",false,true);


	return 0;
}

