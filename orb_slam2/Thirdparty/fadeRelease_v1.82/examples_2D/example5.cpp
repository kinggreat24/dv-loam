#include <Fade_2D.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iomanip>
using namespace GEOM_FADE2D;
using namespace std;

template<class T_InType>
std::string toString(const T_InType& in)
{
	std::ostringstream oss;
	oss << in;
	return oss.str();
}

int example5_main()
{
	std::cout<<"example5: Zone operations - Boolean operations with zones\n";
	std::cout<<"* Union\n";
	std::cout<<"* Intersection\n";
	std::cout<<"* Difference\n";
	std::cout<<"* Symmetric Difference\n\n";

	cout<<"NOTE: When two constraint segments intersect then they are"<<endl;
	cout<<"subdivided at their intersection point. Be aware that the"<<endl;
	cout<<"exact intersection point does not necessarily exist in double"<<endl;
	cout<<"precision floating point arithmetic, so rounding can take place."<<endl;

	// * 1 *   Insert 4 points in order to create a box around the data
	Fade_2D dt;
	dt.insert(Point2(-100,-100));
	dt.insert(Point2(+100,-100));
	dt.insert(Point2(+100,+270));
	dt.insert(Point2(-100,+270));

	// * 2 *   Prepare two vectors of Segments
	int numPoints(10);
	std::vector<Point2> vConstraintPoints0;
	std::vector<Point2> vConstraintPoints1;
	generateCircle(numPoints,0,150,80,80,vConstraintPoints0);
	generateCircle(numPoints,0,40,60,130,vConstraintPoints1);

	std::vector<Segment2> vSegments1;
	std::vector<Segment2> vSegments2;
	for(size_t i=0;i<vConstraintPoints0.size();++i)
	{
		Point2& p0(vConstraintPoints0[i]);
		Point2& p1(vConstraintPoints0[(i+1)%vConstraintPoints0.size()]);
		vSegments1.push_back(Segment2(p0,p1));

		Point2& p2(vConstraintPoints1[i]);
		Point2& p3(vConstraintPoints1[(i+1)%vConstraintPoints1.size()]);
		vSegments2.push_back(Segment2(p2,p3));
	}

	// * 3 *   Use the segments to create two ConstraintGraphs. Then show what we have
	ConstraintGraph2* pCG1=dt.createConstraint(vSegments1,CIS_CONSTRAINED_DELAUNAY);
	ConstraintGraph2* pCG2=dt.createConstraint(vSegments2,CIS_CONSTRAINED_DELAUNAY);
	dt.show("example5_constraints.ps");

	// Make sure pCG1 and pCG2 are closed
	if(!pCG1->isPolygon() || !pCG2->isPolygon() )
	{
		std::cout<<"pCG1 and pCG2 must be closed polygons, stop"<<std::endl;
		return 1;
	}

	// * 4 *   Create two Zone2 objects using ZL_INSIDE:
	//         Note: Boolean operations require that the two zones belong to the same Fade_2D object!
	Zone2* pZone0(dt.createZone(pCG1,ZL_INSIDE));
	pZone0->show("example5_zone0.ps",true,true); // all triangles=true, show constraints=true

	Zone2* pZone1(dt.createZone(pCG2,ZL_INSIDE));
	pZone1->show("example5_zone1.ps",true,true);

	// * 5 *   BOOLEAN OPERATIONS

	//    a) Union operation
	Zone2* pZoneUnion(zoneUnion(pZone0,pZone1));
	pZoneUnion->show("example5_zoneUnion.ps",true,true);

	//    b) Intersection operation
	Zone2* pZoneIntersection(zoneIntersection(pZone0,pZone1));
	pZoneIntersection->show("example5_zoneIntersection.ps",true,true);

	//    c) Difference operation
	Zone2* pZoneDifference(zoneDifference(pZone0,pZone1));
	pZoneDifference->show("example5_zoneDifference.ps",true,true);

	//    d) Symmetric Difference operation
	Zone2* pZoneSymmetricDifference(zoneSymmetricDifference(pZone0,pZone1));
	pZoneSymmetricDifference->show("example5_zoneSymmetricDifference.ps",true,true);

	// * 6 *   Retrieve the (oriented but unordered) border edges of pZoneSymmetricDifference
	vector<Edge2> vBorders;
	pZoneSymmetricDifference->getBorderEdges(vBorders);
	Visualizer2 v("example5_symDiffBorders.ps");
	dt.show(&v,false); // show only the triangles
	v.addObject(vBorders,Color(CGREEN,0.5,true));
	v.writeFile();

	// * 7 *   Turn the border edges into polygons
	vector<Edge2> vRemainingEdges;
	vector<vector<Edge2> > vvEdges;
	edgesToPolygons(vBorders,vvEdges,vRemainingEdges);
	assert(vRemainingEdges.empty()); // All edges belong to polygons

	// * 8 *   Draw
	Visualizer2 polyVis("example5_polygons.ps");
	pZoneSymmetricDifference->show(&polyVis,false,false); // show only the triangles
	for(size_t poly=0;poly<vvEdges.size();++poly)
	{
		const vector<Edge2>& vPolygon(vvEdges[poly]);
		Color c(Color::getNextColorName(),1.0,false); // Next color from a (repeating) sequence
		for(size_t j=0;j<vPolygon.size();++j)
		{
			const Edge2& edge(vPolygon[j]);
			polyVis.addObject(edge,c);

			Point2* pSourcePoint(edge.getSrc());
			Label lab(*pSourcePoint,toString(j),true,16);
			polyVis.addObject(lab,c);
		}
	}
	polyVis.writeFile();
	return 0;
}
