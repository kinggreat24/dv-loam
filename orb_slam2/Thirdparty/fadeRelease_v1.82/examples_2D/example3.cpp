#include <Fade_2D.h>
#include <stdlib.h>
#include <stdio.h>
using namespace GEOM_FADE2D;
using namespace std;

// Draws a triangulation (postscript output)
void draw(const std::string& name, Fade_2D& dt,ConstraintGraph2* pCG)
{
	Visualizer2 vis(name);
	dt.show(&vis);
	if(pCG!=NULL)
	{
			vector<Point2*> vPointsOfConstraintEdge;
			pCG->getPolygonVertices(vPointsOfConstraintEdge);
			for(size_t i=0;i+1<vPointsOfConstraintEdge.size();++i)
			{
				Point2* p0(vPointsOfConstraintEdge[i]);
				Point2* p1(vPointsOfConstraintEdge[i+1]);
				vis.addObject(Segment2(*p0,*p1),Color(1,0,0,0.01));
				vis.addObject(*p0,Color(0,0,1,0.1));
				vis.addObject(*p1,Color(0,0,1,0.1));
			}
			vis.writeFile();
	}

}

int example3_main()
{
	std::cout<<"\n";
	std::cout<<"example3: Constraints - Enforce constraint edges\n";

	// 1) Generate some input points
	std::vector<Point2> vInputPoints;
	vInputPoints.push_back(Point2(-100,-100));
	vInputPoints.push_back(Point2(+100,+100));
	vInputPoints.push_back(Point2(-50,-70));
	vInputPoints.push_back(Point2(-50,-30));
	vInputPoints.push_back(Point2(50,70));
	vInputPoints.push_back(Point2(50,30));

	// 2) Triangulate the points and show
	Fade_2D dt;
	dt.insert(vInputPoints);
	draw("example3_noConstraints.ps",dt,NULL);

	// 3) Prepare a vector of one or more edges to be enforced
	std::vector<Segment2> vSegments;
	vSegments.push_back(Segment2(vInputPoints[0],vInputPoints[1]));

	// 4) Insert the Constraint Segments
	// Note: Use always the recommended constraint insertion strategy
	// CIS_CONSTRAINED_DELAUNAY. This strategy does not subdivide the
	// constraint segments except when they intersect an existing
	// vertex or another constraint segment. 
	// Other insertion strategies exist also but they are deprecated
	// and only kept for backwards compatibility. Their behavior is
	// perfectly replaced by faster and more reliable methods, see
	// for example the functions ConstraintGraph2::makeDelaunay() and 
	// Fade_2D::drape().  
	ConstraintGraph2* pCG=dt.createConstraint(vSegments,CIS_CONSTRAINED_DELAUNAY);
	draw("example3_withConstraints.ps",dt,pCG);

	// 5) makeDelaunay() - an optional function call to subdivide a
	// ConstraintGraph in order to achieve better shaped triangles.
	// Segments smaller than $minLen are not subdivided. This parameter
	// is thought to prevent excessive subdivision in narrow settings.
	double minLen(0.1);
	pCG->makeDelaunay(minLen);
	draw("example3_makeDelaunay.ps",dt,pCG);

	return 0;
}



