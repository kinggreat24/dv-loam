#include <Fade_2D.h>

using namespace std;
using namespace GEOM_FADE25D;


class PeelPredicate:public UserPredicateT
{
public:
	bool operator()(const Triangle2* pT)
	{
		Vector2 nv(pT->getNormalVector());
		Vector2 up(0,0,1);

		double angle(-1);
		double cosPhi(nv*up);
		if(cosPhi>1.0) angle=0; // Robustness in case of numeric inaccuracy
			else if(cosPhi<-1.0) angle=180.0; // Robustness in case of numeric inaccuracy
				else angle=acos(cosPhi)*180/3.14159;

		if(angle>88.0)
		{
			cout<<endl;
			cout<<"This is your custom peel predicate. Deciding that a triangle at the"<<endl;
			cout<<"zone's border shall be removed. Its angle to the XY plane is "<<angle<<"°"<<endl;
			cout<<"Its 2D area is "<<pT->getArea2D()<<", its 2.5D area="<<pT->getArea25D()<<endl<<endl;
			return true;
		}
		return false;
	}
};

int removeBorderTriangles_main()
{
	std::cout<<"\n* Fade2.5D Demo - RemoveBorderTriangles"<<std::endl;
	std::cout<<"-----------------------------------------"<<std::endl<<std::endl;

	// * 1 *   Create 6 surface points
	cout<<"Creating 6 surface points"<<endl;
	vector<Point2> vSurfacePoints;
	vSurfacePoints.push_back(Point2(0,0,10));
	vSurfacePoints.push_back(Point2(0,50,0));
	vSurfacePoints.push_back(Point2(0,100,10));
	vSurfacePoints.push_back(Point2(20,0,10));
	vSurfacePoints.push_back(Point2(20,50,0));
	vSurfacePoints.push_back(Point2(20,100,10));

	// * 2 *   Triangulate and create a zone
	cout<<"Constructing pZone0"<<endl;
	Fade_2D dt0;
	dt0.insert(vSurfacePoints);
	Zone2* pZone0(dt0.createZone(NULL,ZL_GLOBAL,false));

	// * 3 *   Show
	pZone0->showGeomview("pZone0.list","1 0 0 0.5");
	cout<<"Zone0, number of triangles: "<<pZone0->getNumberOfTriangles()<<endl<<endl;

	// * 4 *   Perturbation
	cout<<"...so far everything is as expected. Now one of the previous points"<<endl;
	cout<<"is moved slightly inwards and the triangulation is constructed again. A Delaunay"<<endl;
	cout<<"triangulation is always convex and thus an additional triangle appears"<<endl;
	cout<<"at the border whose area in the XY plane is almost 0.\n"<<endl;
	vSurfacePoints[1]=Point2(1e-5,50,0);

	// * 5 *   Triangulate and create a zone again
	Fade_2D dt1;
	dt1.insert(vSurfacePoints);
	Zone2* pZone1(dt1.createZone(NULL,ZL_GLOBAL,false));

	// * 6 *   Show
	pZone1->showGeomview("pZone1.list","0 1 0 0.5");
	cout<<"Zone1, number of triangles: "<<pZone1->getNumberOfTriangles()<<endl<<endl;

	// * 7 *   Peel unwanted border triangles off
	cout<<"Getting rid of unwanted border triangles"<<endl;
	PeelPredicate pp;
	Zone2* pZone1peeled=peelOffIf(pZone1,&pp,false);

	pZone1peeled->showGeomview("pZone1peeled.list","0 0 1 0.5");
	cout<<"Zone1peeled, number of triangles: "<<pZone1peeled->getNumberOfTriangles()<<endl<<endl;


	return 0;

}


