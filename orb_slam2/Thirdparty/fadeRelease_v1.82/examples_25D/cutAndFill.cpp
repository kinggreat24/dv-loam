#include <Fade_2D.h>

using namespace std;
using namespace GEOM_FADE25D;

// A terminal progress bar (optional)
class MyCAFProgBar:public GEOM_FADE25D::MsgBase
{
public:
	// Fade calls the update method with d={0.0,...,1.0}
	void update(MsgType ,const std::string& s,double d)
	{
		int numEq(int(d*20));
		std::string bar(numEq,'=');
		bar.append(20-numEq,' ');
		cout<<s<<" ["<<bar<<"] "<<d*100.0<<" %    \r"<<flush;
	}
};

// Returns the angle (0-180) between two normalized vectors
double getAngle(const Vector2& normVec0,const Vector2& normVec1)
{
	double cosPhi(normVec0*normVec1);
	if(cosPhi>1.0) return 0.0; // Robustness in case of numeric inaccuracy
	if(cosPhi<-1.0) return 180.0; // Robustness in case of numeric inaccuracy
	return acos(cosPhi)*180/3.14159;
}

// Optional peel predicate: Not absolutely required but very useful
class PeelPredicate:public UserPredicateT
{
public:
	bool operator()(const Triangle2* pT)
	{
		Vector2 nv(pT->getNormalVector()); // the triangle's normal vector
		Vector2 up(0,0,1);  // a vector pointing in z direction
		double angle(getAngle(nv,up)); // the angle between

		if(angle>88.0)
		{
			cout<<"This is your custom peel predicate. Deciding that a border triangle"<<endl;
			cout<<"shall be removed from the zone. Its angle to the XY plane is "<<angle<<"°"<<endl;
			cout<<"Its 2D area is "<<pT->getArea2D()<<", its 2.5D area="<<pT->getArea25D()<<endl;
			return true;
		}
		return false;
	}
};

int cutAndFill_main()
{
	std::cout<<"\n* Fade2.5D Demo - Cut & Fill"<<std::endl;
	std::cout<<"----------------------------"<<std::endl<<std::endl;

	// * 1 *   Create two random surfaces, 2 x ~10 000 triangles.
	std::cout<<"\nPreparing two surfaces...\n\n"<<std::endl;
	vector<Point2> vRndSurfacePoints0;
	vector<Point2> vRndSurfacePoints1;
	generateRandomSurfacePoints(
		70, // numPointsX
		70, // numPointsY
		20, // numCenters
		500,0,-60,1000,1000,60, // Bounds xmin,ymin,zmin,xmax,ymax,zmax
		vRndSurfacePoints0,	// Output vector
		1	// Seed
		);
	generateRandomSurfacePoints(
		70, // numPointsX
		70, // numPointsY
		1, // numCenters
		0,0,-30,1000,1000,30, // Bounds xmin,ymin,zmin,xmax,ymax,zmax
		vRndSurfacePoints1,	// Output vector
		2	// Seed
		);
	Fade_2D dt0;
	Fade_2D dt1;
	dt0.insert(vRndSurfacePoints0);
	dt1.insert(vRndSurfacePoints1);

	// * 2 *   Create Zones: One zone before the earthworks and one zone
	// after. The two zones do not need to match exactly, the algorithm
	// uses the overlapping area. Although the raw zones could be used
	// it is advised to peel off salient border triangles first whose
	// projection to the XY plane is almost zero. The behavior is
	// controlled by the custom PeelPredicate class above.
	Zone2* pZoneBeforeRaw(dt0.createZone(NULL,ZL_GLOBAL,false));
	Zone2* pZoneAfterRaw(dt1.createZone(NULL,ZL_GLOBAL,false));
	PeelPredicate pp;
	Zone2* pZoneBefore=peelOffIf(pZoneBeforeRaw,&pp,false);
	Zone2* pZoneAfter=peelOffIf(pZoneAfterRaw,&pp,false);

	cout<<"Surface before: "<<pZoneBefore->getNumberOfTriangles()<<" triangles"<<endl;
	cout<<"Surface after: "<<pZoneAfter->getNumberOfTriangles()<<" triangles"<<endl<<endl;



	// * 3 *   Visualize the two surfaces as *.obj files or as *.list
	// files for the Geomview viewer
	// pZoneBefore->showGeomview("zoneBefore.list","1 0 0 0.5");
	// pZoneAfter->showGeomview("zoneAfter.list","0 1 0 0.5");
	dt0.writeObj("zoneBefore.obj",pZoneBefore);
	dt1.writeObj("zoneAfter.obj",pZoneAfter);



	// * 4 *   Create a CutAndFill object, register a progress bar and
	// start the computations
	CutAndFill caf(pZoneBefore,pZoneAfter);
	MyCAFProgBar progBar=MyCAFProgBar();
	caf.subscribe(MSG_PROGRESS,&progBar);

	cout<<"Calling caf.go() now..."<<endl<<endl;
	bool bOK=caf.go();
	if(!bOK)
    {
		cout<<"Cut&Fill computation (caf.go) failed: Check if the input surfaces do overlap!"<<endl;
		cout<<"- Can't continue"<<endl;
		return 1;
	}

	// * 5 *   Fetch connected components and report
	cout<<"\nReport:"<<endl;
	cout<<"-------"<<endl;
	cout<<"Number of components: "<<caf.getNumberOfComponents()<<endl;
	for(size_t i=0;i<caf.getNumberOfComponents();++i)
	{
		CAF_Component* pComponent(caf.getComponent(i));
		cout<<*pComponent<<endl;
	}



	// * 6 *   Draw
	Visualizer2 vis("result.ps");
	caf.show(&vis);
	vis.writeFile();




	// * 7 *   Access the internal datastructures and write a facelist

	// The map mVtx2BeforeAfter contains for each vertex the heights in
	// the two input surfaces while pDiffZone contains the overlapping
	// part of the two input surfaces. The z-values of the zone vertices
	// correspond to the height difference (before minus after)
	std::map< Point2 *, std::pair< double, double > > mVtx2BeforeAfter;
	Zone2* pDiffZone(NULL);
    bool bOK2=caf.getDiffZone(pDiffZone,mVtx2BeforeAfter);
	if(!bOK2)
    {
		cout<<"caf.getDiffZone() failed: Did not get pDiffZone. Check your input data."<<endl;
		return 1;
	}

	// Assign indices to the vertices
	vector<Point2*> vZonePoints;
	pDiffZone->getVertices(vZonePoints);
	int counter(0);
    for(vector<Point2*>::iterator it(vZonePoints.begin());it!=vZonePoints.end();++it)
    {
        Point2* pVtx(*it);
        pVtx->setCustomIndex(counter++);
    }

	// Create a file
	const char* filename("facelist.txt");
	std::ofstream file(filename);
	if(file.is_open())
	{
		cout<<"\nwriting "<<filename<<endl;
	}
	else
	{
		cout<<filename<<" can't be written"<<endl;
		return 1;
	}

	// Write the vertices
    vector<Triangle2*> vTriangles;
    pDiffZone->getTriangles(vTriangles);
	file<<"Facelist: numPoints="<<vZonePoints.size()<<", numTriangles: "<<vTriangles.size()<<endl;
	for(vector<Point2*>::iterator it(vZonePoints.begin());it!=vZonePoints.end();++it)
    {
        Point2* pVtx(*it);
        int idx(pVtx->getCustomIndex());
        std::map< Point2 *, std::pair< double, double > >::iterator q_it(mVtx2BeforeAfter.find(pVtx));
        assert(q_it!=mVtx2BeforeAfter.end());
        double zBefore(q_it->second.first);
        double zAfter(q_it->second.second);
		file<<idx<<"\tx="<<pVtx->x()<<" y="<<pVtx->y()<<"\tzdiff="<<pVtx->z()<<" zBefore="<<zBefore<<" zAfter="<<zAfter<<endl;
    }

	// Write the corner indices of the triangles
    for(vector<Triangle2*>::iterator it(vTriangles.begin());it!=vTriangles.end();++it)
    {
        Triangle2* pT(*it);
        file<<pT->getCorner(0)->getCustomIndex()<<" ";
        file<<pT->getCorner(1)->getCustomIndex()<<" ";
        file<<pT->getCorner(2)->getCustomIndex()<<"\n";
    }
	file.close();

	return 0;
}


