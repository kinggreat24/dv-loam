#include <Fade_2D.h>

using namespace std;
using namespace GEOM_FADE25D;

void initialZone();
void strategy_default();
void strategy_growFactor();
void strategy_maxLength();
void strategy_customParameters();
void strategy_heightGuide();
void strategy_gridMeshing();
Zone2* createSimpleZone(Fade_2D& dt);
void createHeightGuide(Fade_2D& dt_guide);


int advancedMeshing_main()
{
	std::cout<<"\n* Fade2.5D Demo - advanced meshing"<<std::endl;
	std::cout<<"------------------------------------"<<std::endl<<std::endl;
	initialZone();
	strategy_default();
	strategy_growFactor();
	strategy_maxLength();
	strategy_gridMeshing();
	strategy_customParameters();
	strategy_heightGuide();
	return 0;
}

// Shows a simple zone
void initialZone()
{
	Fade_2D dt;
	Zone2* pZone=createSimpleZone(dt);
	pZone->show("initialZone.ps",true,true); // show all triangles=true, show constraints=true
}

// Uses MeshGenParams with all default parameters
void strategy_default()
{
	Fade_2D dt;
	Zone2* pZone=createSimpleZone(dt);
	MeshGenParams params(pZone);
	dt.refineAdvanced(&params);
	pZone->show("default.ps",true,true);
}

// Uses MeshGenParams and restricts the growFactor parameter
void strategy_growFactor()
{
	Fade_2D dt;
	Zone2* pZone=createSimpleZone(dt);

	MeshGenParams params(pZone);
	params.growFactor=5.0;
	dt.refineAdvanced(&params);
	pZone->show("growFactor.ps",true,true);
}

// Uses MeshGenParams and restricts the maximum edge length
void strategy_maxLength()
{
	Fade_2D dt;
	Zone2* pZone=createSimpleZone(dt);
	MeshGenParams params(pZone);
	params.maxEdgeLength=10.0;
	dt.refineAdvanced(&params);
	pZone->show("maxLength.ps",true,true);
}

void strategy_gridMeshing()
{
	Fade_2D dt;
	Zone2* pZone=createSimpleZone(dt);
	MeshGenParams params(pZone);
	params.gridLength=3.0;
	params.gridVector=Vector2(1.0,0.0,0.0);
//	params.gridVector=Vector2(1.0,0.3,0.0);

	dt.refineAdvanced(&params);
	pZone->show("gridMeshing.ps",true,true);
}


class CustomParameters:public MeshGenParams
{
public:
	CustomParameters(Zone2* pZone):MeshGenParams(pZone)
	{
	}
	double getMaxTriangleArea(Triangle2* pT)
	{
		Point2 barycenter(pT->getBarycenter());
		if(barycenter.x()<20 && barycenter.y()<40)
		{
			// Dense meshing in the lower left corner
			return 1.0;
		}
		else
		{
			// No density restriction otherwise
			return 10.0;
		}
	}
};

// Uses CustomParameters with a custom getMaxTriangleArea method
void strategy_customParameters()
{
	Fade_2D dt;
	Zone2* pZone=createSimpleZone(dt);

	CustomParameters params(pZone);
	dt.refineAdvanced(&params);
	pZone->show("customParameters.ps",true,true);
}


void strategy_heightGuide()
{
	Fade_2D dt;
	Zone2* pZoneSimple=createSimpleZone(dt);

	// Another triangulation is used as height guide
	Fade_2D dt_guide;
	createHeightGuide(dt_guide);

	MeshGenParams params(pZoneSimple);
	params.maxTriangleArea=10.0;
	params.pHeightGuideTriangulation=&dt_guide;
	dt.refineAdvanced(&params);
	dt.writeWebScene("mesh_with_height");
}


Zone2* createSimpleZone(Fade_2D& dt)
{
	// Create a shape
	std::vector<Point2> vConstraintPoints;
	vConstraintPoints.push_back(Point2(30,0,0));
	vConstraintPoints.push_back(Point2(80,0,0));
	vConstraintPoints.push_back(Point2(55,20,0));
	vConstraintPoints.push_back(Point2(100,20,0));
	vConstraintPoints.push_back(Point2(100,100,0));
	vConstraintPoints.push_back(Point2(54.5,22,0));
	vConstraintPoints.push_back(Point2(0,100,0));
	vConstraintPoints.push_back(Point2(0,20,0));
	vConstraintPoints.push_back(Point2(54,20,0));

	std::vector<Segment2> vSegments;
	for(size_t i=0;i<vConstraintPoints.size();++i)
	{
		Point2& p0(vConstraintPoints[i]);
		Point2& p1(vConstraintPoints[(i+1)%vConstraintPoints.size()]);
		vSegments.push_back(Segment2(p0,p1));
	}

	ConstraintGraph2* pCG(NULL);
	pCG=dt.createConstraint(vSegments,CIS_CONSTRAINED_DELAUNAY);
	dt.applyConstraintsAndZones();
	Zone2* pZone(dt.createZone(pCG,ZL_INSIDE));
	return pZone;

}


void createHeightGuide(Fade_2D& dt_guide)
{
	// We use createSimpleZone in order to insert the same
	// constraints but we are actually not interested in
	// the zone created by this method.
	createSimpleZone(dt_guide);

	// We insert a few more constraint edges. Their height
	// is different from 0.
	Point2 p0(15,30,20);
	Point2 p1(30,30,20);
	Point2 p2(30,45,20);
	Point2 p3(15,45,20);
	std::vector<Segment2> vSegments;
	vSegments.push_back(Segment2(p0,p1));
	vSegments.push_back(Segment2(p1,p2));
	vSegments.push_back(Segment2(p2,p3));
	vSegments.push_back(Segment2(p3,p0));
	dt_guide.createConstraint(vSegments,CIS_CONSTRAINED_DELAUNAY);
	dt_guide.applyConstraintsAndZones();
	dt_guide.writeWebScene("heightGuide");

}




