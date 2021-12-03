#include <Fade_2D.h>
#include <Visualizer3.h>
using namespace std;
using namespace GEOM_FADE25D;
void isoContours(std::vector<Triangle2*>& vTriangles);
void createTerrain(Fade_2D& dt);


// main()
int terrain_main()
{
	std::cout<<"\n* Fade2.5D Demo - Terrain triangulation, constraints, ISO contours"<<std::endl;
	std::cout<<"------------------------------------------------------------------"<<std::endl<<std::endl;

	// * 1 *   Create a random surface
	Fade_2D dt;
	createTerrain(dt);

	// * 2 *   Create Segments ("Breaklines")
	std::vector<Segment2> vSegments;
	std::vector<Point2> vCirclePoints;
	generateCircle(5,50.0,50.0,50.0,45.0,45.0,vCirclePoints);
	for(size_t i=0;i<vCirclePoints.size();++i)
	{
		vSegments.push_back(Segment2(vCirclePoints[i],vCirclePoints[(i+1)%vCirclePoints.size()]));
	}


	// * 3 *   Insert Segments ("Breaklines"), please choose a mode: ...

	int yourChosenMode(2); // Use one of {0,1,2,3}

	vector<Segment2> vDrapedSegments;
	switch(yourChosenMode)
	{
		case 0: // MODE 0
		{
			// This method does not subdivide the segments except when
			// they intersect an existing vertex or another constraint.
			dt.createConstraint(vSegments,CIS_CONSTRAINED_DELAUNAY);
			break;
		}
		case 1: // MODE 1
		{
			// This mode subdivides the segments in order to achieve better 
			// shaped triangles. The minLen parameter is thought to avoid 
			// excessive subdivision in narrow settings.
			ConstraintGraph2* pCG=dt.createConstraint(vSegments,CIS_CONSTRAINED_DELAUNAY);
			double minLen(0.1);
			pCG->makeDelaunay(minLen); // Create better shaped triangles
			break;
		}
		case 2: // MODE 2
		{
			// This mode drapes the segments onto the existing surface
			// before insertion. If zTolerance < 0 then the segments
			// are subdivided at all intersections with triangulation
			// edges. Otherwise subdivision takes only place to maintain
			// the defined tolerance i.e., the error between segment and
			// the surface.
			double zTolerance(1.0);
			dt.drape(vSegments,vDrapedSegments,zTolerance);
			dt.createConstraint(vDrapedSegments,CIS_CONSTRAINED_DELAUNAY);
			break;
		}
		case 3: // MODE 3
		{
			// Same as mode2 but with a call to makeDelaunay that
			// subdivides the segments to create better shaped triangles
			double zTolerance(.5);
			dt.drape(vSegments,vDrapedSegments,zTolerance);
			ConstraintGraph2* pCG=dt.createConstraint(vDrapedSegments,CIS_CONSTRAINED_DELAUNAY);
			double minLen(0.1);
			pCG->makeDelaunay(minLen); // Create better shaped triangles
			break;
		}
		default:
		{
			cout<<"mode "<<yourChosenMode<<" does not exist"<<endl;
			exit(1);
		}
	}

	// * 3 *   Visualize - different methods
	dt.writeWebScene("terrain_constraint_webScene"); // For web browsers
	Visualizer3 vis("terrain_constraint.list"); // For the Geomview viewer (Linux)
	dt.showGeomview(&vis);
	vis.writeSegments(vSegments,"0 0 1 0.5",true);
	vis.writeSegments(vDrapedSegments,"1 0 0 0.5",true);
	dt.show("terrain_constraint.ps"); // Postscript for gv, gsview or online ps-viewers


	// *** Test ISO-contour computation ***
	std::cout<<"\n* Testing ISO-contour computation"<<std::endl;

	// * 1 *   Create a global zone
	Zone2* pZone(dt.createZone(NULL,ZL_GLOBAL));
	if(pZone==NULL)
	{
		std::cout<<"No zone, something went wrong."<<std::endl;
		return 1;
	}

	// * 2 *   Get its triangles
	std::vector<Triangle2*> vTriangles;
	pZone->getTriangles(vTriangles);
	std::cout<<"Number of triangles: "<<vTriangles.size()<<std::endl;

	// * 3 *   Compute ISO contours
	isoContours(vTriangles);

	std::cout<<"Normal program end"<<std::endl;
	return 0;
}



// Creates random terrain points, decimates and inserts them
void createTerrain(Fade_2D& dt)
{
	std::vector<Point2> vInputPoints;

	// * 1 *   Create random terrain points
	/*int seed(1); // seed=0 means random, seed>0 is for repeatable randomness
	generateRandomSurfacePoints(
        150, // numPointsX
        100, // numPointsY
        2, // numCenters
        0,0,0,100,100,50, // Bounds xmin,ymin,zmin,xmax,ymax,zmax
        vInputPoints, // Output vector
        seed  // Seed
        );*/

	// * 1 *   Alternative: Read terrain points from an ASCII file
        readXYZ("fade2d.txt",vInputPoints);


	// * 2 *   Optional pruning to represent the model more efficiently
	cout<<"\nOriginal number of points: "<<vInputPoints.size()<<endl;
	EfficientModel em(vInputPoints);
	vInputPoints.clear();
	double maxError(.1);
	em.extract(maxError,vInputPoints);
	cout<<"Efficient number of points: "<<vInputPoints.size()<<endl<<endl;

	// * 3 *   Insert the points
	dt.insert(vInputPoints);

	// * 4 *   Several options to visualize

	// Write a wavefront *.obj file
	dt.writeObj("terrain.obj");
	// Write a visualization for web browsers
	dt.writeWebScene("terrain_webScene");
	// Write a file for the Geomview viewer
	dt.showGeomview("terrain.list");
}


// The isoContours(..) function below computes ISO lines at 3 different heights
void isoContours(std::vector<Triangle2*>& vTriangles)
{
	// Create an instance of the IsoContours class and 3 height values
	IsoContours isoManager(vTriangles);
	double minZ(isoManager.getMinHeight());
	double maxZ(isoManager.getMaxHeight());
	vector<double> vHeights;
	vHeights.push_back(minZ+(maxZ-minZ)*1/4);
	vHeights.push_back(minZ+(maxZ-minZ)*2/4);
	vHeights.push_back(minZ+(maxZ-minZ)*3/4);


	// Open a result file
	std::ofstream f("contours.txt");
	if(!f.is_open())
	{
		std::cout<<"File contours.txt can't be written"<<std::endl;
		return;
	}
	std::cout<<"isoContours(): Writing result file contours.txt"<<std::endl;

	// For all height values
	for(size_t i=0;i<vHeights.size();++i)
	{
		double height(vHeights[i]);

		// The result will be stored in a vector of polygons or polylines
		std::vector<std::vector<Segment2> > vvIsoContours;

		// IsoContours::getContours() intersects the triangulation with a horizontal
		// plane at height z. Certain height values lead to degenerate intersections
		// and IsoContours::getContours() returns false in this case. Then a different
		// height value must be chosen (small random perturbation is sufficient).
		while(!isoManager.getContours(height,vvIsoContours,true))
		{
			double rnd=(1e-4*rand()/(RAND_MAX+1.0));
			std::cout<<"Degenerate intersection at height "<<height<<std::endl;
			height+=rnd;
		}

		f<<"\n\n\n** Number of contours at height "<<height<<": "<<vvIsoContours.size()<<std::endl<<std::endl;

		// Write the result
		for(size_t i=0;i<vvIsoContours.size();++i)
		{
			std::vector<Segment2>& vContour(vvIsoContours[i]);
			f<<"Contour no. "<<i<<" at z="<<height<<" consists of "<<vContour.size()<<" segments"<<std::endl;
			Point2 sourcePoint(vContour[0].getSrc());
			Point2 targetPoint(vContour.back().getTrg());
			if(sourcePoint==targetPoint)
			{
				f<<"the contour is a closed polygon"<<std::endl;
			}
			else
			{
				f<<"the contour is an open polyline"<<std::endl;
			}

			for(std::vector<Segment2>::iterator it(vContour.begin());it!=vContour.end();++it)
			{
				Segment2& seg(*it);
				f<<seg<<std::endl;
			}
			f<<std::endl;
		}
	}
}





