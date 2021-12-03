#include <Fade_2D.h>
using namespace GEOM_FADE25D;
using namespace std;


// This file contains two examples:
// *) segmentCheckerSimple_main() is short, it uses random segments
// *) segmentCheckerTerrain_main() uses ISO lines from a terrain


void segmentCheckerSimple_main()
{
	// * 1 *   Create random segments
	vector<Segment2> vRandomSegments;
	generateRandomSegments(100,-100,+100,30,vRandomSegments,1);
	vector<Segment2*> vRandomSegmentPtrs;
	for(size_t i=0;i<vRandomSegments.size();++i) vRandomSegmentPtrs.push_back(&vRandomSegments[i]);

	// * 2 *   Determine all intersecting segments (ignore endpoint intersections)
timer("getIllegalSegments");
	SegmentChecker segChecker(vRandomSegmentPtrs);
	std::vector<Segment2*> vIllegalSegments;
	segChecker.getIllegalSegments(false,vIllegalSegments);
timer("getIllegalSegments"); // Takes 0.09 seconds for 105739 segments of which 37 do intersect

	// * 3 *   Analyze the intersecting segments
	cout<<"Number of illegal segments: "<<vIllegalSegments.size()<<endl;
	for(size_t i=0;i<vIllegalSegments.size();++i)
	{
		Segment2* pSeg(vIllegalSegments[i]);
		cout<<"\nAnalyzing segment no. "<<segChecker.getIndex(pSeg)<<": "<<endl;

		// Get the intersectors of pSeg
		std::vector<std::pair< Segment2*,SegmentIntersectionType> > vIntersectors;
		segChecker.getIntersectors(pSeg,false,vIntersectors);

		// Iterate over the intersectors of pSeg:
		for(size_t j=0;j<vIntersectors.size();++j)
		{
			// The intersector and the intersection type
			Segment2* pOtherSeg(vIntersectors[j].first);
			SegmentIntersectionType sit(vIntersectors[j].second);
			cout<<"  Conflicting segment no. "<<segChecker.getIndex(pOtherSeg)<<"\t type="<<segChecker.getIntersectionTypeString(sit)<<endl;

			// Depending on the segment intersection type (sit):
			switch(sit)
			{
				case SIT_ENDPOINT:
				case SIT_POINT:
				{
					// Two segments can intersect at two different z values, thus two intersection points
					Point2 isp0,isp1;
					segChecker.getIntersectionPoint(sit,*pSeg,*pOtherSeg,isp0,isp1);
					cout<<"    intersection point on segment "<<segChecker.getIndex(pSeg)<<": "<<isp0<<endl;
					cout<<"    intersection point on segment "<<segChecker.getIndex(pOtherSeg)<<": "<<isp1<<endl;

					break;
				}
				case SIT_SEGMENT:
				{
					// Same for a collinear intersection, there may be two segments at different heights
					Segment2 iss0,iss1;
					segChecker.getIntersectionSegment(*pSeg,*pOtherSeg,iss0,iss1);
					cout<<"    intersection segment on segment "<<segChecker.getIndex(pSeg)<<": "<<iss0<<endl;
					cout<<"    intersection segment on segment "<<segChecker.getIndex(pOtherSeg)<<": "<<iss1<<endl;
					break;
				}
				case SIT_NONE: // Never reached
				{
					cout<<"    no intersection, impossible case"<<endl;
					break;
				}
				default: // Never reached
				{
					cout<<"    uninitialized, impossible case"<<endl;
				}
			}
		}
	}

	// * 4 *   Visualize the segments and their intersections
	segChecker.showIllegalSegments(false,"simpleSegments_out.ps");

}


// A terminal progress bar
class MyProgressBar:public GEOM_FADE25D::MsgBase
{
public:
        // Fade calls the update method with d={0.0,...,1.0}
        void update(MsgType ,const std::string& s,double d)
        {
                cout<<s<<" [";
                for(size_t i=0;i<10;++i)
                {
                        if(i/10.0<d) cout<<"=";
                                else cout<<" ";
                }
                cout<<"] "<<d*100.0<<" %    \r"<<flush;
                if(d==1.0) cout<<endl<<endl;
                //printf("Progress %s: %.2f %%\n",s.c_str(),100.0*d);
        }
};


// The isoContours(..) function computes ISO lines at different heights
void isoContours(	std::vector<Triangle2*>& vTriangles,std::vector<Segment2>& vISOSegmentsOut)
{
	// Create an instance of the IsoContours class
	IsoContours isoManager(vTriangles);

	// Determine a few heights where we compute ISO contours
	vector<double> vLevels;
	double minZ(isoManager.getMinHeight());
	double maxZ(isoManager.getMaxHeight());
	const int numLevels(40);
	double zRange(maxZ-minZ);
	double zStep(zRange/(numLevels+1));
	for(int i=1;i<=numLevels;++i) vLevels.push_back(minZ+i*zStep);

	vLevels.push_back(minZ+(maxZ-minZ)*3/4);

	// For all height values
	for(size_t i=0;i<vLevels.size();++i)
	{
		double height(vLevels[i]);

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

		// Write the result
		for(size_t i=0;i<vvIsoContours.size();++i)
		{
			std::vector<Segment2>& vContour(vvIsoContours[i]);
			copy(vContour.begin(),vContour.end(),back_inserter(vISOSegmentsOut));
		}
	}
}






void segmentCheckerTerrain_main()
{
	// * 1 *   Read terrain points from an ASCII file (X Y Z)
	vector<Point2> vTerrainPoints;
	if(!readXYZ("../examples_25D/gaeta_small.xyz",vTerrainPoints)) return;

	// * 2 *   Triangulate the points
	Fade_2D dt;
	dt.insert(vTerrainPoints);
	// dt.writeWebScene("gaeta_webScene"); // Visualization for your browser

	// * 3 *   Compute ISO contours (these do clearly not intersect each other)
	std::vector<Triangle2*> vTriangles;
	dt.getTrianglePointers(vTriangles);
	std::vector<Segment2> vISO;
	isoContours(vTriangles,vISO);
	cout<<"numTriangles: "<<vTriangles.size()<<", num ISO segments: "<<vISO.size()<<endl;

	// * 4 *   Create one additional (fake) Segment which intersects other segments
	Segment2 fakeSegment(	Point2(2391500,4569000, 4711),Point2(2396000,4569000, 4711));
	vISO.push_back(fakeSegment);

	// * 5 *   Determine intersecting segments
	std::vector<Segment2*> vISOPtr; // SegmentChecker needs pointers to segments
	for(size_t i=0;i<vISO.size();++i) vISOPtr.push_back(&vISO[i]);
	MyProgressBar progressBar;

	timer("getIllegalSegments");
		SegmentChecker segChecker(vISOPtr);
		segChecker.subscribe(MSG_PROGRESS,&progressBar);

		std::vector<Segment2*> vIllegalSegments;
		segChecker.getIllegalSegments(false,vIllegalSegments);
	timer("getIllegalSegments"); // Takes 0.09 seconds for 105739 segments of which 37 do intersect

	// * 6 *   Visualization
	segChecker.showIllegalSegments(false,"illegalSegments.ps");

	// * 7 *   Analyze the illegal segments
	cout<<"Number of illegal segments: "<<vIllegalSegments.size()<<endl;
	for(size_t i=0;i<vIllegalSegments.size();++i)
	{
		Segment2* pSeg(vIllegalSegments[i]);
		cout<<"\nAnalyzing segment no. "<<segChecker.getIndex(pSeg)<<": "<<endl;

		// Get the intersectors of pSeg
		std::vector<std::pair< Segment2*,SegmentIntersectionType> > vIntersectors;
		segChecker.getIntersectors(pSeg,false,vIntersectors);

		// Iterate over the intersectors of pSeg:
		for(size_t j=0;j<vIntersectors.size();++j)
		{
			// The intersector and the intersection type
			Segment2* pOtherSeg(vIntersectors[j].first);
			SegmentIntersectionType sit(vIntersectors[j].second);
			cout<<"  Conflicting segment no. "<<segChecker.getIndex(pOtherSeg)<<"\t type="<<segChecker.getIntersectionTypeString(sit)<<endl;

			// Depending on the segment intersection type (sit):
			switch(sit)
			{
				case SIT_ENDPOINT:
				case SIT_POINT:
				{
					// Two segments can intersect at two different z values, thus two intersection points
					Point2 isp0,isp1;
					segChecker.getIntersectionPoint(sit,*pSeg,*pOtherSeg,isp0,isp1);
					cout<<"    intersection point on segment "<<segChecker.getIndex(pSeg)<<": "<<isp0<<endl;
					cout<<"    intersection point on segment "<<segChecker.getIndex(pOtherSeg)<<": "<<isp1<<endl;

					break;
				}
				case SIT_SEGMENT:
				{
					// Same for a collinear intersection, there may be two segments at different heights
					Segment2 iss0,iss1;
					segChecker.getIntersectionSegment(*pSeg,*pOtherSeg,iss0,iss1);
					cout<<"    intersection segment on segment "<<segChecker.getIndex(pSeg)<<": "<<iss0<<endl;
					cout<<"    intersection segment on segment "<<segChecker.getIndex(pOtherSeg)<<": "<<iss1<<endl;
					break;
				}
				case SIT_NONE: // Never reached
				{
					cout<<"    no intersection, impossible case"<<endl;
					break;
				}
				default: // Never reached
				{
					cout<<"    uninitialized, impossible case"<<endl;
				}
			}

			//Draw it if you want
			//Visualizer2 vis("intersection.ps");
			//vis.addObject(*pSeg,Color(CGREEN));
			//vis.addObject(pSeg->getSrc(),Color(CGREEN));
			//vis.addObject(pSeg->getTrg(),Color(CGREEN));
			//vis.addObject(*pOtherSeg,Color(CRED));
			//vis.addObject(pOtherSeg->getSrc(),Color(CRED));
			//vis.addObject(pOtherSeg->getTrg(),Color(CRED));
			//cout<<*pSeg<<endl;
			//cout<<*pOtherSeg<<endl;
			//vis.writeFile();
		}
	}
}
