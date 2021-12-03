#include <Fade_2D.h>
#include <stdio.h>
#include <map>
#include <string>
#include <sstream>
#include <iomanip>
using namespace GEOM_FADE2D;
using namespace std;

// Good method
void customIndexMethod(vector<Point2>& vPoints)
{
	Fade_2D dt;
	for(size_t i=0;i<vPoints.size();++i)
	{
		vPoints[i].setCustomIndex(int(i));
	}
	dt.insert(vPoints);

	vector<Triangle2*> vAllTriangles;
	dt.getTrianglePointers(vAllTriangles);

	timer("customIndexMethod");
	std::ofstream f("out0.txt");
	for(vector<Triangle2*>::iterator it(vAllTriangles.begin());
		it!=vAllTriangles.end();++it)
	{
		Triangle2* pT(*it);
		for(int i=0;i<3;++i)
		{
			Point2* pCorner(pT->getCorner(i));
			f<<pCorner->getCustomIndex()<<" ";
		}
		f<<"\n";
	}
	timer("customIndexMethod");
	f.close();
}

// Good method
void mapMethod(vector<Point2>& vPoints)
{
	Fade_2D dt;
	vector<Point2*> vHandles;
	dt.insert(vPoints,vHandles);
	
	vector<Triangle2*> vAllTriangles;
	dt.getTrianglePointers(vAllTriangles);


	map<Point2*,int> mVertexIndex;
	for(size_t i=0;i<vHandles.size();++i)
	{
		Point2* pVtx(vHandles[i]);
		mVertexIndex[pVtx]=int(i);
	}

	std::ofstream f("out1.txt");
	timer("mapMethod");
	for(vector<Triangle2*>::iterator it(vAllTriangles.begin());
		it!=vAllTriangles.end();++it)
	{
		Triangle2* pT(*it);
		for(int i=0;i<3;++i)
		{
			Point2* pCorner(pT->getCorner(i));
			f<<mVertexIndex[pCorner]<<" ";
		}
		f<<"\n";
	}
	timer("mapMethod");
	f.close();
}

// Bad method
void badMethod(vector<Point2>& vPoints)
{
	std::cout<<"\n *** Note: Remove the <return;> statement to run the bad demo also. But it won't finish in reasonable time *** \n\n";
	return;

	Fade_2D dt;
	dt.insert(vPoints);

	// BAD CODE FOR DEMONSTRATION - don't do it like that!
	vector<Triangle2*> vAllTriangles;
	dt.getTrianglePointers(vAllTriangles);

	std::ofstream f("out2.txt");
	timer("badMethod");
	for(vector<Triangle2*>::iterator it(vAllTriangles.begin());
		it!=vAllTriangles.end();++it)
	{
		Triangle2* pT(*it);
		for(int i=0;i<3;++i)
		{
			Point2* pCorner(pT->getCorner(i));
			for(size_t i=0;i<vPoints.size();++i)
			{
				if(vPoints[i]==*pCorner)
				{
					f<<i<<" ";
					break;
				}
			}
		}
		f<<"\n";
	}
	timer("badMethod");
	f.close();
}


int facelist_main()
{
	std::cout<<"\n";
	std::cout<<"best practices: Face list\n";

	// Create random points
	vector<Point2> vPoints;
	generateRandomPoints(500000,0,100,vPoints,0);


	// Start the three methods	
	customIndexMethod(vPoints);
	mapMethod(vPoints);
	badMethod(vPoints);



	return 0;
}


