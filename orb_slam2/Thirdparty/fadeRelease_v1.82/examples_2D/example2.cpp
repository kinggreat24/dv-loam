#include <Fade_2D.h>
#include <stdio.h>
using namespace GEOM_FADE2D;
using namespace std;

std::string toString(int i)
{
	std::ostringstream oss;
	oss << i;
	return oss.str();
}

// Goal: Lern to use the visualizer class
void how_to_draw()
{
	// *** COLORS ***
	// Define color red by r,g,b values, linewidth and bFill
	Color cRed(1,0,0,0.001,false);
	// Or simply use a color name and the defaults linewidth=0.001 and bFill=false
	Color cBlue(CBLUE);
	Color cGreen(CGREEN);
	Color cPurple(CPURPLE);
	// Specify a bolt green color (linewidth=1.0) and another one with
	// bFill=true to fill the area of an object which is drawn using
	// that color. In case that a segment is drawn with bFill=true,
	// marks for its endpoints are added.
	Color cGreenBolt(CGREEN,1.0,false);
	Color cGreenFill(CGREEN,.001f,true);

	// Postscript writer
	Visualizer2 vis("how_to_draw.ps");

	// Create a label and add it to the Visualizer
	Label headLabel(Point2(10,80),	// Position
					"This is the Fade2D\nPostscript Visualizer",
					false,			// Don't write an x-mark
					40);			// Font size
	vis.addObject(headLabel,cRed); 	// Add the label

	// Add a row of points
	for(int x=0;x<100;x+=5)
	{
		Point2 p(x,65);
		vis.addObject(p,cPurple); // Add the point
	}

	// Draw a segment with cGreen
	Point2 p0(0,0);
	Point2 p1(100,0);
	Segment2 seg0(p0,p1);
	vis.addObject(seg0,cGreen); // Add the segment


	// Draw a segment with cGreenFill
	Point2 p2(0,10);
	Point2 p3(100,10);
	Segment2 seg1(p2,p3);
	vis.addObject(seg1,cGreenFill); // Add the segment

	// Draw a segment with cGreenBolt
	Point2 p4(0,20);
	Point2 p5(100,20);
	Segment2 seg2(p4,p5);
	vis.addObject(seg2,cGreenBolt); // Add the segment

	// Draw labels
	Label lab0(Point2(20,0),"Segment in cGreen",false,11);
	vis.addObject(lab0,cBlue); // Add the label

	Label lab1(Point2(20,10),"Segment in cGreenFill",false,11);
	vis.addObject(lab1,cBlue); // Add the label

	Label lab2(Point2(20,20),"Segment in cGreenBolt",false,11);
	vis.addObject(lab2,cBlue); // Add the label


	// Add three circles with radius=4 (squared radius 16)
	double sqRadius(4.0*4.0);
	Circle2 circ0(50,40,sqRadius);
	Circle2 circ1(60,40,sqRadius);
	Circle2 circ2(70,40,sqRadius);
	vis.addObject(circ0,cGreen);	// Add the circle
	vis.addObject(circ1,cGreenBolt);// Add the circle
	vis.addObject(circ2,cGreenFill);// Add the circle

	// The postscript file is only written when writeFile() is called.
	vis.writeFile();
}



// Goal: Access elements of a triangulation
void accessAndDraw(Fade_2D& dt)
{
	Visualizer2 vis("accessAndDraw.ps");

	// Some colors.
	Color cBlack(CBLACK);
	Color cBlue(CBLUE);
	Color cGreen(CGREEN);
	Color cRed(CRED);

	// 1) Get the points of the triangulation
	std::vector<Point2*> vAllPoints;
	dt.getVertexPointers(vAllPoints);
	std::cout<<"vAllPoints.size()="<<vAllPoints.size()<<std::endl;

	// 2) Draw the points together with their (optional) custom index
	for(std::vector<Point2*>::iterator it(vAllPoints.begin());it!=vAllPoints.end();++it)
	{
		Point2* currentPoint(*it);
		int customIndex(currentPoint->getCustomIndex());
		std::string customIndexString(toString(customIndex));
		vis.addObject(Label(*currentPoint,customIndexString),cBlack);
	}

	// 3) Get and draw the triangles
	std::vector<Triangle2*> vAllDelaunayTriangles;
	dt.getTrianglePointers(vAllDelaunayTriangles);
	for(std::vector<Triangle2*>::iterator it=vAllDelaunayTriangles.begin();it!=vAllDelaunayTriangles.end();++it)
	{
		Triangle2* pT(*it);
		vis.addObject(*pT,cBlack);

		// An alternative method (just to show how to access the vertices) would be:
		//Point2* p0=pT->getCorner(0);
		//Point2* p1=pT->getCorner(1);
		//Point2* p2=pT->getCorner(2);
		//vis.addObject(Segment2(*p0,*p1),cBlack);
		//vis.addObject(Segment2(*p1,*p2),cBlack);
		//vis.addObject(Segment2(*p2,*p0),cBlack);
	}

	// 4) Choose some triangle and fill it green
	Triangle2* pT(vAllDelaunayTriangles[0]);
	Color cGreenFill(CPALEGREEN,0.001f,true);
	vis.addObject(*pT,cGreenFill);

	// 5) The corners of pT can be accessed through the so called intra-
	// triangle-indices 0,1,2. They are counterclockwise oriented (CCW).
	// Let's write intra-triangle-index labels beside the corners.
	for(int intraTriangleIndex=0;intraTriangleIndex<3;++intraTriangleIndex)
	{
		Point2* pCorner(pT->getCorner(intraTriangleIndex));
		std::string text("\nidx="+toString(intraTriangleIndex));
		vis.addObject(Label(*pCorner,text,true,20),cBlue);
	}

	// Each triangle has three neighbor triangles (or NULL pointers at
	// the the outer border). They are accessed through the described
	// intra-triangle-indices. The i'th opposite triangle of pT is the
	// one that is opposite to the i'th vertex. Let's draw that:
	Label label_pT(pT->getBarycenter(),"This is pT",true,15);
	vis.addObject(label_pT,cBlue);
	for(int intraTriangleIndex=0;intraTriangleIndex<3;++intraTriangleIndex)
	{
		Triangle2* pNeigT(pT->getOppositeTriangle(intraTriangleIndex));
		if(pNeigT==NULL) continue; // No adjacent triangle at this edge

		// Compute the barycenter and write a label there
		Point2 barycenter(pNeigT->getBarycenter());
		std::string text("\n\nThis is\npT->getOppositeTriangle("+toString(intraTriangleIndex)+")");
		Label neigLabel(barycenter,text,true,15);
		vis.addObject(neigLabel,cRed);
	}

	// Write the postscript file to disk
	vis.writeFile();
}

bool getVoronoiCell(Point2* pVtx,vector<Point2>& vVoronoiVerticesOut)
{
	TriangleAroundVertexIterator start_it(pVtx);
	TriangleAroundVertexIterator end_it=start_it;
	do
	{
		if(*start_it==NULL) // Infinite cell
		{
			vVoronoiVerticesOut.clear();
			return false;
		}
		vVoronoiVerticesOut.push_back((*start_it)->getDual().first);
	} while(++start_it!=end_it);
	return true;
}

// Goal: Draw a Voronoi diagram
void voronoiDraw(Fade_2D& dt)
{
	// Drawing a triangulation is a frequent task. Therefore a quick
	// method to draw the triangles exists:
	Visualizer2 vis("voronoi.ps");
	dt.show(&vis);

	// For all vertices of the triangulation: Draw the finite Voronoi
	// cells. In the present example only one cell is finite.
	Color cRed(CRED,1,true);
	std::vector<Point2*> vAllPoints;
	dt.getVertexPointers(vAllPoints);
	for(std::vector<Point2*>::iterator it( vAllPoints.begin());it!=vAllPoints.end();++it)
	{
		Point2* pVtx(*it);
		vector<Point2> vVoronoiVertices;
		bool bFiniteCell(getVoronoiCell(pVtx,vVoronoiVertices));
		if(bFiniteCell)
		{
			for(size_t i=0;i<vVoronoiVertices.size();++i)
			{
				Point2& p0(vVoronoiVertices[i]);
				Point2& p1(vVoronoiVertices[(i+1)%vVoronoiVertices.size()]);
				Segment2 seg(p0,p1);
				vis.addObject(seg,cRed);
			}
		}
	}

	// Draw the circumcircles of the triangles
	Color cGreen(CGREEN);
	vector<Triangle2*> vAllT;
	dt.getTrianglePointers(vAllT);
	for(vector<Triangle2*>::iterator it(vAllT.begin());it!=vAllT.end();++it)
	{
		Triangle2* pT(*it);
		Point2 circumCenter(pT->getDual().first);
		Point2* pCorner0(pT->getCorner(0));
		double sqRadius=sqDistance2D(circumCenter,*pCorner0);
		Circle2 circ(circumCenter,sqRadius);
		vis.addObject(circ,cGreen);
	}

	// Finally, write the postscript file to disk
	vis.writeFile();

}


int example2_main()
{
	std::cout<<"\n";
	std::cout<<"example2: Access and draw elements of a triangulation\n";

	// *** Goal: Lern to draw
	how_to_draw();


	// *** Goal: Create points, use the optional setCustomIndex
	// method to attach indices. Triangulate the points using one
	// of the two basic insert methods.

	// Some input points
	std::vector<Point2> vInputPoints;
	vInputPoints.push_back(Point2(0,0));
	vInputPoints.push_back(Point2(80,0));
	vInputPoints.push_back(Point2(49.8,62.5));
	vInputPoints.push_back(Point2(-17.8,78));
	vInputPoints.push_back(Point2(-72,34.7));
	vInputPoints.push_back(Point2(-72,-34.7));
	vInputPoints.push_back(Point2(-17.8,-78));
	vInputPoints.push_back(Point2(49.8,-62.5));

	// Optional step: Store an arbitrary index in the points. Such
	// an index is for example useful to associate your program's data
	// structures with the Fade points.
	int someCustomIndex(77); // An arbitrary value
	for(size_t i=0;i<vInputPoints.size();++i)
	{
		vInputPoints[i].setCustomIndex(someCustomIndex);
		++someCustomIndex;
	}

	// There are two basic methods to insert points, the all-at-once and
	// the one-by-one method. The all-at-once method is faster, so use
	// it if you can. Both methods return pointers to the created
	// vertices. In case of a (duplicate) vertex the pointer of the
	// first inserted vertex is returned.
	Fade_2D dt; // The Delaunay triangulation
	std::vector<Point2*> vVertexPointersBack; // Used to store the pointers

	// All-at-once method (returned pointers have the same order as in vInputPoints)
	dt.insert(vInputPoints,vVertexPointersBack);

	// One-by-one method
	//for(std::vector<Point2>::const_iterator p_it=vInputPoints.begin();
		//p_it!=vInputPoints.end();++p_it)
	//{
		//Point2* pDelaunayVertex=dt.insert(*p_it); // A single vertex is inserted and a pointer is returned
		//vVertexPointersBack.push_back(pDelaunayVertex); // ...and stored into vDelaunayVertexPointers
	//}

	// *** Goal: Access and draw elements of the triangulation
	accessAndDraw(dt);

	// *** Goal: Compute and draw the dual Voronoi diagram
	voronoiDraw(dt);



	return 0;
}


