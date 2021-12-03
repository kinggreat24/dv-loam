// Copyright (C) Geom Software e.U, Bernhard Kornberger, Graz/Austria
//
// This file is part of the Fade2D library. The student license is free
// of charge and covers personal non-commercial research. Licensees
// holding a commercial license may use this file in accordance with
// the Commercial License Agreement.
//
// This software is provided AS IS with NO WARRANTY OF ANY KIND,
// INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE.
//
// Please contact the author if any conditions of this licensing are
// not clear to you.
//
// Author: Bernhard Kornberger, bkorn (at) geom.at
// http://www.geom.at

#pragma once


#include "common.h"
#if GEOM_PSEUDO3D==GEOM_TRUE

#include "Point2.h"
#include "Segment2.h"
#include "VertexPair2.h"
#include "Edge2.h"
namespace GEOM_FADE25D
{


/** \brief Visualizer3
*/
class  Visualizer3
{
public:
	static std::string CLIGHTBLUE;
	static std::string CDARKBLUE;
	static std::string CYELLOW;
	static std::string CPINK;
	static std::string CBLACK;
	static std::string CLIGHTBROWN;
	static std::string CDARKBROWN;
	static std::string CORANGE;
	static std::string CPURPLE;
	static std::string CGRAY;
	static std::string CLIGHTGRAY;
	static std::string CRED;
	static std::string CGREEN;
	static std::string CWHITE;
	static std::string CRIMSON;
	static std::string CDARKORANGE;
	static std::string CGOLDENROD;
	static std::string COLIVE;
	static std::string CLAWNGREEN;
	static std::string CGREENYELLOW;
	static std::string CPALEGREEN;
	static std::string CMEDSPRINGGREEN;
	static std::string CLIGHTSEAGREAN;
	static std::string CCYAN;
	static std::string CSTEELBLUE;
	static std::string MIDNIGHTBLUE;
	static std::string CWHEAT;

	static std::string getColor(int i);
	static std::string getNextColor();
	static std::string getNextColorAndName(std::string& name);
	explicit CLASS_DECLSPEC Visualizer3(const std::string& name);
	CLASS_DECLSPEC ~Visualizer3();
	void CLASS_DECLSPEC closeFile();
	void CLASS_DECLSPEC openFile(const std::string& name);
	void CLASS_DECLSPEC writeNormals(const std::vector<Triangle2*>& vT,double scale);
	void CLASS_DECLSPEC writePoints(const std::vector<Point2*>& vPoints,unsigned linewidth,const std::string& color) ;
	void CLASS_DECLSPEC writePoints(const std::vector<Point2>& vPoints,unsigned linewidth,const std::string& color) ;

	void CLASS_DECLSPEC writePoint(const Point2& p,unsigned linewidth,const std::string& color);
	//void CLASS_DECLSPEC writeSegment(const Point2& src,const Point2& trg,double r,double g,double b,double alpha);
	void CLASS_DECLSPEC writeSegment(const Point2& src,const Point2& trg,const std::string& color,bool bWithEndPoints=false);
	void CLASS_DECLSPEC writeSegments(const std::vector<Segment2>& vSegments,const std::string& color,bool bWithEndPoints=false);
	void CLASS_DECLSPEC writeSegments(const std::vector<Edge2>& vSegments,const std::string& color,bool bWithEndPoints=false);
	void CLASS_DECLSPEC writeVertexPairs(const std::vector<VertexPair2>& vVertexPairs,const std::string& color);
	//void CLASS_DECLSPEC writePolygon(const std::vector<Point2>& vPoints,double r,double g,double b,double alpha);
	void CLASS_DECLSPEC writeCubes(const std::vector<Point2>& vPoints,const std::string& color);
	void CLASS_DECLSPEC writeTriangles(const std::vector<Triangle2*>& vT,const std::string& color,bool bWithNormals=false);
	void CLASS_DECLSPEC writeTriangles(const std::vector<Point2>& vTriangleCorners,const std::string& color,bool bWithNNV);
	void CLASS_DECLSPEC writeTriangle(const Triangle2& t,const std::string& color);
	void CLASS_DECLSPEC writeTriangle(const Point2& p0,const Point2& p1,const Point2& p2,const std::string& color);
	void CLASS_DECLSPEC writeBall(Point2& p,double radius);
	void CLASS_DECLSPEC setBackfaces(bool bWithBackfaces_);
private:
	void startList(size_t numPoints,size_t numTriangles,bool bWithEdges);
	void endList();
	std::ofstream outFile;
	static int nextColor;
	bool bWithBackfaces;
};

} // NAMESPACE FADE25D

#elif GEOM_PSEUDO3D==GEOM_FALSE
#else
#error GEOM_PSEUDO3D is not defined
#endif

