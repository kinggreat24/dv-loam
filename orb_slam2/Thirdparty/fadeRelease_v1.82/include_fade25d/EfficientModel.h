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

/** \file EfficientModel.h
 */





namespace GEOM_FADE25D {

struct EMData; // FWD
struct Candidate;// FWD

/// SmoothingStrategy for EfficientModel::zSmoothing()
enum SmoothingStrategy
{
	SMS_MINIMUM, ///< Assign the minimum height
	SMS_MAXIMUM, ///< Assign the maximum height
	SMS_MEDIAN, ///< Assign the median height
	SMS_AVERAGE ///< Assign the average height
};

/** \brief EfficientModel
*
* Point clouds from terrains are often oversampled. The present class
* aims to smooth and to reduce these point clouds in a controlled way
* such that the resulting surface keeps a certain user specified
* quality.\n
*
* \image html pointCloudReduction.png "Point Cloud Reduction: Left original, right reduced" width=95%
* \image latex pointCloudReduction.eps "Point Cloud Reduction: Left original, right reduced" width=15cm

*/
class CLASS_DECLSPEC EfficientModel
{
public:
	explicit EfficientModel(const std::vector<Point2>& vPoints);
	~EfficientModel();

/** \brief Smoothing
 *
 * This method should be used before extract(). It adapts the z-values
 * according to the chosen SmoothingStrategy @p sms.
 *
 * @param numIterations Number of iterations
 * @param maxDifferencePerIteration is the maximum change of any z-value
 * @param sms is one of SMS_MINIMUM, SMS_MAXIMUM, SMS_MEDIAN, SMS_AVERAGE
 */
	void zSmoothing(int numIterations,double maxDifferencePerIteration,SmoothingStrategy sms);


/** \brief Extract a subset of points
*
* This method extracts a subset of the original point cloud that
* represents the model more efficiently. Thereby the original and
* the simplified model cover the same area.
*
* @param maxError is the maximum height difference between the original
* points and the simplified model.
* @param [out] vEfficientPointsOut is used to return a subset of the
* original points that represents the model more efficiently.
*
* @note When @p maxError is tiny i.e., below the noise level of the
* point cloud, then processing can take quite some time. Consider
* using the zSmoothing() method before.
*
*/
	void extract(double maxError,std::vector<Point2>& vEfficientPointsOut);


	//void extract2(double maxError,std::vector<Point2>& vEfficientPointsOut);
protected:
	void solveCand(Candidate* pCand,double maxErr);
	//void assignT();
	void go();
	void part1_extractFC();
	void part2_setWeights();

	void sortVtx(std::vector<Point2*>& vVtx);
	int insertKeepError(
						double factor,
						double err,
						std::vector<Point2*>& vIn,
						std::vector<Point2*>& vNeedlessBigError,
						std::vector<Point2*>& vNeedlessSmallError
						);


	void insertMinHull();
	void show(const std::string& name);

	/// @private
	EMData* pEMData;
private:
	EfficientModel(const EfficientModel&);

};

} // NAMESPACE

// up to here: if GEOM_PSEUDO3D==GEOM_TRUE
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif
