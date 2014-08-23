/*
 * Copyright (C) 2014 by
 *   Neuroinformatics and Cognitive Robotics Labs (NICR) at TU Ilmenau, GERMANY
 * All rights reserved.
 *
 * Contact: christoph.weinrich@tu-ilmenau.de,
 *          tim.wengefeld@tu-ilmenau.de
 *
 * GNU General Public License Usage:
 *   This file may be used under the terms of the GNU General Public License
 *   version 3.0 as published by the Free Software Foundation and appearing in
 *   the file LICENSE.GPL3 included in the packaging of this file. Please review
 *   the following information to ensure the GNU General Public License
 *   version 3.0 requirements will be met: http://www.gnu.org/copyleft/gpl.html.
 *   Alternatively you may (at your option) use any later version of the GNU
 *   General Public License if such license has been publicly approved by NICR.
 *
 * IN NO EVENT SHALL "NICR" BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
 * SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS
 * SOFTWARE AND ITS DOCUMENTATION, EVEN IF "NICR" HAS BEEN ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * "NICR" SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND "NICR"
 * HAVE NO OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS OR
 * MODIFICATIONS.
 */

/**
 * @file GDIFDetector.C
 *    source File for the GDIF Detector
 *
 * @author Tim Wengefeld,Christoph Weinrich
 * @date   2014/08/22
 */

#include <GDIFDetectorTree.h>

using namespace mira;
using namespace mira::robot;
using namespace mira::adaboosttreeclassifier;

namespace mira { namespace laserbasedobjectdetection {

void GDIFDetectorTree::inititalize(boost::shared_ptr<AdaboostClassifierNodeParams> adaboostParams,
		 	 	 SegmentationParams segmentationParams,
		 	 	 BoundingBoxParams boundingBoxParams)
{
	mAdaboostParams=adaboostParams;
	mClassifier.initialize(mAdaboostParams);
	mSegmentationParams = segmentationParams;
	mBoundingBoxParams = boundingBoxParams;
	firstScan=true;
}

std::vector<StageLabel> GDIFDetectorTree::classifyScan(RangeScan const& iRangeScan,std::vector<Point2f> & oPositions){
	vector<Point2f> center = getRangeSegmentsCenter(iRangeScan,mSegmentationParams.mJumpDistance,mSegmentationParams.mMinSegmentSize);
	std::vector<StageLabel> labels;
	if(firstScan){
		mAngles.reserve(iRangeScan.range.size());
		for(uint j=0;j<iRangeScan.range.size();j++){
			mAngles.push_back(iRangeScan.startAngle+iRangeScan.deltaAngle*(float)j);
		}
		firstScan=false;
	}

	for(int i=center.size()-1;i>=0;i--){
		if(std::sqrt(center[i].x()*center[i].x()+center[i].y()*center[i].y())>mSegmentationParams.mMaxRange)continue;
		GDIFeatures sample;
		if(mBoundingBoxParams.mBoxMode==BoxMode::LEFT){
			sample.buildBoxFromLeft(iRangeScan,center[i],mBoundingBoxParams);
		}
		else if(mBoundingBoxParams.mBoxMode==BoxMode::CENTER){
			sample.buildBoxFromCenter(iRangeScan,center[i],mBoundingBoxParams);
		}

		if(sample.isValid()){
			std::pair<float,StageLabel> predict;
			sample.calcRadialFeatures(iRangeScan.range,mAngles);
			predict = mClassifier.apply(sample.getRadialFeatures());
			if(predict.second!=NO_PERSON){
				oPositions.push_back(center[i]);
				labels.push_back(predict.second);
			}
		}
	}
	return labels;
}

}
}
