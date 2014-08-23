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

/*
 * When using this software for your own research, please acknowledge the effort
 * that went into its construction by citing the corresponding paper:
 *
 *   C. Weinrich, T. Wengefeld, C. Schröter and H.-M. Gross
 *   People Detection and Distinction of their Walking Aids in 2D Laser Range
 *   Data based on Generic Distance-Invariant Features.
 *   In Proceedings of the IEEE International Symposium on Robot and Human
 *   Interactive Communication (RO-MAN), 2014, Edinburgh (UK)
 */

/**
 * @file AdaboostClassifierNode.h
 *    header File for the AdaboostClassifierNode
 *
 * @author Tim Wengefeld,Christoph Weinrich
 * @date   2014/08/22
 */

#ifndef ADABOOSTCLASSIFIERTREENODE_H
#define ADABOOSTCLASSIFIERTREENODE_H

#include <AdaboostClassifierNode.h>

namespace mira {
namespace adaboosttreeclassifier {

void AdaboostClassifierNode::initialize(boost::shared_ptr<AdaboostClassifierNodeParams> adaboostClassifierParams){
	mParams=adaboostClassifierParams;
	mNodeParams=adaboostClassifierParams;
	this->loadOpenCv();
	if(mNodeParams->mPosChild!=NULL){
		this->mPosChild.reset(new AdaboostClassifierNode());
		this->mPosChild->initialize(mNodeParams->mPosChild);
	}
	if(mNodeParams->mNegChild!=NULL){
		this->mNegChild.reset(new AdaboostClassifierNode());
		this->mNegChild->initialize(mNodeParams->mNegChild);
	}
}

std::pair<float,StageLabel> AdaboostClassifierNode::apply(std::vector<float> const &sample){
	cv::Mat cvtfeatures;

	cvtfeatures = cv::Mat(1, sample.size(), CV_32F);
	for (uint f = 0; f < sample.size(); ++f) {
		cvtfeatures.at<float>(0, f) = sample[f];
	}

	float result = this->predict(cvtfeatures,cv::Mat(),cv::Range::all(),false,true);

	if(result+mParams->mThreshold>0){
		if(this->mPosChild==NULL){
			std::pair<float,StageLabel> retValue(result+mParams->mThreshold,mNodeParams->mPosLabel);
			return retValue;
		}
		else{
			return this->mPosChild->apply(sample);
		}
	}
	else{
		if(this->mNegChild==NULL){
			std::pair<float,StageLabel> retValue(result+mParams->mThreshold,mNodeParams->mNegLabel);
			return retValue;
		}
		else{
			return this->mNegChild->apply(sample);
		}
	}
}

}
}

#endif
