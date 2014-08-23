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
 * @file AdaboostClassifierNodeParams.h
 *    header File for the AdaboostClassifierNodeParams
 *
 * @author Tim Wengefeld,Christoph Weinrich
 * @date   2014/08/22
 */

#ifndef ADABOOSTCLASSIFERTREENODEPARAMS_H_
#define ADABOOSTCLASSIFERTREENODEPARAMS_H_

enum StageLabel{
	NO_PERSON=-1, STANDING_PEOPLE=0, WALKER=1 , WHEELCHAIR=2, TOUGH_PERSON=3 , PEOPLEANDWALKER=5, CRUTCH=6,ELSE=7
};

struct AdaboostClassifierNodeParams : public AdaboostClassifierParams{
	AdaboostClassifierNodeParams(){}
	AdaboostClassifierNodeParams(
	                         	StageLabel posLabel,
	                         	StageLabel negLabel,
	                        	string description,
	                        	string openCvPath,
	                        	float threshold,
	                        	uint featureVectorSize
	                         	) : AdaboostClassifierParams(openCvPath, threshold, featureVectorSize){
     	mPosLabel = posLabel;
     	mNegLabel = negLabel;
    	mClassifierDescription = description;
	}

    template<typename Reflector>
    void reflect(Reflector& r) {
    	AdaboostClassifierParams::reflect(r);
    	r.member("PosLabel", mPosLabel, "");
    	r.member("NegLabel",mNegLabel,"");
    	r.member("ClassifierDescription",mClassifierDescription,"");
    	r.member("PosChild",mPosChild,"");
    	r.member("NegChild",mNegChild,"");
    }

	StageLabel mPosLabel;
	StageLabel mNegLabel;
	string mClassifierDescription;
	boost::shared_ptr<AdaboostClassifierNodeParams> mPosChild;
	boost::shared_ptr<AdaboostClassifierNodeParams> mNegChild;
};

#endif /* ADABOOSTCLASSIFERTREENODEPARAMS_H_ */
