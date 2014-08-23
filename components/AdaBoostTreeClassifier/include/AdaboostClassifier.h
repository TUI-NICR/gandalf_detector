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
 * @file AdaboostClassifier.h
 *    header File for the Adaboost Classifier
 *
 * @author Tim Wengefeld,Christoph Weinrich
 * @date   2014/08/22
 */

#ifndef ADABOOSTCLASSIFIER_H
#define ADABOOSTCLASSIFIER_H

#include <opencv/cv.h>
#include <opencv/ml.h>
#include <AdaboostClassifierParams.h>
#include <boost/shared_ptr.hpp>

namespace mira {
namespace adaboosttreeclassifier {

using namespace std;

enum SampleLabel {
    NEG = -1, POS = 1
};

class AdaboostClassifier : public cv::Boost{
public :
	AdaboostClassifier(){}

    /**
     * @brief apply the cascade if a sample reaches the last stage the result will be returned, else -FLT_MAX
     * @param sample - the sample to be classified
     * @return the result or -FLT_MAX
     */
    float apply(std::vector<float> const &sample) const;

    /**
     * @brief evalCascade
     * @param tPosSamples - positive samples
     * @param tNegSamples - negative samples
     * @param TP - true positives
     * @param FN - false negatives
     * @param TN - true negatives
     * @param FP - false positives
     */
    void testClassifier(std::vector<std::vector<float> > const& tPosSamples,
              	  	    std::vector<std::vector<float> > const& tNegSamples,
              	  	    long &TP,
              	  	    long &FN,
              	  	    long &TN,
              	  	    long &FP);

    void generateLearningCurve(int steps);

    /**
     * @brief setTrainData
     * @param posSamples - feature vector of the positive training samples
     * @param negSamples - feature vector of the negative training samples
     */
    void setTrainData(std::vector<std::vector<float> > posSamples,std::vector<std::vector<float> > negSamples);

    /**
     * @brief saves the trained opencv adaboost classifier
     * @param opencvPath the path where the serialized classifier is saved
     */
    void saveOpenCv(string opencvPath){
    	///mParams->mOpenCvPath=opencvPath;
    	//this->save(mParams->mOpenCvPath.c_str());
    	this->save(opencvPath.c_str());
    }
    /**
     * @brief loads the opencv adaboost classifier
     * @param opencvPath the path to the serialized opencv adaboost classifier
     */
    void inline loadOpenCv(){
    	this->load(mParams->mOpenCvPath.c_str());
    }

    /**
     * @brief setBoostParameters for the adaboost cascade
     * @param pRoundsOfTraining - rounds of training (amount of weak learner)
     * @param pWeight_trim_rate
     * @param pMaxDepthOfTrees - depth of the tree weak learner
     * @param pBoostingMethod - boosting method DISCRETE=0, REAL=1, LOGIT=2, GENTLE=3;
     * @param pcvFolds - the folds of the n-fold cross validation
     * @param Stages - the quantity of the stages you want to use
     * @param minTPR - the minimum true positive rate for each stage
     */
    void inline trainClassifier(uint pRoundsOfTraining,
            float pWeight_trim_rate,
            int pMaxDepthOfTrees,
            int pBoostingMethod,
            int pcvFolds){

        mRoundsOfTraining = pRoundsOfTraining;
        weight_trim = pWeight_trim_rate;
        maxDepthOfTrees = pMaxDepthOfTrees;
        boostingMethod = pBoostingMethod;
        cvFolds = pcvFolds;

    	this->adapt(mPosSamples,mNegSamples);
    }

    void inline setThreshold(float threshold){
    	mParams->mThreshold=threshold;
    }

    virtual void inline initialize(boost::shared_ptr<AdaboostClassifierParams> adaboostClassifierParams){
    	mParams=adaboostClassifierParams;
    	this->loadOpenCv();
    }

private:
    void adapt(std::vector<std::vector<float> > const& tPosSamples,
               std::vector<std::vector<float> > const& tNegSamples);

protected:
    boost::shared_ptr<AdaboostClassifierParams> mParams;

private:
    std::vector<std::vector<float> > mPosSamples;
    std::vector<std::vector<float> > mNegSamples;

    // Opencv paramter
    int mRoundsOfTraining;
    int maxDepthOfTrees;
    float weight_trim;
    int cvFolds;
    int boostingMethod; ///< DISCRETE=0, REAL=1, LOGIT=2, GENTLE=3;
};

}
}

#endif
