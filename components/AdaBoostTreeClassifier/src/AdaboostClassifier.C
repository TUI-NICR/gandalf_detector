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
 * @file AdaboostClassifier.C
 *    source File for the Adaboost Classifier
 *
 * @author Tim Wengefeld,Christoph Weinrich
 * @date   2014/08/22
 */

#include <AdaboostClassifier.h>
#include <fstream>

namespace mira {
namespace adaboosttreeclassifier {

void AdaboostClassifier::setTrainData(std::vector<std::vector<float> > posSamples,std::vector<std::vector<float> > negSamples){
    if(posSamples.size()==0||negSamples.size()==0){
        std::cerr << "no valid trainingset" << endl;
        return;
    }
    mPosSamples = posSamples;
    mNegSamples = negSamples;

    std::random_shuffle(mPosSamples.begin(),mPosSamples.end());
    std::random_shuffle(mNegSamples.begin(),mNegSamples.end());
}

void AdaboostClassifier::adapt(std::vector<std::vector<float> > const& tPosSamples,
                               std::vector<std::vector<float> > const& tNegSamples){

    // start trainings
    cout << "start training classifier with " << mPosSamples[0].size() << " features" << endl;
    cout << "###########################" << endl;
    cout << "POS " << tPosSamples.size() << " NEG " << tNegSamples.size() << endl;
    mParams->mFeatureVectorSize=mPosSamples[0].size();

    // convert feature vector to cv::Mat
    cv::Mat features = cv::Mat(tPosSamples.size()+tNegSamples.size(), tPosSamples[0].size(), CV_32F);
    cv::Mat classLabelResponses = cv::Mat(tPosSamples.size()+tNegSamples.size(), 1, CV_32S);

    for(uint row=0;row<tPosSamples.size();row++){
        for(uint col=0;col<tPosSamples[row].size();col++){
            features.at<float>(row, col) = tPosSamples[row][col];
        }
        classLabelResponses.at<int>(row, 0) = POS;
    }

    for(uint row=0;row<tNegSamples.size();row++){
        for(uint col=0;col<tNegSamples[row].size();col++){
            features.at<float>(tPosSamples.size()+row, col) = tNegSamples[row][col];
        }
        classLabelResponses.at<int>(tPosSamples.size()+row, 0) = NEG;
    }

    cv::BoostParams boostparm = cv::BoostParams(boostingMethod,
            mRoundsOfTraining, weight_trim, maxDepthOfTrees, false,	0);
    boostparm.cv_folds = cvFolds;

    this->train(features, CV_ROW_SAMPLE, classLabelResponses, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(),boostparm);
    this->mParams->mThreshold=0;
    cout << "training done" << endl;
}

void AdaboostClassifier::generateLearningCurve(int resoulution){

    ofstream oStream2;
    oStream2.open("LearningCurve.txt");

    float frac = mNegSamples.size()/mPosSamples.size();

    cout << "Begin Lerning Curve" << endl;
    this->clear();
    for(int i=10;i<mNegSamples.size()*0.8;i++){

        uint posTrainsize = i;
        uint negTrainsize = i*frac;

        if(posTrainsize>mPosSamples.size()*0.8||negTrainsize>mNegSamples.size()*0.8)break;

        std::vector<std::vector<float> >::const_iterator firsttrain = mPosSamples.begin();
        std::vector<std::vector<float> >::const_iterator lasttrain = mPosSamples.begin() + posTrainsize;
        std::vector<std::vector<float> >::const_iterator lasttest = mPosSamples.end();
        std::vector<std::vector<float> > tTrainPosSamples(firsttrain, lasttrain);
        std::vector<std::vector<float> > tTestPosSamples(lasttrain, lasttest);

        firsttrain = mNegSamples.begin();
        lasttrain = mNegSamples.begin() + negTrainsize;
        lasttest = mNegSamples.end();
        std::vector<std::vector<float> > tTrainNegSamples(firsttrain, lasttrain);
        std::vector<std::vector<float> > tTestNegSamples(lasttrain, lasttest);

        cout << "step " << i << endl;
        cout << "Trainsize "  << tTrainPosSamples.size() << " " << tTrainNegSamples.size() << endl;
        cout << "Testsize "  << tTestPosSamples.size() << " " << tTestNegSamples.size() << endl;

        //train calssifier
        this->adapt(tTrainPosSamples,tTrainNegSamples);

        //evaluate error over train and testset
        long TPtrain,FNtrain,TNtrain,FPtrain;
        long TPtest,FNtest,TNtest,FPtest;
        this->testClassifier(tTrainPosSamples,tTrainNegSamples,TPtrain,FNtrain,TNtrain,FPtrain);
        this->testClassifier(tTestPosSamples,tTestNegSamples,TPtest,FNtest,TNtest,FPtest);

        //calc BER for train- and testset
        float trainerror = 0.5*(float(FNtrain)/float(tTrainPosSamples.size())+float(FPtrain)/float(tTrainNegSamples.size()));
        float testerror = 0.5*(float(FNtest)/float(tTestPosSamples.size())+float(FPtest)/float(tTestNegSamples.size()));

        cout << "step : " << i << " || trainerror : " << trainerror << " || testerror : " << testerror << endl;
        cout <<  " | TPtrain  : " << TPtrain << " | FPtrain :" << FPtrain  << endl;
        oStream2 << "step " << i
                 << " trainerror " << trainerror
                 << " testerror " << testerror
                 << " TrainsetSize " << tTrainPosSamples.size()+tTrainNegSamples.size()
                 << " PosTrain " << tTrainPosSamples.size()
                 << " NegTrain " << tTrainNegSamples.size()
                 << " PosTest " << tTestPosSamples.size()
                 << " NegTest " << tTestNegSamples.size()
                 << " TPRTest " << TPtest/(TPtest+FNtest)
                 << " FPRTest " << FPtest/(FPtest+TNtest)
                 << endl;
    }
    oStream2.close();
}

void AdaboostClassifier::testClassifier(std::vector<std::vector<float> > const& tPosSamples,
                                     std::vector<std::vector<float> > const& tNegSamples,
                                     long &TP,
                                     long &FN,
                                     long &TN,
                                     long &FP){

    TP=0;FN=0;TN=0;FP=0;
    for(uint i=0;i<tPosSamples.size();i++){
        float result = this->apply(tPosSamples[i]);
        if(result>0)TP++;
        else FN++;
    }
    for(uint i=0;i<tNegSamples.size();i++){
        float result = this->apply(tNegSamples[i]);
        if(result>0)FP++;
        else TN++;
    }
}

float AdaboostClassifier::apply(std::vector<float> const &sample) const{
#ifdef Dbg
    if(sample.size()==0){
        std::cerr << "error::sample size 0" << std::endl;
        return 0;
    }
    else if(mFeatureVectorSize!=sample.size()){
        std::cerr << "feature size doesn't match the training" << std::endl;
        std::cerr << "trainsize : " << mFeatureVectorSize << std::endl;
        std::cerr << "applysize : " << sample.size() << std::endl;
        return 0;
    }
#endif

	cv::Mat cvtfeatures;

	cvtfeatures = cv::Mat(1, sample.size(), CV_32F);
	for (uint f = 0; f < sample.size(); ++f) {
		cvtfeatures.at<float>(0, f) = sample[f];
	}

	float result = this->predict(cvtfeatures,cv::Mat(),cv::Range::all(),false,true);
	//cout << "result"  << result << endl;
	//cout << "Threshold"  << mParams->mThreshold << endl;
    return result+mParams->mThreshold;
}

////////////////////////////////////////////////
}
}
