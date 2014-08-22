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
 * @file GDIFFeatures.C
 * 	source file for the GDIF Features
 *
 * @author Tim Wengefeld, Christoph Weinrich
 * @date   2014/08/22
 */

#include <GDIFeatures.h>

namespace mira {
namespace laserbasedobjectdetection {
///////////////////////////////////////////////////////////////////////////////

void GDIFeatures::buildBoxFromCenter(RangeScan const& rangescan,
                                  	  	  	Point2f const& center,
                                  	  	BoundingBoxParams const& config){
    mCenter=center;
    mCenterPhi =std::atan2(mCenter.y(),mCenter.x());
    mCenterRange = std::sqrt(mCenter.x()*mCenter.x()+mCenter.y()*mCenter.y());
	mUseHighFreqFeats = config.mUseHighFreqFeats;

    mHeight=config.mBoxHeight;
    mWidth=config.mBoxWidth;
    mBinQuantity=config.mBinQuantity;

    mStartAngle=rangescan.startAngle;
    mEndAngle=rangescan.startAngle + rangescan.deltaAngle * (float)(rangescan.range.size()-1);
	mDeltaAngle=rangescan.deltaAngle;

    mSensorResolution=rangescan.deltaAngle;

    mOrthogonalAngle=mCenterPhi+M_PI/2.0;

    //calulate the edgepoints for the bins and initialize the feature vector;
    float dist=mWidth/float(mBinQuantity);
    for(int i=0;i<mBinQuantity;i++){
        Point2f newpoint(mCenter.x()+std::cos(mOrthogonalAngle)*((dist*i)-mWidth/2.0),mCenter.y()+std::sin(mOrthogonalAngle)*((dist*i)-mWidth/2.0));
        mBinEndPoints.push_back(newpoint);
        mBinEndPointAngles.push_back(std::atan2(newpoint.y(),newpoint.x()));
        if(config.mUseHighFreqFeats){
			mRadialFeatures.push_back(NaNf); // minimum value
			mRadialFeatures.push_back(NaNf); // maximum value
        }
        mRadialFeatures.push_back(NaNf); // average value
    }

    //mRightPoint=mBinEndPoints[0];
    //mLeftPoint=mBinEndPoints[mBinEndPoints.size()-1];

    mStartIndex = std::floor(std::abs(mStartAngle - mBinEndPointAngles[0]) / mSensorResolution);
    mEndIndex   = std::ceil(std::abs(mStartAngle - mBinEndPointAngles[mBinEndPointAngles.size()-1]) / mSensorResolution);

    if(mStartIndex<0)mStartIndex=0;
    if(mEndIndex>(int)rangescan.range.size()-1)mEndIndex=rangescan.range.size()-1;
}

void GDIFeatures::buildBoxFromLeft(RangeScan const& rangescan,
	  	  						   Point2f const& left,
	  	  						   BoundingBoxParams const& config){
	mHeight=config.mBoxHeight;
	mWidth=config.mBoxWidth;
	mBinQuantity=config.mBinQuantity;
	mUseHighFreqFeats = config.mUseHighFreqFeats;

	mStartAngle=rangescan.startAngle;
	mEndAngle=rangescan.startAngle + rangescan.deltaAngle * (float)(rangescan.range.size()-1);
	mDeltaAngle=rangescan.deltaAngle;

	mSensorResolution=rangescan.deltaAngle;

    // calulate the center of the box
    float GK=(mWidth/2.0)-config.mBoxFromLeftOffset;
    float HYP=std::sqrt(left.x()*left.x()+left.y()*left.y());
    float alpha = std::asin(GK/HYP);
    float AK= std::sqrt(HYP*HYP-GK*GK);
    mCenter = Point2f(AK,std::atan2(left.y(),left.x())-alpha);
    mCenterRange=std::sqrt(mCenter.x()*mCenter.x()+mCenter.y()*mCenter.y());
    mCenterPhi= std::atan2(mCenter.y(),mCenter.x());
    mOrthogonalAngle=mCenterPhi+M_PI/2.0;

    //calculate the edgepoints for the bins and initialize the feature vector;
    float dist=mWidth/float(mBinQuantity);
    for(int i=0;i<mBinQuantity;i++){
        Point2f newpoint(mCenter.x()+std::cos(mOrthogonalAngle)*((dist*i)-mWidth/2.0),mCenter.y()+std::sin(mOrthogonalAngle)*((dist*i)-mWidth/2.0));

        mBinEndPoints.push_back(newpoint);
        mBinEndPointAngles.push_back(std::atan2(newpoint.y(),newpoint.x()));
        if(config.mUseHighFreqFeats){
			mRadialFeatures.push_back(NaNf); // minimum value
			mRadialFeatures.push_back(NaNf); // maximum value
        }
        if(i!=mBinQuantity)mRadialFeatures.push_back(NaNf); // average value
    }
    //mRightPoint.setCartesianCoordiantes(mBinEndPoints[0].x(),mBinEndPoints[0].y());
    //mLeftPoint.setCartesianCoordiantes(mBinEndPoints[mBinEndPoints.size()-1].x(),mBinEndPoints[mBinEndPoints.size()-1].y());

    mStartIndex = std::floor(std::abs(mStartAngle - mBinEndPointAngles[0]) / mSensorResolution);
    mEndIndex   = std::ceil(std::abs(mStartAngle - mBinEndPointAngles[mBinEndPointAngles.size()-1]) / mSensorResolution);

    if(mStartIndex<0)mStartIndex=0;
    if(mEndIndex>(int)rangescan.range.size()-1)mEndIndex=rangescan.range.size()-1;
}

void GDIFeatures::calcRadialFeatures(std::vector<float> const& rays,std::vector<float> const& angles){

	int binindex = 0;
    int pointsinsidebin[mBinQuantity];
    for(int i=0;i<mBinQuantity;i++)pointsinsidebin[i]=0;

    for(int i=mStartIndex;i<=mEndIndex;i++){
        //if no points fall in this bin skip it
        while(angles[i]>mBinEndPointAngles[binindex+1]&&binindex<mBinQuantity-1)binindex++;
        float diffRange = this->diffRange(rays[i],angles[i]);

        //normalize to -Height/2.0 ... Height/2.0
        if(diffRange>mHeight/2.0f)diffRange=mHeight/2.0f;
        if(diffRange<(-1.0f)*mHeight/2.0f)diffRange=(-1.0f)*mHeight/2.0f;

        if(mUseHighFreqFeats){
			if(mRadialFeatures[(binindex*3)]>diffRange||std::isnan(mRadialFeatures[(binindex*3)]))mRadialFeatures[(binindex*3)]=diffRange;
			if(mRadialFeatures[(binindex*3)+1]<diffRange||std::isnan(mRadialFeatures[(binindex*3)+1]))mRadialFeatures[(binindex*3)+1]=diffRange;
			if(std::isnan(mRadialFeatures[(binindex*3)+2]))mRadialFeatures[(binindex*3)+2]=diffRange;
			else{
				mRadialFeatures[(binindex*3)+2]+=diffRange;
			}
        }
        else{
			if(std::isnan(mRadialFeatures[binindex]))mRadialFeatures[binindex]=diffRange;
			else mRadialFeatures[binindex]+=diffRange;
        }
        pointsinsidebin[binindex]++;
    }
    /*for(uint i;i<mRadialFeatures.size();i+=3){
    	cout << mRadialFeatures[i]<< " " <<mRadialFeatures[i+1]<< " "<< mRadialFeatures[i+2] << endl;
    }*/
    // average the average values of the bins
    for(int i=0;i<mBinQuantity;i++){
    	if(mUseHighFreqFeats){
    		if(pointsinsidebin[i]>0)mRadialFeatures[(i*3)+2]/=pointsinsidebin[i];
    	}
    	else{
    		if(pointsinsidebin[i]>0)mRadialFeatures[i]/=pointsinsidebin[i];
    	}
    }
    /*for(uint i;i<mRadialFeatures.size();i+=3){
    	cout << mRadialFeatures[i]<< " " <<mRadialFeatures[i+1]<< " "<< mRadialFeatures[i+2] << endl;
    }*/
    /*for(uint i;i<mRadialFeatures.size();i+=3){
    	cout << mRadialFeatures[i]<< " " <<mRadialFeatures[i+1]<< " "<< mRadialFeatures[i+2] << endl;
    }*/

    //Interpolate features for empty bins TODO : integration in prev loop
    for(int i=0;i<mBinQuantity;i++){
		if(mBinEndPointAngles[i]<mStartAngle||mBinEndPointAngles[i]>mEndAngle){
			if(mUseHighFreqFeats){
				if(std::isnan(mRadialFeatures[(i*3)])){
					mRadialFeatures[(i*3)]=-(mHeight)/2;
					mRadialFeatures[(i*3)+1]=-(mHeight)/2;
					mRadialFeatures[(i*3)+2]=-(mHeight)/2;
				}
			}
			else{
				if(std::isnan(mRadialFeatures[(i)])){
					mRadialFeatures[i]=-(mHeight)/2;
				}
			}
		}
		else{
			if(std::isnan(mRadialFeatures[(i*3)])){
				int prevIndex = std::floor((mBinEndPointAngles[i]-mStartAngle)*mDeltaAngle);
				// temp fix the real issue
				//if(prevIndex<0)prevIndex=0;
				//if(prevIndex>(int)rays.size()-2)prevIndex=rays.size()-2;

				float a1 = (mCenterRange*rays[prevIndex]*std::cos(mCenterPhi-angles[prevIndex]))/mCenterRange;
				float a2 = mCenterRange-a1;
				float b1 = rays[prevIndex];

				float diffRangePrev = (-1.0)*a2*b1/a1;
				//normalize to -Height/2.0 ... Height/2.0
				if(diffRangePrev>mHeight/2.0f)diffRangePrev=mHeight/2.0f;
				if(diffRangePrev<(-1.0f)*mHeight/2.0f)diffRangePrev=(-1.0f)*mHeight/2.0f;

				a1 = (mCenterRange*rays[prevIndex+1]*std::cos(mCenterPhi-angles[prevIndex+1]))/mCenterRange;
				a2 = mCenterRange-a1;
				b1 = rays[prevIndex+1];

				float diffRangeNext = (-1.0)*a2*b1/a1;
				//normalize to -Height/2.0 ... Height/2.0
				if(diffRangeNext>mHeight/2.0f)diffRangeNext=mHeight/2.0f;
				if(diffRangeNext<(-1.0f)*mHeight/2.0f)diffRangeNext=(-1.0f)*mHeight/2.0f;

				if(mUseHighFreqFeats){
					mRadialFeatures[(i*3)]=(diffRangePrev+diffRangeNext)/2;
					mRadialFeatures[(i*3)+1]=(diffRangePrev+diffRangeNext)/2;
					mRadialFeatures[(i*3)+2]=(diffRangePrev+diffRangeNext)/2;
				}
				else{
					mRadialFeatures[i]=(diffRangePrev+diffRangeNext)/2;
				}
			}
		}
	}
    /*for(uint i;i<mRadialFeatures.size();i+=3){
    	cout << mRadialFeatures[i]<< " " <<mRadialFeatures[i+1]<< " "<< mRadialFeatures[i+2] << endl;
    }*/
}

///////////////////////////////////////////////////////////////

} // namespace legdetectordepthhistogram
} // namespace mira
