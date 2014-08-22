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
 * @file GDIFeatures.h
 * 	header file for the GDIF Features
 *
 * @author Tim Wengefeld, Christoph Weinrich
 * @date   2014/08/22
 */

#ifndef BOUNDINGBOXSEGMENT_H
#define BOUNDINGBOXSEGMENT_H

#include <limits>
#include <BoundingBoxParams.h>
#include <robot/RangeScan.h>
#include <geometry/Point.h>


namespace mira {
namespace laserbasedobjectdetection {

using namespace std;
using namespace mira::robot;

const float NaNf =std::numeric_limits<float>::signaling_NaN();

class GDIFeatures{
public :
	GDIFeatures(){}
    ~GDIFeatures(){}

    /** builds the bounding box by using the center of the segment  as the reference point for the center of the box
     *  @param points of the Laserscan
     *  @param the rangesegment you want to classifie
     *  @param the width of the box
     *  @param the height of the box
     *  @param the binquantity of the box of the box
     *  @param useMoreFeatures experimental!!! - if true the features of the box will also contain the old features of the segment
     */
    void buildBoxFromCenter(RangeScan const& rangescan,
                  Point2f const& center,
                  BoundingBoxParams const& config);

    /** builds the bounding box by using the endpoint of the segment as the reference point for the left of the box
     *  @param points of the Laserscan
     *  @param the rangesegment you want to classifie
     *  @param the width of the box
     *  @param the height of the box
     *  @param the binquantity of the box of the box
     *  @param experimental - doesnt work right no
     */
    void buildBoxFromLeft(RangeScan const& rangescan,
    			  Point2f const& left,
    			  BoundingBoxParams const& config);

    /**  calculate the features of the box
     * @param the points of the laserscan
     */
    void calcRadialFeatures(std::vector<float> const& range,std::vector<float> const& angles);

    /** checks that the bounding box is inside a valid angle of the rangescan
     *  @return true if the box is valid, false else
     */
    bool inline isValid(){
        if(mBinEndPointAngles[0]>mBinEndPointAngles[mBinEndPointAngles.size()-1]){
            return false;
        }
        if(mBinEndPointAngles[0]>mStartAngle&&mBinEndPointAngles[mBinEndPointAngles.size()-1]<mEndAngle){
            return true;
        }
        else{
            return false;
        }
    }

    /**  will return a value for a point if he is inside or not
     * @param the point you want to check
     * @return -2 is out of range
     * @return -1 is before
     * @return 0 is inside
     * @return 1 is behinde
     */
    inline int isInside(Point2f const& point) const;

    /**  will return the rigth angle of the box
     * @return the angle of the right side of the box
     */
    float inline getMinAngle() const {return mBinEndPointAngles[0];}

    /**  will return the left angle of the box
     * @return the angle of the left side of the box
     */
    float inline getMaxAngle()const {return mBinEndPointAngles[mBinEndPointAngles.size()-1];}

    /**  will return the width of the box
     * @return the width of the box
     */
    float inline getWidth() const {return mWidth;}

    /**  will return the height of the box
     * @return the height of the box
     */
    float inline getHeight() const {return mHeight;}

    /**  will return the index of the first point outside the range of the box from the right
     * @return the index of the first point outside the range of the box from the right
     */
    float inline getStartIndex() const {return mStartIndex;}

    /**  will return the index of the first point outside the range of the box from the left
     * @return the index of the first point outside the range of the box from the left
     */
    float inline getEndIndex() const {return mEndIndex;}

    /**  will return center of the box
     * @return the center of the box
     */
    Point2f inline getCenter() const {return mCenter;}

    /**  will return the radial features of the box
     * @return the features of the box
     */
    std::vector<float> inline getRadialFeatures() const {
		return mRadialFeatures;
    }

    /**  will return the end-points of the bins (on the middleline)
     *   just for visualization with no further use
     * @return the points of the bins
     */
    std::vector<Point2f> inline getBinEndPoints() const{return mBinEndPoints;}


    //serialization
    template<typename Reflector>
    void reflect(Reflector& r) {
    	int version = r.version(2);
		r.member("Center", mCenter, "center of the box");
		r.member("BinEndPoints", mBinEndPoints, "the end points of the bins");
		r.member("width", mWidth, "width of the box");
		r.member("height", mHeight, "height of the box");
		r.member("RadialFeatures", mRadialFeatures, "the radial features of the box");
    }

private :
    /**
     * @brief distance function with trigonometrie and Cross-multiplication (use this one!!!!!)
     * @param center centerpoint of the box
     * @param point the point you want to calculate the distance for
     * @return distance of the point to the middleline of the bounding box
     */
    float inline diffRange(float const& range,float const& angle);

    Point2f mCenter; // centerpoint of the Box
    float mCenterRange;
    float mCenterPhi;

    //Point2f mLeftPoint; // left point of the orthogonal linesegment trough the centerpoint
    //Point2f mRightPoint; // right point of the orthogonal linesegment trough the centerpoint

    float mOrthogonalAngle; // the orthogonal angle of the linesegment through the center
    std::vector<Point2f> mBinEndPoints; // the ending points of every bin from left to right (seen from the center)
    std::vector<float> mBinEndPointAngles;

    float mHeight,mWidth; // width and height of the Box
    int mBinQuantity; // the Quantity of the bins
    float mStartAngle,mEndAngle,mDeltaAngle; // the starting and ending angle of the laserscan
    int mStartIndex,mEndIndex; // the starting and ending index of the scanpoints which fall in a bin
    float mSensorResolution; //the resolutions of the sensor in radians
    bool mUseHighFreqFeats;

    std::vector<float > mRadialFeatures; // the features of the segment for the radial projection
};

int inline GDIFeatures::isInside(Point2f const& point) const{
	float range = std::sqrt(point.x()*point.x()+point.y()*point.y());
	float phi = std::atan2(point.y(),point.x());
    if(phi>=mBinEndPointAngles[0]&&phi<=mBinEndPointAngles[mBinEndPointAngles.size()-1]){
        float a1 = (mCenterRange*range*std::cos(mCenterPhi-phi))/mCenterRange;
        float a2 = mCenterRange-a1;
        float b1 = range;

        float diffRange = (-1.0)*a2*b1/a1;
        if(diffRange<(-1.0f)*mHeight/2.0f) {return -1;} // before
        else if(diffRange>mHeight/2.0f) {return 1;} // behinde
        else {return 0;} // inside
    }
    else {return -2;} // out of angle
}

float inline GDIFeatures::diffRange(float const& range,float const& angle){
    float a1 = range*std::cos(mCenterPhi-angle);
    float a2 = mCenterRange-a1;
    float b1 = range;
    return (-1.0)*a2*b1/a1;
}

}
}

#endif // BOUNDINGBOXSEGMENT_H
