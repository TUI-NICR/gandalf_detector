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
 * @file LaserRangeSegment.h
 *    header File for a LaserrangeSegment
 *
 * @author Tim Wengefeld, Christoph Weinrich, Michael Volkard
 * @date   2014/08/22
 */

#ifndef RANGESEGMENT_H_
#define RANGESEGMENT_H_

#include <geometry/Point.h>

namespace mira {
namespace laserbasedobjectdetection {

///////////////////////////////////////////////////////////////////////////////
enum SegmentLabel{ FG=0 , BG=1 };
/**
 * class to represent a collection of neighboring laser points that are grouped into
 * a segment
 */
class RangeSegment{ //: public mira::Object {
    //MIRA_OBJECT( mira::laserbasedobjectdetection::RangeSegment );

public:

	RangeSegment();
	virtual ~RangeSegment();

	/**
	 * add point to segment
	 * @param pPoint the point to be added to the segment
	 */
    void addPoint(Point2f const& pPoint);

	/**
	 * add point with x and y coordinates to segment
	 * @param pX x coordinate of point
	 * @param pY y coordinate of point
	 */
	void addPoint(float pX, float pY);

	/**
	 * sets all points of segment with given vector of points
	 * @param pPoints vector of points that the segment should represent
	 */
    void setPoints(std::vector<Point2f> const& pPoints);
	/**
	 * returns points of the segments
	 * @return vector of points of the segment
	 */
    std::vector<Point2f> const& getPoints() const;

	/**
	 * clears the points of the segment
	 */
	void clearPoints();
	/**
	 * number of points in the segment
	 * @return number of segment points
	 */
	uint size() const;

	/**
	 * sets the last point of the previous segment (algorithm specific, see [Arras07])
	 * @param pLastPointFromPrevSegment last point of previous segment
	 */
    void setLastPointFromPrevSegment(Point2f const& pLastPointFromPrevSegment);
	/**
	 * sets the first point of the next segment (algorithm specific, see [Arras07])
	 * @param pFirstPointFromNextSegment first point of next segment
	 */
	void setFirstPointFromNextSegment(
            Point2f const& pFirstPointFromNextSegment);
	/**
	 * returns the last point of the previous segment
	 * @return last point of previous segment
	 */
    Point2f getLastPointFromPrevSegment() const;
	/**
	 * returns the first point of the next segment
	 * @return first point of next segment
	 */
    Point2f getFirstPointFromNextSegment() const;
	
    /**
     * @return the width of the segment
     */
    float getSegmentWidth() const;

    /**
     * @brief calculate and return the center of the segment
     * @return the center of the segment
     */
    Point2f inline getCenter(){
        if(CenterIsAvailible){
            return mSegmentCenter;
        }
        float centerX=0;
        float centerY=0;

        for(uint i=0;i<mPoints.size();i++){
            centerX+=mPoints[i].x();
            centerY+=mPoints[i].y();
        }
        centerX/=mPoints.size();
        centerY/=mPoints.size();
        Point2f segmentCenter;
        segmentCenter = Point2f(centerX,centerY);
        CenterIsAvailible=true;
        mSegmentCenter=segmentCenter;
        return mSegmentCenter;
	}

    /**
     * @brief calculate and return the distance of the sensor
     * @return
     */
    float inline getDistToSensor(){
        if(!CenterIsAvailible){
            this->getCenter();
        }
        float range = std::sqrt(mSegmentCenter.x()*mSegmentCenter.x()+mSegmentCenter.y()*mSegmentCenter.y());
        return range;
    }

    Point2f inline getFirstPoint() const {
        return mPoints[0];
    }

    Point2f inline getLastPoint() const {
        return mPoints[mPoints.size()-1];
    }

	//serialization
	template<typename Reflector>
	void reflect(Reflector& r) {
    	int version = r.version(1);
		r.member("LastPointFromPrevSegment", mLastPointFromPrevSegment,
				"last point of previous segment");
		r.member("FirstPointFromNextSegment", mFirstPointFromNextSegment,
				"first point of next segment");
		r.member("Points", mPoints, "points of segment");
	}


protected:

	Point2f mLastPointFromPrevSegment;///<last point of previous segment
	Point2f mFirstPointFromNextSegment;///<first point of next segment
    std::vector<Point2f> mPoints;///< points of the segment
    Point2f mSegmentCenter; //to sort the clusters when using meanshift
    std::vector<RangeSegment*> occludetBy;//pointer to the Segment which partially occlude this Segment
    std::vector<RangeSegment*> cutBy;//pointer to the Segment which cut this Segment (these Segments containing points which lay behind this Segment)
    Point2f segmentCenter;
    bool CenterIsAvailible;
    bool lastPointFromPrevAvailible;
    bool firstPointFromNextAvailible;
};

/** Label of segments
 *  INCOMPLETE if at the Border of the Segments were invalid Scans
 *  NO_LEG if the Segment is Background
 *  NOT_LABELED for different purpose
 *  LEG if segment is foreground and contains exactly one other foreground segment inside its MaxLegDistance and both are not adjaszent
 *  MERGED_LEGS if there is just one foreground segment or two adjaszent foreground Segments inside the MaxLegDistance
 *  NOT_DECIDABLE if there are more than one other foreground segments inside the MaxLegDistance you can't decide if they are legs or the segmenting failed
 */
/*enum Label {
    INCOMPLETE = -2, NO_LEG = -1, NOT_LABELED = 0, LEG = 1, MERGED_LEGS = 2, NOT_DECIDABLE = 3
};*/

/**
 * class that represents a labeled segment
 * also saves which laser rays created the segment for nicer visualization
 */
class RangeSegmentLabeled: public RangeSegment {

    //MIRA_OBJECT( mira::laserbasedobjectdetection::RangeSegmentLabeled );

public:
	/**
	 * clears everything and sets label to NOT_LABELED
	 */
	RangeSegmentLabeled();
	virtual ~RangeSegmentLabeled();

	/**
	 * clears everything and sets label to NOT_LABELED
	 */
	void init();
	/**
	 * sets label to segment
	 * @param pLabel label should be of enum Label
	 */
	void setLabel(SegmentLabel pLabel);
	/**
	 * gets current label
	 * @return enum of label
	 */
	SegmentLabel getLabel() const;

	/**
	 * adds index of laser ray to segment
	 * @param iRayIndex
	 */
	void addRaysIdx(uint iRayIndex);
	/**
	 * returns the rays that created the segment
	 * @return vector of laser scan indices that created the segment
	 */
	std::vector<uint> const& getRangeRays() const;

	/**
	 * clears the mRangeRays vector
	 */
	void clearRangeRays();

	//serialization
	template<typename Reflector>
	void reflect(Reflector& r) {
		RangeSegment::reflect(r);
    	int version = r.version(1);
		r.member("Label", mLabel, "label of segment", BG);
	}

protected:
	SegmentLabel mLabel; ///< label of segment
	std::vector<uint> mRangeRays; ///< which range rays are in this segment (for visualization)
};

///////////////////////////////////////////////////////////////////////////////

}
}

#endif /* RANGESEGMENT_H_ */
