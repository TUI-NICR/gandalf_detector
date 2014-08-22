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
 * @file LaserRangeSegment.C
 *    source File for laser Range Segment
 *
 * @author Tim Wengefeld, Christoph Weinrich
 * @date   2014/08/22
 */


#include <LaserRangeSegment.h>
//#include <Point2f.h>

using namespace std;

//MIRA_CLASS_SERIALIZATION(mira::laserbasedobjectdetection::RangeSegment, mira::Object );
//MIRA_CLASS_SERIALIZATION(mira::laserbasedobjectdetection::RangeSegmentLabeled,
//        mira::laserbasedobjectdetection::RangeSegment );

namespace mira {
namespace laserbasedobjectdetection {

///////////////////////////////////////////////////////////////////////////////

RangeSegment::RangeSegment() {
    CenterIsAvailible=false;
    lastPointFromPrevAvailible=false;
    firstPointFromNextAvailible=false;
	CenterIsAvailible=false;
	mPoints.clear();
}

RangeSegment::~RangeSegment() {
	// TODO Auto-generated destructor stub
}

void RangeSegment::addPoint(Point2f const& pPoint) {
	mPoints.push_back(pPoint);
}

void RangeSegment::addPoint(float pX, float pY) {
    mPoints.push_back(Point2f(pX, pY));
}

void RangeSegment::setPoints(vector<Point2f> const& pPoints) {
	mPoints = pPoints;
}
vector<Point2f> const& RangeSegment::getPoints() const {
	return mPoints;
}

void RangeSegment::clearPoints() {
	mPoints.clear();
}

uint RangeSegment::size() const {
	return mPoints.size();
}

void RangeSegment::setLastPointFromPrevSegment(Point2f const& pLastPointFromPrevSegment) {
	mLastPointFromPrevSegment = pLastPointFromPrevSegment;
}

void RangeSegment::setFirstPointFromNextSegment(Point2f const& pFirstPointFromNextSegment) {
	mFirstPointFromNextSegment = pFirstPointFromNextSegment;
}

Point2f RangeSegment::getLastPointFromPrevSegment() const {
	return mLastPointFromPrevSegment;
}

Point2f RangeSegment::getFirstPointFromNextSegment() const{
	return mFirstPointFromNextSegment;
}

float RangeSegment::getSegmentWidth() const{
    return std::sqrt(std::pow(mPoints[0].x()-mPoints[mPoints.size()-1].x(),2)+std::pow(mPoints[0].y()-mPoints[mPoints.size()-1].y(),2));
}

//------------------------------------------------------------------------------
RangeSegmentLabeled::RangeSegmentLabeled() {
	init();
}

RangeSegmentLabeled::~RangeSegmentLabeled() {

}

void RangeSegmentLabeled::init() {
	clearPoints();
	clearRangeRays();
	mLabel = SegmentLabel::BG;

}

void RangeSegmentLabeled::setLabel(SegmentLabel pLabel) {
	mLabel = pLabel;
}

SegmentLabel RangeSegmentLabeled::getLabel() const {
	return mLabel;
}

void RangeSegmentLabeled::addRaysIdx(uint iRayIndex) {
	mRangeRays.push_back(iRayIndex);
}

std::vector<uint> const& RangeSegmentLabeled::getRangeRays() const {
	return mRangeRays;
}

void RangeSegmentLabeled::clearRangeRays() {
	mRangeRays.clear();
}

//------------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////

}
}
