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
 * @file Segmentation.C
 *    source File Segmentation stuff
 *
 * @author Tim Wengefeld, Christoph Weinrich
 * @date   2014/08/22
 */

#include <Segmentation.h>

namespace mira{
namespace laserbasedobjectdetection{

void filterSmallFGSegments(RangeScanWithBackgroundModel & rangeScan,uint const& minSegmentPoints,float const& jumpDistance,float const& BGJumpDistance){
	std::vector<uint> breakpoints;
	breakpoints.push_back(0);

	int currentLabel,lastLabel; // 0=FG, 1=BG;
	if(rangeScan.range[0]<rangeScan.bgrange[0]-BGJumpDistance)lastLabel=0;
	else lastLabel=1;

    for(uint i=1;i<rangeScan.range.size();i++){
        float tdiff;

        tdiff=std::abs(rangeScan.range[i-1]-rangeScan.range[i]); // jump distance between 2 consecutive beams

    	if(rangeScan.range[i]<rangeScan.bgrange[i]-BGJumpDistance)currentLabel=0;
    	else currentLabel=1;

        if(tdiff>jumpDistance||currentLabel!=lastLabel){ // begin new Segment
        	breakpoints.push_back(i);
        }
        lastLabel=currentLabel;
    }
    breakpoints.push_back(rangeScan.range.size()-1);
	for(uint i=1;i<breakpoints.size();i++){
		if(breakpoints[i]-breakpoints[i-1]<minSegmentPoints){
			for(uint j=breakpoints[i-1];j<breakpoints[i];j++){
				rangeScan.bgrange[j]=0.0;
			}
		}
	}
}

Point2f getGroundTruth(RangeScanWithBackgroundModel const& rangeScan,float const& backgroundJD){
	float centerR=0;
	float centerPhi=0;
	int fgCounter=0;
	for(uint i=0;i<rangeScan.range.size();i++){
		if(rangeScan.range[i]<rangeScan.bgrange[i]-backgroundJD){
			fgCounter++;
			centerR+=rangeScan.range[i];
			centerPhi+=rangeScan.startAngle+rangeScan.deltaAngle*(float)i;
		}
	}
	centerR/=(float)fgCounter;
	centerPhi/=(float)fgCounter;
	return Point2f(cos(centerPhi)*centerR,sin(centerPhi)*centerR);
}

std::vector<RangeSegmentLabeled> getLabeledRangeSegments(RangeScanWithBackgroundModel const& rangeScan,float JumpDistance,float BGJumpDistance){
	std::vector<uint> breakpoints = getBreakPoints(rangeScan,JumpDistance);
	std::vector<Point2f> points = getPoints(rangeScan);
	std::vector<bool> isFG = getFGClassifikation(rangeScan,BGJumpDistance);
	std::vector<RangeSegmentLabeled> rangeSegments;

	for(uint i=1;i<breakpoints.size();i++){
		vector<Point2f> segPoints(points.begin()+breakpoints[i-1],points.begin()+breakpoints[i]);
		RangeSegmentLabeled segment;
		segment.setPoints(segPoints);
		int fgcounter=0;
		for(uint j=breakpoints[i-1];j<breakpoints[i];j++){
			if(isFG[j])fgcounter++;
			else fgcounter--;
		}
		if(fgcounter>=0)segment.setLabel(SegmentLabel::FG);
		else segment.setLabel(SegmentLabel::BG);
		rangeSegments.push_back(segment);
	}
	return rangeSegments;
}

std::vector<Point2f> getPoints(RangeScan const& rangeScan) {
	std::vector<Point2f> points;
	for(uint i=0;i<rangeScan.range.size();i++){
		float x = rangeScan.range[i] * cos(rangeScan.startAngle + rangeScan.deltaAngle * (float) i );
		float y = rangeScan.range[i] * sin(rangeScan.startAngle + rangeScan.deltaAngle * (float) i );
		points.push_back(Point2f(x,y));
	}
	return points;
}

std::vector<bool> getFGClassifikation(RangeScanWithBackgroundModel const& rangeScan,float const& BGJumpDiastance){
	std::vector<bool> isFG;
	for(uint i=0;i<rangeScan.range.size();i++){
		if(rangeScan.range[i]<rangeScan.bgrange[i]-BGJumpDiastance) isFG.push_back(true);
		else isFG.push_back(false);
	}
	return isFG;
}

std::vector<uint> getBreakPoints(RangeScan const& rangeScan,float const& jumpDistance){
	std::vector<uint> breakPoints;
	breakPoints.push_back(0);
    for(uint i=1;i<rangeScan.range.size();i++){
        float tdiff;

        tdiff=std::abs(rangeScan.range[i-1]-rangeScan.range[i]); // jump distance between 2 consecutive beams

        if(tdiff>jumpDistance){ // begin new Segment
        	breakPoints.push_back(i);
        }
    }
    breakPoints.push_back(rangeScan.range.size()-1);
    return breakPoints;
}

std::vector<RangeSegment> getRangeSegments(RangeScan const& rangeScan,float JumpDistance){
	std::vector<uint> breakpoints = getBreakPoints(rangeScan,JumpDistance);
	std::vector<Point2f> points = getPoints(rangeScan);
	std::vector<RangeSegment> rangeSegments;

	for(uint i=1;i<breakpoints.size();i++){
		vector<Point2f> segPoints(points.begin()+breakpoints[i-1],points.begin()+breakpoints[i]);
		RangeSegment segment;
		segment.setPoints(segPoints);
		rangeSegments.push_back(segment);
	}
	return rangeSegments;
}

std::vector<Point2f> getRangeSegmentsCenter(RangeScan const& rangeScan,float JumpDistance,uint minSegmentSize){
	std::vector<uint> breakpoints = getBreakPoints(rangeScan,JumpDistance);
	std::vector<Point2f> CenterPoints;

	float CenterRange;
	float CenterPhi;
	for(uint i=1;i<breakpoints.size();i++){
		if(breakpoints[i]-breakpoints[i-1]<minSegmentSize)continue;
		CenterRange=0.0;
		CenterPhi=0.0;
		for(uint j=breakpoints[i-1];j<breakpoints[i];j++){
			CenterRange+=rangeScan.range[j];
		}
		CenterRange/=(breakpoints[i]-breakpoints[i-1]);
		CenterPhi=rangeScan.startAngle + rangeScan.deltaAngle*float(breakpoints[i-1]) + rangeScan.deltaAngle*float(breakpoints[i]-breakpoints[i-1])*0.5f;
		CenterPoints.push_back(Point2f(CenterRange*std::cos(CenterPhi),CenterRange*std::sin(CenterPhi)));
	}
	return CenterPoints;
}

}
}
