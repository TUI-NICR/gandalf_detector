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
 * @file RangeScanWithBackgroundModel.h
 *    header File for the a Range Scan with background model
 *
 * @author Tim Wengefeld, Christoph Weinrich
 * @date   2014/08/22
 */

#ifndef RangeScanWithBackgroundModel_H
#define RangeScanWithBackgroundModel_H

#include <robot/RangeScan.h>
#include <geometry/Point.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <LaserRangeSegment.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/algorithm/string.hpp>

namespace mira {
namespace laserbasedobjectdetection {

using namespace mira::robot;
using namespace std;

///////////////////////////////////////////////////////////////////////////////

/**
 * class to represent a collection of neighboring laser points that are grouped into
 * a segment
 */
class RangeScanWithBackgroundModel : public RangeScan{

public:

	RangeScanWithBackgroundModel(){
	    this->aperture = 0;
	    this->coneAngle = 0.5;
	    this->stdError = 0;
	    this->maximumRange = 5.0;
	    this->minimumRange = 0.1;
	    mFrameID=1;
	};

	RangeScanWithBackgroundModel(const RangeScan & rangescan){
		this->range=rangescan.range;
		this->valid=rangescan.valid;
	    this->aperture = rangescan.aperture;
	    this->startAngle = rangescan.startAngle;
	    this->deltaAngle = rangescan.deltaAngle;
	    this->coneAngle = rangescan.coneAngle;
	    this->stdError = rangescan.stdError;
	    this->maximumRange = rangescan.maximumRange;
	    this->minimumRange = rangescan.minimumRange;
	    this->bgrange.resize(this->range.size(),0);
	    mFrameID=1;
	}

    virtual ~RangeScanWithBackgroundModel(){};

    void setBGModel(std::vector<float> bgmodel){
    	assert (bgrange.size()!=this->range.size());
    	bgrange=bgmodel;
    }

    /**
     * returns the ID of the Frame the segment belongs to
     * @return the frame identifier
     */
    int inline getFrameID() const{
        return mFrameID;
    }

    /**
     * Sets an Identifier at which Frame the segment was recordet
     * @param ID from the Frame the segment belongs to
     */
    void inline setFrameID(int frameid){
        mFrameID=frameid;
    }

	void inline setTimestamp(string timestamp){
		mTimestamp=timestamp;
	}

	string inline getTimestamp() const {
		return mTimestamp;
	}

    void setScan(std::vector<float> scan,float deltaAngle){
    	this->range=scan;
    	this->startAngle=(-1)*((float)this->range.size()-1)*0.5*deltaAngle;
    	this->deltaAngle=deltaAngle;
    	this->coneAngle=deltaAngle;
    }

    void setScan(std::vector<float> scan,std::vector<float> bgModel,float deltaAngle){
    	this->range=scan;
    	this->bgrange=bgModel;
    	this->startAngle=(-1)*((float)this->range.size()-1)*0.5*deltaAngle;
    	this->deltaAngle=deltaAngle;
    	this->coneAngle=deltaAngle;
    }

    /**
     * sets data from the Spinello fromat
     * @param str a line of the input file
     */
    void setScanFromString(std::string str,float deltaAngle){
    	this->range.clear();
        std::vector<std::string> tokens;
        boost::split(tokens, str, boost::is_any_of(" ")); // split the line strings by whitespace

        this->mFrameID = atoi(tokens[0].c_str());

        for(uint i=1;i<tokens.size()-1;i+=3){

            if(atoi(tokens[i+2].c_str())==0){ // <-- point is foreground
            	float x = boost::lexical_cast<double>(tokens[i].c_str());
            	float y = boost::lexical_cast<double>(tokens[i+1].c_str());
            	this->range.push_back(std::sqrt(x*x+y*y));
            	this->bgrange.push_back(std::sqrt(x*x+y*y)+1.0);
            }
            else{ // <-- point is background
            	float x = boost::lexical_cast<double>(tokens[i].c_str());
            	float y = boost::lexical_cast<double>(tokens[i+1].c_str());
            	this->range.push_back(std::sqrt(x*x+y*y));
            	this->bgrange.push_back(0.0);
            }
        }
    	this->startAngle=(-1)*((float)this->range.size()-1)*0.5*deltaAngle;
    	this->deltaAngle=deltaAngle;
    	this->coneAngle=deltaAngle;
    }

    template<typename Reflector>
    void reflect(Reflector& r) {
    	r.member("range", range, "Backgound rays of the laserscan");
    	r.member("aperture",aperture,"");
    	r.member("startAngle",startAngle,"");
    	r.member("deltaAngle",deltaAngle,"");
    	r.member("coneAngle",coneAngle,"");
    	r.member("stdError",stdError,"");
    	r.member("maximumRange",maximumRange,"");
    	r.member("minimumRange",minimumRange,"");
    	r.member("valid",valid,"");

        r.member("bgrange", bgrange, "Backgound rays of the laserscan");
        r.member("FrameID", mFrameID, "the ID of the laserscan");
        r.member("Timestamp", mTimestamp, "the timestamp of the laserscan");
    }

public :
    std::vector<float> bgrange;
private:
    int mFrameID;
    string mTimestamp;
};

}
}

#endif // LASERRANGESCAN_H
