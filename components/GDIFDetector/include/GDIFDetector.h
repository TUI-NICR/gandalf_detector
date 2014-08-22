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
 * @file GDIFDetector.h
 *    header File for the GDIF Detector
 *
 * @author Tim Wengefeld,Christoph Weinrich
 * @date   2014/08/22
 */

#include <AdaboostClassifier.h>
#include <BoundingBoxParams.h>
#include <Segmentation.h>
#include <SegmentationParams.h>
#include <GDIFeatures.h>

using namespace mira;
using namespace mira::robot;
using namespace mira::adaboosttreeclassifier;

namespace mira { namespace laserbasedobjectdetection {

///////////////////////////////////////////////////////////////////////////////

class GDIFDetector {
private:
    string mClassifierPath;
    boost::shared_ptr<AdaboostClassifierParams> mAdaboostParams;
    BoundingBoxParams mBoundingBoxParams;
    SegmentationParams mSegmentationParams;
    AdaboostClassifier mClassifier;
    std::vector<float> mAngles;
    bool firstScan;
    //std::vector<RangeSegment> mRangeSegments;

public:

    void inititalize(boost::shared_ptr<AdaboostClassifierParams> adaboostParams,
    		 	 	 SegmentationParams segmentationParams,
    		 	 	 BoundingBoxParams boundingBoxParams);

    std::vector<Point2f> classifyScan(RangeScan const& rangeScan);
};

///////////////////////////////////////////////////////////////////////////////

}}
