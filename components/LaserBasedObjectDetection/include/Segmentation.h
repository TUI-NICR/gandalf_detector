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
 * @file Segmentation.h
 *    header File for some segmentation stuff
 *
 * @author Tim Wengefeld, Christoph Weinrich
 * @date   2014/08/22
 */

#include <RangeScanWithBackgroundModel.h>
#include <geometry/Point.h>

using namespace mira;
using namespace mira::robot;

namespace mira{
namespace laserbasedobjectdetection{

///////////////////////////////////////////////////////////////////////////////

void filterSmallFGSegments(RangeScanWithBackgroundModel & rangeScan,uint const& minSegmentPoints,float const& jumpDistance,float const& BGJumpDistance);

Point2f getGroundTruth(RangeScanWithBackgroundModel const& rangeScan,float const& backgroundJD);

std::vector<RangeSegmentLabeled> getLabeledRangeSegments(RangeScanWithBackgroundModel const& rangeScan,float JumpDistance,float BGJumpDistance);

std::vector<Point2f> getPoints(RangeScan const& rangeScan);

std::vector<bool> getFGClassifikation(RangeScanWithBackgroundModel const& rangeScan,float const& BGJumpDiastance);

std::vector<uint> getBreakPoints(RangeScan const& rangeScan,float const& jumpDistance);

std::vector<RangeSegment> getRangeSegments(RangeScan const& rangeScan,float JumpDistance);

std::vector<Point2f> getRangeSegmentsCenter(RangeScan const& rangeScan,float JumpDistance,uint minSegmentSize);

///////////////////////////////////////////////////////////////////////////////

}
}
