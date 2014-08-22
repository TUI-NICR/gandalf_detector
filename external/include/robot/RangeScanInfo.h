/*
 * Copyright (C) 2012 by
 *   MetraLabs GmbH (MLAB), GERMANY
 * and
 *   Neuroinformatics and Cognitive Robotics Labs (NICR) at TU Ilmenau, GERMANY
 * All rights reserved.
 *
 * Contact: info@mira-project.org
 *
 * Commercial Usage:
 *   Licensees holding valid commercial licenses may use this file in
 *   accordance with the commercial license agreement provided with the
 *   software or, alternatively, in accordance with the terms contained in
 *   a written agreement between you and MLAB or NICR.
 *
 * GNU General Public License Usage:
 *   Alternatively, this file may be used under the terms of the GNU
 *   General Public License version 3.0 as published by the Free Software
 *   Foundation and appearing in the file LICENSE.GPL3 included in the
 *   packaging of this file. Please review the following information to
 *   ensure the GNU General Public License version 3.0 requirements will be
 *   met: http://www.gnu.org/copyleft/gpl.html.
 *   Alternatively you may (at your option) use any later version of the GNU
 *   General Public License if such license has been publicly approved by
 *   MLAB and NICR (or its successors, if any).
 *
 * IN NO EVENT SHALL "MLAB" OR "NICR" BE LIABLE TO ANY PARTY FOR DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF
 * THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF "MLAB" OR
 * "NICR" HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * "MLAB" AND "NICR" SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND "MLAB" AND "NICR" HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS OR MODIFICATIONS.
 */

/**
 * @file RangeScanInfo.h
 *    Information about configuration of 2D range scans
 *
 * @author Tim Langner, Erik Einhorn
 * @date   2010/12/06
 */

#ifndef _MIRA_RANGESCANINFO_H_
#define _MIRA_RANGESCANINFO_H_

#include <math/Angle.h>
//#include <transform/Pose.h>

//#include <robot/RobotDataTypesExports.h>

namespace mira { namespace robot {

///////////////////////////////////////////////////////////////////////////////

/**
 * This class stores information about the configuration and geometry of
 * 2D range scans e.g. for laser, infrared or ultra-sonic sensors.
 * A laser would have several scans in the scan vector whereas the scan vector
 * of an ultra-sonic sensor would only contain a single scan.
 */
class //MIRA_ROBOT_DATATYPES_EXPORT
RangeScanInfo
{
public:
	/** @name Constructors and reflect */
	//@{

	RangeScanInfo() : 
		minimumRange(0.0f),
		maximumRange(1000.0f)
//		,scanTime(Duration::milliseconds(0))
	{}

	template<typename Reflector>
	void reflect(Reflector& r)
	{
		int version = r.version(2);
		r.property("StartAngle", startAngle, 
		           "The starting orientation of the first sensor cone in [deg]");
		r.property("DeltaAngle", deltaAngle, 
		           "The delta angle between neighboring cones in [deg]");
		r.property("ConeAngle", coneAngle, 
		           "The opening angle of a single cone in [deg]");
		r.property("Aperture", aperture, 
		           "The opening diameter (aperture) of the sensor in [m]");
		r.property("StdError", stdError, 
		           "The measuring error of the sensor in [m]");
		r.property("MinimumRange", minimumRange, 
		           "The minimum range of valid measurements "
		           "(measurements below this value result in BelowMinimum range code)");
		r.property("MaximumRange", maximumRange, 
		           "The the maximum range of valid measurements "
		           "(measurements above this value result in AboveMaximum range code)");
//		if (version>=2)
//			r.property("ScanTime", scanTime,
//			           "Duration between first and last measurement. Time between consecutive range measurement = scanTime / number of scans",
//			           Duration::milliseconds(0));
	}

	//@}

public:

	/**
	 * Returns pair of adjusted start and delta angle for this scan assuming the
	 * scan was taken at the given orientation. This is useful for lasers
	 * mounted upside down.
	 * @throw XInvalidParameter when laser is not mounted upright or upside down.
	 */
	//std::pair<SignedAnglef, SignedAnglef> getAnglesForOrientation(const Eigen::Quaternion<float>& orientation) const;

public:
	/// The starting orientation of the first sensor cone.
	SignedAnglef startAngle;

	/// The delta angle between neighboring cones.
	SignedAnglef deltaAngle;

	/// The resolution of a single cone.
	Anglef coneAngle;

	/// The aperture of the sensor in [m].
	float aperture;

	/// The measuring error of the sensor that delivered this scan.
	float stdError;

	/// The minimum range of valid measurements
	/// (measurements below this value result in BelowMinimum range code).
	float minimumRange;

	/// The maximum range of valid measurements
	/// (measurements above this value result in AboveMaximum range code).
	float maximumRange;

	/// Duration between first and last measurement.
	/// Time between consecutive range measurement = scanTime / number of scans
//	Duration scanTime;
};

///////////////////////////////////////////////////////////////////////////////

}}

#endif
