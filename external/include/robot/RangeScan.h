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
 * @file RangeScan.h
 *    RangeScan for representing range sensor scans.
 *
 * @author Tim Langner
 * @date   2010/08/19
 */

#ifndef _MIRA_RANGESCAN_H_
#define _MIRA_RANGESCAN_H_

//#include <serialization/adapters/std/vector>
#include <platform/Types.h>
#include <robot/RangeScanInfo.h>

namespace mira { namespace robot {

///////////////////////////////////////////////////////////////////////////////

/**
 * This class represents 2D range scans e.g. for laser, infrared or ultra-sonic
 * sensors. A laser would have several scans in the scan vector whereas the scan
 * vector of an ultra-sonic sensor would only contain a single scan.
 */
class RangeScan : public RangeScanInfo
{
public:
	/**
	 * Codes used for single scans to signal if they are valid
	 */
	enum RangeCode
	{
		Valid = 0,      ///< Scan is valid
		BelowMinimum,   ///< Scan is below the minimum scan range of the sensor
		AboveMaximum,   ///< Scan is above the maximum scan range of the sensor

		// Invalid is the first enum that indicates a measurement that really
		// is not usable. Other enums (like Masked) that also indicate an
		// unusable measurement must follow below.

		Invalid,        ///< Scan is invalid
		Masked,	        ///< Scan is masked out and therefore not used

		InvalidUser = 1000  ///< The first range code for device specific codes.
	};

public:
	/** @name Constructors and reflect */
	//@{

	RangeScan() {}

	/**
	 * Construct a new range scan object with a given number of scans.
	 *
	 * The vectors range and valid will be initialized with the given size,
	 * but the content of this vectors will be undefined!
	 * The vectors certainty and reflectance will be initialized with size 0.
	 *
	 * @param[in] scans The number of scans in the range scan.
	 */
	RangeScan(std::size_t scans) :
		range(scans), valid(scans) {}

	template<typename Reflector>
	void reflect(Reflector& r)
	{
		RangeScanInfo::reflect(r);

		r.member("Range", range, "The range scan vector. Values in meter");
		r.member("Valid", valid, "Valid range scans (value=0 for valid scans) "
		         "referenced to the elements in the range scan vector");
		r.member("Certainty", certainty, "Optional certainty values for each "
		         "scan referenced to the appropriate scan vector element");
		r.member("Reflectance", reflectance, "Optional reflectance values for "
		         "each scan referenced to the appropriate scan vector element");
	}

	//@}

public:

	/// The range scans. Values in meter.
	std::vector<float> range;

	/// Information if a scan is valid.
	/// Each value is referenced to the appropriate scan vector element.
	/// The elements should be taken from RangeCode for standard values.
	/// Device specific value can be used starting at RangeCode::InvalidUser.
	std::vector<uint16> valid;

	/// Optional certainty values for each scan referenced to the
	/// appropriate scan vector element
	std::vector<float> certainty;

	/// Optional reflectance values for each scan referenced to the
	/// appropriate scan vector element
	std::vector<float> reflectance;
};

///////////////////////////////////////////////////////////////////////////////

}}

#endif
