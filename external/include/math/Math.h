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
 * @file Math.h
 *    Includes often needed math headers and methods and provides additional
 *    constants.
 *
 * @author Christian Martin
 * @date   2010/09/01
 */

#ifndef _MIRA_MATH_H_
#define _MIRA_MATH_H_

///////////////////////////////////////////////////////////////////////////////
#include <cmath>

// Include for boost::math::isnan, ...
#include <boost/math/special_functions/fpclassify.hpp>

// include round
#include <boost/math/special_functions/round.hpp>

// include constants
#include <boost/version.hpp>
#ifndef Q_MOC_RUN
#include <boost/math/constants/constants.hpp>
#endif

///@cond INTERNAL
namespace boost { namespace math { namespace constants {

// boost >= 1.50 need three arguments for BOOST_DEFINE_MATH_CONSTANT
#if (BOOST_VERSION >= 105000)
// pi/4
BOOST_DEFINE_MATH_CONSTANT(quarter_pi,     0.78539816339744830961566084581987572104929234984378,  "0.78539816339744830961566084581987572104929234984378")
// pi / 180
BOOST_DEFINE_MATH_CONSTANT(pi_div_deg180,  0.017453292519943295769236907684886127134428718885417, "0.017453292519943295769236907684886127134428718885417")
// 180 / pi
BOOST_DEFINE_MATH_CONSTANT(deg180_div_pi, 57.295779513082320876798154814105170332405472466564,    "57.295779513082320876798154814105170332405472466564")
#endif

// boost < 1.50 need four arguments for BOOST_DEFINE_MATH_CONSTANT
#if (BOOST_VERSION < 105000)
// pi/2
BOOST_DEFINE_MATH_CONSTANT(half_pi,        1.5707963267948966192313216916397514420985846996876,   0, 0)
// pi/4
BOOST_DEFINE_MATH_CONSTANT(quarter_pi,     0.78539816339744830961566084581987572104929234984378,  0, 0)
// pi / 180
BOOST_DEFINE_MATH_CONSTANT(pi_div_deg180,  0.017453292519943295769236907684886127134428718885417, 0, 0)
// 180 / pi
BOOST_DEFINE_MATH_CONSTANT(deg180_div_pi, 57.295779513082320876798154814105170332405472466564,    0, 0)
#endif

// for older boost versions define several useful constants that are part of boost >=1.46
#if (BOOST_VERSION < 104600)
BOOST_DEFINE_MATH_CONSTANT(two_pi,              6.2831853071795864769252867665590057683943388015061,  0, 0)
BOOST_DEFINE_MATH_CONSTANT(half_root_two,       0.70710678118654752440084436210484903928483593756084, 0, 0)
BOOST_DEFINE_MATH_CONSTANT(one_div_root_two,    0.70710678118654752440084436210484903928483593756084, 0, 0)
BOOST_DEFINE_MATH_CONSTANT(one_div_root_two_pi, 0.39894228040143267793994605993438186847585863095671, 0, 0)
#endif

}}}
///@endcond

// inject into our namespace
namespace mira {
	using boost::math::round;
	using namespace boost::math::constants;
}

///////////////////////////////////////////////////////////////////////////////

#endif
