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
 * @file Point.h
 *    Class for 2D, 3D and N-dimensional points.
 *
 * @author Jens Kessler, Tim Langner
 * @date   2010/08/11
 */

#ifndef _MIRA_POINT_H_
#define _MIRA_POINT_H_

#include <opencv/cxcore.h>

//#ifndef Q_MOC_RUN
//#include <boost/geometry/geometry.hpp>
//#endif

//#include <utils/PParam.h>
//#include <math/Eigen.h>
#include <Eigen/Eigen>

namespace mira {

///////////////////////////////////////////////////////////////////////////////
/**
 * @ingroup GeometryModule
 *
 * The base template class of point, which covers the basic functionality of
 * each point. Basic functionality is type conversion, some constructors and
 * operators. This class is a template which covers three parameters. The first
 * parameter T gives the data type of the point, the second D gives the number
 * of dimensions and the last parameter Derived represents the class which is
 * derived from PointBase. Normally this class is for internal use only and the
 * specialized point types should be sufficient. Anyway, it is not forbidden to
 * use this template...
 */
template <typename T, int D, typename Derived>
class PointBase : public Eigen::Matrix<T,D,1>
{
	typedef Eigen::Matrix<T,D,1> Base;

public:
	typedef T Type;
	enum { Dim = D};

public:
	/// Default constructor
	PointBase() {}

	/// Create from boost geometry point
//	PointBase(const boost::geometry::model::point<T,D,boost::geometry::cs::cartesian>& other) {
//		operator=(other);
//	}

	/// Create from Eigen matrix expression
	template <typename DerivedMatrix>
	PointBase(const Eigen::MatrixBase<DerivedMatrix>& other) : Base(other) {}

	/// assignment of Eigen matrix expression
	template <typename DerivedMatrix>
	Derived& operator=(const Eigen::MatrixBase<DerivedMatrix>& other) {
		Base::operator=(other);
		return *this;
	}

	/// converts from native boost::geometry::model::point
//	Derived& operator=(const boost::geometry::model::point<T,D,boost::geometry::cs::cartesian>& other)
//	{
//		for(int i=0; i < D; i++) // compiler will unroll loop
//			(*this)[i] = other(i);
//		return *this;
//	}

	/// converts to native boost::geometry::point
//	operator boost::geometry::model::point<T,D,boost::geometry::cs::cartesian>() const
//	{
//		boost::geometry::model::point<T,D,boost::geometry::cs::cartesian> r;
//		for(int i=0; i < D; i++) // compiler will unroll loop
//			r(i) = (*this)[i];
//		return r;
//	}
};

///////////////////////////////////////////////////////////////////////////////

/**
 * @ingroup GeometryModule
 *
 * General point class template. This class has two template parameters:
 * first (T) the data type of the point (like float, int) and the second, which
 * specifies the number of dimensions of the point.
 *
 * @note This class is specialized for 2D and 3D points.
 */
template <typename T, int D>
class Point : public PointBase<T,D, Point<T,D> >
{
	typedef PointBase<T,D, Point<T,D> > Base;

public:
	/// Default constructor
	Point() {}

	/// Creates Point from Eigen matrix
	template <typename DerivedMatrix>
	Point(const Eigen::MatrixBase<DerivedMatrix>& other) : Base(other) {}

	/// Creates Point from boost::geometry::point
//	Point(const boost::geometry::model::point<T,D,boost::geometry::cs::cartesian>& other) {
//		operator=(other);
//	}
};

///////////////////////////////////////////////////////////////////////////////

///@cond INTERNAL
//template <typename T, int D>
//class IsTransparentSerializable<Point<T,D>> : public std::true_type {};
///@endcond

///////////////////////////////////////////////////////////////////////////////

/**
 * @ingroup GeometryModule
 *
 * Specialization of Point for 2 dimensions with specialized constructors and
 * converters.
 */
template <typename T>
class Point<T,2> : public PointBase<T,2, Point<T,2> >
{
	typedef PointBase<T,2, Point<T,2> > Base;

public:
	/// Default-constructor.
	Point() {}

	/// Creates point from its two coordinates.
	Point(T x, T y)
	{
		(*this)[0] = x;
		(*this)[1] = y;
	}

	/// Creates Point from Eigen matrix
	template <typename DerivedMatrix>
	Point(const Eigen::MatrixBase<DerivedMatrix>& other) : Base(other) {}

	/// Creates Point from boost::geometry::point
//	Point(const boost::geometry::model::point<T,2,boost::geometry::cs::cartesian>& other) {
//		operator=(other);
//	}

	/// Creates Point from OpenCV point
	explicit Point<T,2>(const cv::Point_<T> & other) {
		operator=(other);
	}

	/// converts from OpenCV point
	Point& operator=(const cv::Point_<T>& other) {
		(*this)[0] = other.x;
		(*this)[1] = other.y;
		return *this;
	}

	/// converts to OpenCV point
	operator cv::Point_<T>() const
	{
		return cv::Point_<T>( this->x(), this->y() );
	}

	/// the reflect method for 2D point serialization
	template<typename Reflector>
	void reflect(Reflector& reflector)
	{
		reflector.property("X", this->x(), "The x-coordinate");
		reflector.property("Y", this->y(), "The y-coordinate");
	}
};

///////////////////////////////////////////////////////////////////////////////

///@cond INTERNAL
//template <typename T>
//class IsTransparentSerializable<Point<T,2>> : public std::false_type {};
///@endcond

///////////////////////////////////////////////////////////////////////////////

typedef Point<int,2> Point2i; ///< a 2D integer point
typedef Point<float,2> Point2f; ///< a 2D 32 bit floating precision point
typedef Point<double,2> Point2d; ///< a 2D 64 bit floating precision point

///////////////////////////////////////////////////////////////////////////////
/**
 * @ingroup GeometryModule
 *
 * Specialization of Point for 3 dimensions with specialized constructors and
 * converters.
 */
template <typename T>
class Point<T,3> : public PointBase<T,3, Point<T,3> >
{
	typedef PointBase<T,3, Point<T,3> > Base;

public:
	/// Default-constructor.
	Point(){}

	/// Creates point from its three coordinates.
	Point(T x, T y, T z)
	{
		(*this)[0] = x;
		(*this)[1] = y;
		(*this)[2] = z;
	}

	/// Creates Point from Eigen matrix
	template <typename DerivedMatrix>
	Point(const Eigen::MatrixBase<DerivedMatrix>& other) : Base(other) {}

	/// Creates Point from boost::geometry::point
//	Point(const boost::geometry::model::point<T,3,boost::geometry::cs::cartesian>& other) {
//		operator=(other);
//	}

	/// Creates Point from OpenCV point
	explicit Point(const cv::Point3_<T> & other) {
		operator=(other);
	}

	/// converts from OpenCV point
	Point& operator=(const cv::Point3_<T>& other) {
		(*this)[0] = other.x;
		(*this)[1] = other.y;
		(*this)[2] = other.z;
		return *this;
	}

	/// converts to OpenCV point
	operator cv::Point3_<T>() const
	{
		return cv::Point3_<T>(this->x(),this->y(),this->z());
	}

	/// the reflect method for 3D point serialization
	template<typename Reflector>
	void reflect(Reflector& reflector)
	{
		reflector.property("X", this->x(), "The x-coordinate");
		reflector.property("Y", this->y(), "The y-coordinate");
		reflector.property("Z", this->z(), "The z-coordinate");
	}

};

//////////////////////////////////////////////////////////////////////////////

///@cond INTERNAL
//template <typename T>
//class IsTransparentSerializable<Point<T,3>> : public std::false_type {};
///@endcond

//////////////////////////////////////////////////////////////////////////////

typedef Point<int,3> Point3i;    ///< a 3D integer point
typedef Point<float,3> Point3f;  ///< a 3D 32 bit floating precision point
typedef Point<double,3> Point3d; ///< a 3D 64 bit floating precision point

//////////////////////////////////////////////////////////////////////////////

} // namespace MIRA

///@cond INTERNAL
// BOOST specialization on mira::point to accept them as geometry entity
//namespace boost { namespace geometry { namespace traits {
//
////////////////////////////////////////////////////////////////////////////////
//
//template<typename T, int D>
//struct tag<mira::Point<T, D> >
//{ typedef point_tag type; };
//
//template<typename T, int D>
//struct dimension<mira::Point<T, D> > : boost::mpl::int_<D>
//{};
//
//template<typename T, int D>
//struct coordinate_type<mira::Point<T, D> >
//{ typedef T type; };
//
//template<typename T, int D>
//struct coordinate_system<mira::Point<T, D> >
//{ typedef cs::cartesian type; };
//
//template<typename T, int D, std::size_t Dimension>
//struct access<mira::Point<T, D>, Dimension>
//{
//	static inline T get(mira::Point<T, D> const& p)
//	{
//		return p(Dimension, 0);
//	}
//	static inline void set(mira::Point<T, D>& p, T const& value)
//	{
//		p(Dimension, 0) = value;
//	}
//};
//
////////////////////////////////////////////////////////////////////////////
//
//}}} // namespace boost { namespace geometry { namespace traits {

///@endcond

//////////////////////////////////////////////////////////////////////////////

#endif
