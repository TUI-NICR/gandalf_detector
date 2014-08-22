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
 * @file Angle.h
 * Implementations of angles (values in periodic interval of width 2*pi)
 * with arbitrary base type.
 *
 * @author Erik Einhorn
 * @date   2010/08/02
 */

#include <type_traits>

#include <platform/Platform.h> // for MIRA_DEPRECATED
#include <math/Math.h>

//#include <serialization/GetterSetter.h>


#ifndef _MIRA_ANGLE_H_
#define _MIRA_ANGLE_H_

namespace mira {
///////////////////////////////////////////////////////////////////////////////

/// Convert degree to radian.
template <typename T>
inline T deg2rad(T value) {
	static_assert(std::is_floating_point<T>::value,
	              "deg2rad must be used with floating point types");
	return static_cast<T>(value * pi_div_deg180<T>());
}

/// Convert radian to degree.
template <typename T>
inline T rad2deg(T value) {
	static_assert(std::is_floating_point<T>::value,
	              "rad2deg must be used with floating point types");
	return static_cast<T>(value * deg180_div_pi<T>());
}

///////////////////////////////////////////////////////////////////////////////

///@cond INTERNAL
namespace Private {
/// Tag for marking an angle representation that uses degrees (internal use).
struct DegreeUnitTag {
	/// returns the unit string
	static const char* unit() { return "deg"; }
};
/// Tag for marking an angle representation that uses radians (internal use).
struct RadianUnitTag {
	/// returns the unit string
	static const char* unit() { return "rad"; }
};

// helper for converting angles from FromTag to ToTag, e.g. DegreeUnitTag to RadianUnitTag.
template <typename T, typename FromTag, typename ToTag>
struct AngleConverter {
	static_assert(sizeof(FromTag)==0, "Angle conversion not specialized!");
	static T convert(T value) { return static_cast<T>(0); }
};


// specializations for the actual conversions
template <typename T>
struct AngleConverter<T,DegreeUnitTag,RadianUnitTag> {
	static T convert(T value) { return deg2rad(value); }
};

template <typename T>
struct AngleConverter<T,RadianUnitTag,DegreeUnitTag> {
	static T convert(T value) { return rad2deg(value); }
};

template <typename T,typename SameTag>
struct AngleConverter<T,SameTag,SameTag> {
	static T convert(T value) { return value; }
};

/**
 * Helper that yields the most generic type of A and B,
 * e.g. AnglePromoteHelper<float,int>::type will yield float
 */
template <typename A, typename B>
struct AnglePromoteHelper {
	typedef decltype(A(0) + B(0)) type;
};

}
///@endcond INTERNAL

/**
 * Base class template for derived Angle implementations.
 * Contains major parts of the implementation of the angle classes.
 * Uses curiously recurring template pattern (CRTP) for static polymorphism.
 */
template <typename T, typename UnitTag, typename Derived>
class AngleBase
{
protected:

	// direct fast initialization without normalization
	AngleBase(T value, int) : mValue(value) {}

	AngleBase() {}
	explicit AngleBase(T value)  { setValue(value); }
	AngleBase(const AngleBase& other) : mValue(other.mValue) {}

	template <typename OtherT, typename OtherUnitTag, typename OtherDerived>
	explicit AngleBase(const AngleBase<OtherT,OtherUnitTag,OtherDerived>& other) {
		setValue(conv(other));
	}

	// generated copy constructor is suitable

	// assignment operator implementations that are used by derived classes
	template <typename OtherUnitTag, typename OtherDerived>
	void operator=(const AngleBase<T,OtherUnitTag,OtherDerived>& other) {
		mValue = conv(other);
	}

	void operator=(const T& other) {
		setValue(other);
	}

public:

	template <typename Reflector>
	void reflect(Reflector& r) {
		r.delegate(mValue);
	}

public:

	/**
	 * Returns the upper limit of the defined angle interval.
	 * Note, that the upper limit does not belong to the interval itself, hence
	 * the interval is [lower,upper) .
	 */
	static T upper() { return Derived::lower() + Derived::turn(); }


public:

	/// Set the angle value. The input value is mapped into the angle interval.
	void setValue(const T& value)
	{
		mValue = value;
		// this is faster than using modulo operations:
		while(mValue >= Derived::upper())
			mValue -= Derived::turn();
		while(mValue < Derived::lower())
			mValue += Derived::turn();
	}

	/// Returns the raw angle value given in the native unit of the angle class.
	const T& value() const { return mValue; }

	/// Cast operator to native type (same as value() )
	operator T() const { return mValue; }

	/**
	 * Sets the value in the unit that is used for serialization.
	 * The Angle and SignedAngle classes use radians for storing the value
	 * internal and degrees for serializing the value to simplify the usage
	 * for the human user. For the other classes Degree, SignedDegree,
	 * Radian and SignedRadian setValue() and setSerializedValue() are
	 * doing the same.
	 */
	void setSerializedValue(const T& value) {
		setValue(Derived::convertFromSerialized(value));
	}

	/**
	 * Returns the value in the unit that is used for serialization.
	 * The Angle and SignedAngle classes use radians for storing the value
	 * internal and degrees for serializing the value to simplify the usage
	 * for the human user. For the other classes Degree, SignedDegree,
	 * Radian and SignedRadian value() and serializedValue() yield the same
	 * result.
	 */
	T serializedValue() const { return Derived::convertToSerialized(mValue); }


public:

	/// Returns the value of the angle in degrees.
	T deg() const { return This()->toDeg(); }

	/// Returns the value of the angle in radian.
	T rad() const {	return This()->toRad();}

public:
	// operators

	// note the generic operators take Derived& as first argument, while
	// the operators, where T& is a parameter take AngleBase& as argument.
	// this is necessary to resolve the ISO C++ ambiguity with type conversation

	/// Add two angles
	template <typename OtherUnitTag, typename OtherDerived>
	friend Derived operator+(const Derived& a, const AngleBase<T,OtherUnitTag,OtherDerived>&  b) {
		return Derived( a.mValue + conv(b) );
	}
	/// Add two angles
	friend Derived operator+(const AngleBase& a, const T& b) {
		return Derived( a.mValue + b);
	}
	/// Add two angles
	friend Derived operator+(const T& a, const AngleBase& b) {
		return Derived( a + b.mValue);
	}

	/// Subtract two angles
	template <typename OtherUnitTag, typename OtherDerived>
	friend Derived operator-(const Derived& a,const AngleBase<T,OtherUnitTag,OtherDerived>&  b) {
		return Derived( a.mValue - conv(b) );
	}
	/// Subtract two angles
	friend Derived operator-(const AngleBase& a, const T& b) {
		return Derived( a.mValue - b);
	}
	/// Subtract two angles
	friend Derived operator-(const T& a, const AngleBase& b) {
		return Derived( a - b.mValue);
	}

	/// Unary minus operator
	Derived operator-() const {
		return Derived(-mValue);
	}

	/// Multiply with scalar
	friend Derived operator*(const Derived& a, const T& b) {
		return Derived( a.mValue * b);
	}
	/// Multiply with scalar
	friend Derived operator*(const T& a, const Derived& b) {
		return Derived( a * b.mValue);
	}

	/// Divide by scalar
	friend Derived operator/(const Derived& a, const T& b) {
		return Derived( a.mValue / b);
	}


	/// Add other angle to this angle
	template <typename OtherUnitTag, typename OtherDerived>
	Derived operator+=(const AngleBase<T,OtherUnitTag,OtherDerived>& a) {
		setValue(mValue+conv(a));
		return *This();
	}
	/// Add float value to this angle
	Derived& operator+=(const T& a) {
		setValue(mValue+a);
		return *This();
	}

	/// Subtract other angle from this angle
	template <typename OtherUnitTag, typename OtherDerived>
	Derived operator-=(const AngleBase<T,OtherUnitTag,OtherDerived>& a) {
		setValue(mValue-conv(a));
		return *This();
	}
	/// Subtract float value from this angle
	Derived& operator-=(const T& a) {
		setValue(mValue-a);
		return *This();
	}

	/// Multiply this angle with scalar
	Derived& operator*=(const T& s) {
		setValue(mValue*s);
		return *This();
	}

	/// Divide this angle by scalar
	Derived& operator/=(const T& s) {
		setValue(mValue/s);
		return *This();
	}


	friend bool operator<(const AngleBase& a, const AngleBase& b) { return a.mValue < b.mValue; }
	friend bool operator<=(const AngleBase& a, const AngleBase& b) { return a.mValue <= b.mValue; }
	friend bool operator>(const AngleBase& a, const AngleBase& b) { return a.mValue > b.mValue; }
	friend bool operator>=(const AngleBase& a, const AngleBase& b) { return a.mValue >= b.mValue; }
	friend bool operator==(const AngleBase& a, const AngleBase& b) { return a.mValue == b.mValue; }
	friend bool operator!=(const AngleBase& a, const AngleBase& b) { return a.mValue != b.mValue; }


public:

	/**
	 * Returns the signed difference angle between this angle and the
	 * specified other angle that has the smallest absolute value.
	 * This method is similar to smallestDifference() but returns a floating
	 * point value instead of a signed angle class object.
	 */
	T smallestDifferenceValue(const Derived& other) const
	{
		T d = mValue - other.mValue;

		static const T halfTurn = Derived::turn() / 2;
		if(d > halfTurn)
			d -= Derived::turn();
		else if(d <= -halfTurn)
			d += Derived::turn();

		return d;
	}

	/**
	 * Returns true, if the angle is in the given interval [min,max].
	 * If max<min, both values will be swapped.
	 */
	bool isInInterval(const Derived& min, const Derived& max) const
	{
		if (min.mValue <= max.mValue)
			return mValue >= min.mValue && mValue <= max.mValue;
		return mValue >= min.mValue || mValue <= max.mValue;
	}

public:

	/// stream operator
	friend std::ostream& operator<<(std::ostream& o, const AngleBase& v ) {
		o << v.value() << " " << UnitTag::unit();
		return o;
	}

public:

	/**
	 * Returns the unit of this angle representation as string, e.g.
	 * "rad" or "deg"
	 */
	static const char* unit() {
		return UnitTag::unit();
	}

private:

	/// Converts other angle representation into this angle representation
	template <typename OtherT, typename OtherUnitTag, typename OtherDerived>
	static T conv(const AngleBase<OtherT,OtherUnitTag,OtherDerived>& other) {
		typedef typename Private::AnglePromoteHelper<T,OtherT>::type type;
		return Private::AngleConverter<type,OtherUnitTag,UnitTag>::convert(other.value());
	}


	/// Return <EM>this</EM> pointer, statically casted to correct type (making use of CRTP)
	Derived* This() { return static_cast<Derived*>(this); }

	/// Return const <EM>this</EM> pointer, statically casted to correct type (making use of CRTP)
	const Derived* This() const { return static_cast<const Derived*>(this); }

protected:

	/// the actual value
	T mValue;
};


#define MIRA_ANGLE_CONSTRUCTORS_AND_ASSIGNOPS(Type)                            \
protected:                                                                     \
	Type(const T& value, int) : Base(value,1) {}                               \
public:                                                                        \
	Type() {}                                                                  \
	explicit Type(T value) : Base(value) {}                                    \
	template <typename OtherT, typename OtherUnitTag, typename OtherDerived>   \
	Type(const AngleBase<OtherT,OtherUnitTag,OtherDerived>& other) : Base(other) {} \
	template <typename OtherUnitTag, typename OtherDerived>                    \
	Type& operator=(const AngleBase<T,OtherUnitTag,OtherDerived>& other) {     \
		Base::operator=(other);                                                \
		return *this; }                                                        \
	Type& operator=(const T& other) {                                          \
		Base::operator=(other);                                                \
		return *this; }


///////////////////////////////////////////////////////////////////////////////

/**
 * Base class for angle classes that represent angles using degrees.
 */
template <typename T, typename Derived>
class DegreeBase : public AngleBase<T, Private::DegreeUnitTag, Derived>
{
	friend class AngleBase<T, Private::DegreeUnitTag, Derived>;
	typedef AngleBase<T, Private::DegreeUnitTag, Derived> Base;
public:
	MIRA_ANGLE_CONSTRUCTORS_AND_ASSIGNOPS(DegreeBase)

public:
	/// Returns the amount of a turn (full circle) that is equal to 360 deg.
	static T turn() { return (T)360; }
	
	/// Converts own representation to serialized representation.
	static T convertToSerialized(T value) { return value; }

	/// Converts serialized representation to own representation.
	static T convertFromSerialized(T value) { return value; }

private:
	/// For internal use only: returns the value of the angle in degrees.
	T toDeg() const { return this->mValue; }

	/// For internal use only: returns the value of the angle in radian.
	T toRad() const { return deg2rad(this->mValue); }
};

/**
 * Base class for angle classes that represent angles using radians.
 */
template <typename T, typename Derived>
class RadianBase : public AngleBase<T, Private::RadianUnitTag, Derived>
{
	static_assert(std::is_floating_point<T>::value,
			"Radians make sense with floating point types only. "
			"Use Degree if you want to represent angles with integral types");

	friend class AngleBase<T, Private::RadianUnitTag, Derived>;
	typedef AngleBase<T, Private::RadianUnitTag, Derived> Base;

public:
	MIRA_ANGLE_CONSTRUCTORS_AND_ASSIGNOPS(RadianBase)

public:
	/// Returns the amount of a turn (full circle) that is equal to 360 deg.
	static T turn() { return two_pi<T>(); }

	/// Converts own representation to serialized representation.
	static T convertToSerialized(T value) { return value; }

	/// Converts serialized representation to own representation.
	static T convertFromSerialized(T value) { return value; }

private:
	/// For internal use only: returns the value of the angle in degrees.
	T toDeg() const { return rad2deg(this->mValue); }

	/// For internal use only: returns the value of the angle in radian.
	T toRad() const { return this->mValue; }

};

///////////////////////////////////////////////////////////////////////////////

// forward decl
template <typename T>
class Degree;

/**
 * @ingroup MathModule
 * Signed angle that is represented using degrees.
 * The values are normalized to the interval [-180,180). The class will take
 * care that the values stay within this range, when operations and computations
 * are performed.
 *
 * The values are interchangeable with other angle types,
 * conversions will be done automatically between them.
 */
template <typename T>
class SignedDegree : public DegreeBase<T, SignedDegree<T>>
{
	typedef DegreeBase<T, SignedDegree<T>> Base;
	friend class Degree<T>;
public:
	MIRA_ANGLE_CONSTRUCTORS_AND_ASSIGNOPS(SignedDegree)

	/**
	 * Returns the lower limit of the defined angle interval.
	 * Note, that the lower limit is included in the interval, hence
	 * the interval is [lower,upper) .
	 */
	static T lower() { return static_cast<T>(-180); }

	/**
	 * Returns the signed difference angle between this angle and the
	 * specified other angle that has the smallest absolute value.
	 */
	SignedDegree<T> smallestDifference(const SignedDegree& other) const {
		return SignedDegree<T>(this->smallestDifferenceValue(other), 1);
	}
};

//template <typename T>
//class IsTransparentSerializable<SignedDegree<T>> : public std::true_type {};


/**
 * @ingroup MathModule
 * Unsigned angle that is represented using degrees.
 * The values are normalized to the interval [0,360). The class will take care
 * that the values stay within this range, when operations and computations
 * are performed.
 *
 * The values are interchangeable with other angle types,
 * conversions will be done automatically between them.
 */
template <typename T>
class Degree : public DegreeBase<T, Degree<T>>
{
	typedef DegreeBase<T, Degree<T>> Base;
public:
	MIRA_ANGLE_CONSTRUCTORS_AND_ASSIGNOPS(Degree)

	/**
	 * Returns the lower limit of the defined angle interval.
	 * Note, that the lower limit is included in the interval, hence
	 * the interval is [lower,upper) .
	 */
	static T lower() { return static_cast<T>(0); }

	/**
	 * Returns the signed difference angle between this angle and the
	 * specified other angle that has the smallest absolute value.
	 */
	SignedDegree<T> smallestDifference(const Degree& other) const {
		return SignedDegree<T>(this->smallestDifferenceValue(other), 1);
	}
};

//template <typename T>
//class IsTransparentSerializable<Degree<T>> : public std::true_type {};

///////////////////////////////////////////////////////////////////////////////

// forward decl
template <typename T>
class Radian;

/**
 * @ingroup MathModule
 * Signed angle that is represented using radians.
 * The values are normalized to the interval [-pi,pi). The class will take care
 * that the values stay within this range, when operations and computations
 * are performed.
 *
 * The values are interchangeable with other angle types,
 * conversions will be done automatically between them.
 */
template <typename T>
class SignedRadian : public RadianBase<T, SignedRadian<T>>
{
	typedef RadianBase<T, SignedRadian<T>> Base;
	friend class Radian<T>;
public:
	MIRA_ANGLE_CONSTRUCTORS_AND_ASSIGNOPS(SignedRadian)

	/**
	 * Returns the lower limit of the defined angle interval.
	 * Note, that the lower limit is included in the interval, hence
	 * the interval is [lower,upper) .
	 */
	static T lower() { return -pi<T>(); }

	/**
	 * Returns the signed difference angle between this angle and the
	 * specified other angle that has the smallest absolute value.
	 */
	SignedRadian<T> smallestDifference(const SignedRadian& other) const {
		return SignedRadian<T>(this->smallestDifferenceValue(other), 1);
	}
};

//template <typename T>
//class IsTransparentSerializable<SignedRadian<T>> : public std::true_type {};

/**
 * @ingroup MathModule
 * Unsigned angle that is represented using radians.
 * The values are normalized to the interval [0,2pi). The class will take care
 * that values stay within this range, when operations and computations
 * are performed.
 *
 * The values are interchangeable with other angle types,
 * conversions will be done automatically between them.
 */
template <typename T>
class Radian : public RadianBase<T, Radian<T>>
{
	typedef RadianBase<T, Radian<T>> Base;
public:
	MIRA_ANGLE_CONSTRUCTORS_AND_ASSIGNOPS(Radian)

	/**
	 * Returns the lower limit of the defined angle interval.
	 * Note, that the lower limit is included in the interval, hence
	 * the interval is [lower,upper) .
	 */
	static T lower() { return static_cast<T>(0); }

	/**
	 * Returns the signed difference angle between this angle and the
	 * specified other angle that has the smallest absolute value.
	 */
	SignedRadian<T> smallestDifference(const Radian& other) const {
		return SignedRadian<T>(this->smallestDifferenceValue(other), 1);
	}
};

//template <typename T>
//class IsTransparentSerializable<Radian<T>> : public std::true_type {};

///////////////////////////////////////////////////////////////////////////////

// forward decl
template <typename T>
class Angle;

/**
 * @ingroup MathModule
 * Signed angle that is represented using radians.
 * The values are normalized to the interval [-pi,pi). The class will take care
 * that the values stay within this range, when operations and computations
 * are performed.
 *
 * This class is essentially the same as SignedRadian, except that
 * the angles are serialized using degrees.
 *
 * The values are interchangeable with other angle types,
 * conversions will be done automatically between them.
 */
template <typename T>
class SignedAngle : public RadianBase<T, SignedAngle<T>>
{
	typedef RadianBase<T, SignedAngle<T>> Base;
	friend class Angle<T>;
public:
	MIRA_ANGLE_CONSTRUCTORS_AND_ASSIGNOPS(SignedAngle)

//	template <typename Reflector>
//	void reflect(Reflector& r) {
//		r.delegate(getter<T>(convertToSerialized, this->mValue),
//		           setter<T>(convertFromSerialized, this->mValue));
//	}

	/**
	 * Returns the lower limit of the defined angle interval.
	 * Note, that the lower limit is included in the interval, hence
	 * the interval is [lower,upper) .
	 */
	static T lower() { return -pi<T>(); }

	/**
	 * Returns the signed difference angle between this angle and the
	 * specified other angle that has the smallest absolute value.
	 */
	SignedAngle<T> smallestDifference(const SignedAngle& other) const {
		return SignedAngle<T>(this->smallestDifferenceValue(other), 1);
	}


	/// deprecated, use Radian<T>(value) instead
	MIRA_DEPRECATED("use Radian<T>(value)", static SignedAngle fromRad(T value)) {
		return SignedAngle(Radian<T>(value));
	}
	/// deprecated, use Degree<T>(value) instead
	MIRA_DEPRECATED("use Degree<T>(value)", static SignedAngle fromDeg(T value)) {
		return SignedAngle(Degree<T>(value));
	}

public:
	// overwite convertXYZSerialized since Angle uses different internal and serialized representation.
	/// Converts own representation to serialized representation.
	static T convertToSerialized(T value) { return rad2deg<T>(value); }

	/// Converts serialized representation to own representation.
	static T convertFromSerialized(T value) { return deg2rad<T>(value); }
};

//template <typename T>
//class IsTransparentSerializable<SignedAngle<T>> : public std::true_type {};


/**
 * @ingroup MathModule
 * Unsigned angle that is represented using radians.
 * The values are normalized to the interval [0,2pi). The class will take care
 * that the values stay within this range, when operations and computations
 * are performed.
 *
 * This class is essentially the same as Radian, except that
 * the angles are serialized using degrees.
 *
 * The values are interchangeable with other angle types,
 * conversions will be done automatically between them.
 */
template <typename T>
class Angle : public RadianBase<T, Angle<T>>
{
	typedef RadianBase<T, Angle<T>> Base;
public:
	MIRA_ANGLE_CONSTRUCTORS_AND_ASSIGNOPS(Angle)

//	template <typename Reflector>
//	void reflect(Reflector& r) {
//		r.delegate(getter<T>(convertToSerialized, this->mValue),
//		           setter<T>(convertFromSerialized, this->mValue));
//	}

	/**
	 * Returns the lower limit of the defined angle interval.
	 * Note, that the lower limit is included in the interval, hence
	 * the interval is [lower,upper) .
	 */
	static T lower() { return static_cast<T>(0); }

	/**
	 * Returns the signed difference angle between this angle and the
	 * specified other angle that has the smallest absolute value.
	 */
	SignedAngle<T> smallestDifference(const Angle& other) const {
		return SignedAngle<T>(this->smallestDifferenceValue(other), 1);
	}

	/// deprecated, use Radian<T>(value) instead
	MIRA_DEPRECATED("use Radian<T>(value)", static Angle fromRad(T value)) {
		return Angle(Radian<T>(value));
	}
	/// deprecated, use Degree<T>(value) instead
	MIRA_DEPRECATED("use Degree<T>(value)", static Angle fromDeg(T value)) {
		return Angle(Degree<T>(value));
	}

public:
	// overwite convertXYZSerialized since Angle uses different internal and serialized representation.
	/// Converts own representation to serialized representation.
	static T convertToSerialized(T value) { return rad2deg<T>(value); }

	/// Converts serialized representation to own representation.
	static T convertFromSerialized(T value) { return deg2rad<T>(value); }
};

//template <typename T>
//class IsTransparentSerializable<Angle<T>> : public std::true_type {};


///////////////////////////////////////////////////////////////////////////////

/// Integer precision signed angle. @see SignedDegree @ingroup MathModule
typedef SignedDegree<int>    SignedDegreei;
/// Float precision signed angle. @see SignedDegree @ingroup MathModule
typedef SignedDegree<float>  SignedDegreef;
/// Double precision signed angle. @see SignedDegree @ingroup MathModule
typedef SignedDegree<double> SignedDegreed;
/// Integer precision angle. @see Degree @ingroup MathModule
typedef Degree<int>    Degreei;
/// Float precision angle. @see Degree @ingroup MathModule
typedef Degree<float>  Degreef;
/// Double precision angle. @see Degree @ingroup MathModule
typedef Degree<double> Degreed;

/// Float precision signed angle. @see SignedRadian @ingroup MathModule
typedef SignedRadian<float>  SignedRadianf;
/// Double precision signed angle. @see SignedRadian @ingroup MathModule
typedef SignedRadian<double> SignedRadiand;
/// Float precision angle. @see Radian @ingroup MathModule
typedef Radian<float>  Radianf;
/// Double precision angle. @see Radian @ingroup MathModule
typedef Radian<double> Radiand;

/// Float precision signed angle. @see SignedAngle @ingroup MathModule
typedef SignedAngle<float>  SignedAnglef;
/// Double precision signed angle. @see SignedAngle @ingroup MathModule
typedef SignedAngle<double> SignedAngled;
/// Float precision angle. @see Angle @ingroup MathModule
typedef Angle<float>  Anglef;
/// Double precision angle. @see Angle @ingroup MathModule
typedef Angle<double> Angled;

//////////////////////////////////////////////////////////////////////////////

/**
 * @ingroup MathModule
 * Returns the signed difference angle between the specified angles (in radian)
 * that has the smallest absolute value.
 */
template<typename T>
inline T smallestAngleDifference(const T& a, const T& b)
{
	return SignedAngle<T>(a).smallestDifference(SignedAngle<T>(b)).value();
}

/// @deprecated, use isInAngleInterval()
template<typename T>
inline bool inAngleInterval(T value, T min, T max)
{
	return SignedAngle<T>(value).isInInterval(SignedAngle<T>(min),
	                                          SignedAngle<T>(max));
}

/// Returns true, if the given angle (in radian) is in the given interval [min,max] @ingroup MathModule
template<typename T>
inline bool isInAngleInterval(T value, T min, T max)
{
	return SignedAngle<T>(value).isInInterval(SignedAngle<T>(min),
	                                          SignedAngle<T>(max));
}
//////////////////////////////////////////////////////////////////////////////

/// Getter for serializing radians as degrees. @ingroup MathModule
template <typename T>
inline T rad2degGetter(const T& in)
{
	return rad2deg<T>(in);
}

/// Setter for serializing radians as degrees. @ingroup MathModule
template <typename T>
inline void deg2radSetter(T& out, const T& in)
{
	out = deg2rad<T>(in);
}

///////////////////////////////////////////////////////////////////////////////

}

//////////////////////////////////////////////////////////////////////////////

// specialization of std::abs for all Angle types
namespace std {
template <typename T, typename UnitTag, typename Derived>
inline Derived abs(const mira::AngleBase<T,UnitTag,Derived>& other) {
	return Derived(std::abs(other.value()));
}
}

///////////////////////////////////////////////////////////////////////////////

#endif
