#ifndef DOT_H
#define DOT_H
#include "DQMath.h"
#include "VMath.h"
namespace MATHEX {

	// Ahh, our old friend the dot product. It still means the same thing in geometric algebra plus some extra superpowers
	// The consensus in the GA community is to overload the pipe "|" operator for the dot product, so I'll go with that
	// https://bivector.net/tools.html?p=3&q=0&r=1
	// WHY? https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Why_combine_things.pdf
	// More thoughts at the end of this file

	// A plane and a line dot to make another plane
	// This new plane is orthogonal to the original plane and through the line
	inline const MATH::Plane dot(const MATH::Plane& p, const DualQuat& line) {
		return (p * line).plane;
	}
	inline const MATH::Plane operator | (const MATH::Plane& p, const DualQuat& line) {
		return dot(p, line);
	}

	// A plane and a point dot to make a line
	// This new line is orthogonal to the original plane and through the point!
	inline const DualQuat dot(const MATH::Plane& p, const MATH::Vec4& v) {
		return p * MATH::VMath::inverse(v);	
	}
	inline const DualQuat operator | (const MATH::Plane& p, const MATH::Vec4& v) {
		return dot(p, v);
	}

	// A line and a point dot to make a plane
	// This new plane is orthogonal to the original line and through the point!
	inline const MATH::Plane dot (const DualQuat& line, const MATH::Vec4& v) {
		return (line * v).plane;
	}
	inline const MATH::Plane operator | (const DualQuat& line, const MATH::Vec4& v){
		return dot (line, v);
	}


	// Now we are dotting the same type of things together, it returns just a float
	inline float dot(const MATH::Plane& p1, const MATH::Plane& p2) {
		return (p1 * p2).real;
	}
	inline float operator | (const MATH::Plane& p1, const MATH::Plane& p2){
		return dot(p1, p2);
	}


	inline float dot(const DualQuat& line1, const DualQuat& line2) {
		return (line1 * line2).real;
	}
	inline float operator | (const DualQuat& line1, const DualQuat& line2){
		return dot(line1, line2);
	}


	inline float dot(const MATH::Vec4& v1, const MATH::Vec4& v2) {
		return (v1 * v2).real;
	}
	inline float operator | (const MATH::Vec4& v1, const MATH::Vec4& v2) {
		return dot(v1, v2);
	}
}
#endif

/// For example A dot B = |A|*|B|*cos(theta) which gives us information about the angle between two vectors
/// But now we can dot planes, lines, and points together too! This is going to be fun
/// Usually a dot product returns just a number right? Well, in GA its a bit different
/// If the two things you are dotting are the same type (plane, line, or point) you do get a scalar
/// But if the things are different, it reduces the highest grade by 1 so we sometimes end up with planes or lines
/// Amazingly these new planes and lines are orthogonal & through the original things we were dotting
