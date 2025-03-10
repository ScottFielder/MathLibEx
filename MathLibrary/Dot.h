#ifndef DOT_H
#define DOT_H
#include <VMath.h>
#include "GeometricProduct.h"
namespace MATHEX {

	// Ahh, our old friend the dot product. It still means the same thing in geometric algebra plus some extra superpowers
	// The consensus in the GA community is to overload the pipe "|" operator for the dot product, so I'll go with that
	// I'll use the formula A dot B = <AB>_g, where g is the absolute difference between the grades of A and B
	// As shown in Hamish Todd's lecture around the 58 min mark: https://www.gdcvault.com/play/1029237/
	// WHY? https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Why_combine_things.pdf
	// More thoughts at the end of this file

	// A plane and a line dot to make another plane
	// This new plane is orthogonal to the original plane and through the line
	// TODO (UN): Do I need to code up the opposite order for the dot product? Does it matter?
	inline const Plane dot(const Plane& p, const DualQuat& line) {
		// Extract grade 1 part of the geometric product
		return (p * line).plane;
	}
	inline const Plane operator | (const Plane& p, const DualQuat& line) {
		return dot(p, line);
	}

	// A plane and a point dot to make a line
	// This new line is orthogonal to the original plane and through the point!
	// User might want to extract just the line part of the dual quaternion using DQMath::extractLine
	inline const DualQuat dot(const Plane& p, const MATH::Vec4& v) {
		return p * v;
	}
	inline const DualQuat operator | (const Plane& p, const MATH::Vec4& v) {
		return dot(p, v);
	}

	// A line and a point dot to make a plane
	// This new plane is orthogonal to the original line and through the point!
	inline const Plane dot (const DualQuat& line, const MATH::Vec4& v) {
		// Extract grade 1 part of the geometric product
		return (line * v).plane;
	}
	inline const Plane operator | (const DualQuat& line, const MATH::Vec4& v){
		return dot (line, v);
	}

	// Swap order of the dot product for point and line
	// The dot is commutative. Phew!
	inline const Plane dot(const MATH::Vec4& v, const DualQuat& line) {
		return dot(line, v);
	}
	inline const Plane operator | (const MATH::Vec4& v, const DualQuat& line) {
		return dot(line, v);
	}

	// Now we are dotting the same type of things together, it returns just a float
	// If the two arguments are normalized, returns cos(theta)
	inline float dot(const Plane& p1, const Plane& p2) {
		return (p1 * p2).real;
	}
	inline float operator | (const Plane& p1, const Plane& p2){
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
