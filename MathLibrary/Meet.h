#ifndef MEET_H
#define MEET_H
#include "DualQuat.h"
#include "Plane.h"
#include "Vector.h"
namespace MATHEX {

	// The meet represents where two things "meet". Get it?
	// For example if you "meet" two planes, it gives you the line of intersection
	// It's written as a "^" in  math, so let's override that
	// WHY? https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Why_combine_things.pdf
	// More thoughts at the end of this file

	// A plane and a line meet at a point
	inline const MATH::Vec4 operator ^ (const MATH::Plane& p, const DualQuat& q) {
		MATH::Vec4 result;
		result.e032 = -p.e0 * q.e23 + p.e2 * q.e03 - p.e3 * q.e02;
		result.e013 = -p.e0 * q.e31 - p.e1 * q.e03 + p.e3 * q.e01;
		result.e021 = -p.e0 * q.e12 + p.e1 * q.e02 - p.e2 * q.e01;
		result.e123 = p.e1 * q.e23 + p.e2 * q.e31 + p.e3 * q.e12;
		return result;
	}

	// TODO: Check I can flip the operands this easily. I got this idea from the Klein library
	inline const MATH::Vec4 operator ^ (const DualQuat& q, const MATH::Plane& p) {
		return p ^ q;
	}

	// A plane and a plane meet at a line
	inline const DualQuat operator ^ (const MATH::Plane& p1, const MATH::Plane& p2)
	{
		DualQuat result;
		result.real = 0.0f;
		result.e23 = p1.e2 * p2.e3 - p1.e3 * p2.e2;
		result.e31 = p1.e3 * p2.e1 - p1.e1 * p2.e3;
		result.e12 = p1.e1 * p2.e2 - p1.e2 * p2.e1;
		result.e01 = p1.e0 * p2.e1 - p1.e1 * p2.e0;
		result.e02 = p1.e0 * p2.e2 - p1.e2 * p2.e0;
		result.e03 = p1.e0 * p2.e3 - p1.e3 * p2.e0;
		result.e0123 = 0.0f;
		return result;
	}

	// A point and a plane meet at a dual quaternion (only the e0123 part survives)
	// DERIVATION: https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Meet_point_with_plane.pdf
	inline const DualQuat operator ^ (const MATH::Vec4& v, const MATH::Plane& p)
	{
		DualQuat result;
		result.real = 0.0f;
		result.e23 = 0.0f;
		result.e31 = 0.0f;
		result.e12 = 0.0f;
		result.e01 = 0.0f;
		result.e02 = 0.0f;
		result.e03 = 0.0f;
		result.e0123 = -v.e032 * p.e1 - v.e013 * p.e2 - v.e021 * p.e3 - v.e123 * p.e0;
		return result;
	}

}
#endif

/// The "meet" is the cross product on steroids. Remember A x A = 0?
/// The meet (also known as the exterior product, the wedge, and the outer product) does the same thing
/// It's the part of the geometric product where there are no repeating terms in the e's
