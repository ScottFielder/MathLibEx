#ifndef GEOMETRICPRODUCT_H
#define GEOMETRICPRODUCT_H
#include "DualQuat.h"
#include "Flector.h"
namespace MATHEX {

	// Did your high school teacher say you cannot multiply points, lines, or planes together? Well, think again!
	// Welcome to the geometric product: AB = A ^ B + A dot B
	// WHY? https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Why_combine_things.pdf

	// I wrote this out on paper. My wrist is still hurting.
	// You don't need the brackets, it just helps my eyes
	// DERIVATION: https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Multiplying_dual_quaternions.pdf
	inline const DualQuat operator * (const DualQuat& a, const DualQuat& b) {
		MATHEX::DualQuat result;
		result.real = (a.real * b.real) - (a.e23 * b.e23) - (a.e31 * b.e31) - (a.e12 * b.e12);
		result.e23 = (a.real * b.e23) + (a.e23 * b.real) -  (a.e31 * b.e12) + (a.e12 * b.e31);
		result.e31 = (a.real * b.e31) + (a.e23 * b.e12) +   (a.e31 * b.real) - (a.e12 * b.e23);
		result.e12 = (a.real * b.e12) - (a.e23 * b.e31) +   (a.e31 * b.e23) + (a.e12 * b.real);
		result.e01 = (a.real * b.e01) - (a.e23 * b.e0123) - (a.e31 * b.e03) + (a.e12 * b.e02)
			+ (a.e01 * b.real) - (a.e02 * b.e12) + (a.e03 * b.e31) - (a.e0123 * b.e23);
		result.e02 = (a.real * b.e02) + (a.e23 * b.e03) - (a.e31 * b.e0123) - (a.e12 * b.e01)
			+ (a.e01 * b.e12) - (a.e03 * b.e23) + (a.e02 * b.real) - (a.e0123 * b.e31);
		result.e03 = (a.real * b.e03) - (a.e23 * b.e02) + (a.e31 * b.e01) - (a.e12 * b.e0123)
			- (a.e01 * b.e31) + (a.e02 * b.e23) + (a.e03 * b.real) - (a.e0123 * b.e12);
		result.e0123 = (a.real * b.e0123) + (a.e23 * b.e01) + (a.e31 * b.e02) + (a.e12 * b.e03)
			+ (a.e01 * b.e23) + (a.e02 * b.e31) + (a.e03 * b.e12) + (a.e0123 * b.real);
		return result;
	}
	
	// I'll overload the division operator as well
	// Such that A / B = A * inverse(B)		in that order
	inline const DualQuat operator / (const DualQuat& a, const DualQuat& b) {
		DualQuat inverseB = b;
		inverseB.e23 *= -1.0f;
		inverseB.e31 *= -1.0f;
		inverseB.e12 *= -1.0f;
		inverseB.e01 *= -1.0f;
		inverseB.e02 *= -1.0f;
		inverseB.e03 *= -1.0f;
		return a * inverseB;
	}

	// It's amazing that a plane and a point pops out when you multiply a dual quat with a Vec4 
	// That's why we need the Flectors
	// DERIVATION: https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Multiplying_dq_with_point.pdf
	inline const Flector operator * (const DualQuat& dq, const MATH::Vec4& p){
		Flector result;
		// UN - Oops, I missed a minus sign in the derivation. I've put it in below for that last term
		result.plane.e0 =  dq.e12 * p.e021 + dq.e31 * p.e013 + dq.e23 * p.e032 - dq.e0123 * p.e123;

		result.plane.e1 = -dq.e23 * p.e123;
		result.plane.e2 = -dq.e31 * p.e123;
		result.plane.e3 = -dq.e12 * p.e123;

		result.point.e032 =  dq.e12 * p.e013 - dq.e31 * p.e021 - dq.e01 * p.e123 + dq.real * p.e032;
		result.point.e013 = -dq.e12 * p.e032 + dq.e23 * p.e021 - dq.e02 * p.e123 + dq.real * p.e013;
		result.point.e021 =  dq.e31 * p.e032 - dq.e23 * p.e013 - dq.e03 * p.e123 + dq.real * p.e021;
		result.point.e123 = dq.real * p.e123;
		return result;
	}

	// A few signs flip when you multiply the other way round
	// DERIVATION: https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Multiplying_point_with_dq.pdf
	inline const Flector operator * (const MATH::Vec4& p, const DualQuat& q) {
		Flector result;
		result.plane.e0 = q.e12 * p.e021 + q.e31 * p.e013 + q.e23 * p.e032 + q.e0123 * p.e123;
		result.plane.e1 = -q.e23 * p.e123;
		result.plane.e2 = -q.e31 * p.e123;
		result.plane.e3 = -q.e12 * p.e123;

		result.point.e032 = -q.e12 * p.e013 + q.e31 * p.e021 + q.e01 * p.e123 + q.real * p.e032;
		result.point.e013 = q.e12 * p.e032 - q.e23 * p.e021 + q.e02 * p.e123 + q.real * p.e013;
		result.point.e021 = -q.e31 * p.e032 + q.e23 * p.e013 + q.e03 * p.e123 + q.real * p.e021;
		result.point.e123 = q.real * p.e123;
		return result;
	}

	// DERIVATION: https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Multiplying_plane_with_dq.pdf
	inline const Flector operator * (const Plane& p, const DualQuat& q) {
		Flector result;
		result.plane.e0 = p.e0 * q.real - p.e2 * q.e02 - p.e1 * q.e01 - p.e3 * q.e03;
		result.plane.e1 = p.e1 * q.real - p.e2 * q.e12 + p.e3 * q.e31;
		result.plane.e2 = p.e1 * q.e12 + p.e2 * q.real - p.e3 * q.e23;
		result.plane.e3 = p.e2 * q.e23 + p.e3 * q.real - p.e1 * q.e31;

		result.point.e032 = -p.e0 * q.e23 + p.e1 * q.e0123 + p.e2 * q.e03 - p.e3 * q.e02;
		result.point.e013 = -p.e0 * q.e31 - p.e1 * q.e03 + p.e2 * q.e0123 + p.e3 * q.e01;
		result.point.e021 = -p.e0 * q.e12 + p.e1 * q.e02 - p.e2 * q.e01 + p.e3 * q.e0123;
		result.point.e123 = p.e1 * q.e23 + p.e2 * q.e31 + p.e3 * q.e12;
		return result;
	}

	// We can reuse the Vec4 * DualQuat and Plane * DualQuat functions to implement the Flector * DualQuat
	inline const Flector operator * (const Flector& f, const DualQuat& dq) {
		Flector result = f.point * dq;
		result += f.plane * dq;
		return result;
	}

	// DERIVATION: https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Multiplying_plane_with_point.pdf
	inline const DualQuat operator * (const Plane& plane, const MATH::Vec4& point){
		DualQuat result;
		result.real = 0.0f;
		result.e23 = plane.e1 * point.e123;
		result.e31 = plane.e2 * point.e123;
		result.e12 = plane.e3 * point.e123;
		result.e01 = plane.e3 * point.e013 - plane.e2 * point.e021;
		result.e02 = plane.e1 * point.e021 - plane.e3 * point.e032;
		result.e03 = plane.e2 * point.e032 - plane.e1 * point.e013;
		result.e0123 = plane.e0 * point.e123 + plane.e1 * point.e032
			+ plane.e2 * point.e013 + plane.e3 * point.e021;
		return result;
	}

	// Plane 1 * Plane 2 = a dual quaternion
	// This one wasn't too bad to figure out on paper, as a plane only has e0, e1, e2, and e3 inside
	// After multiplying and keeping track of the e combos you end up with a dual quaternion
	// DERIVATION: https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Multiplying_planes.pdf
	inline const DualQuat operator * (const Plane& p1, const Plane& p2)
	{
		DualQuat result;
		result.real = p1.e1 * p2.e1 + p1.e2 * p2.e2 + p1.e3 * p2.e3;
		result.e23 = p1.e2 * p2.e3 - p1.e3 * p2.e2;
		result.e31 = p1.e3 * p2.e1 - p1.e1 * p2.e3;
		result.e12 = p1.e1 * p2.e2 - p1.e2 * p2.e1;
		result.e01 = p1.e0 * p2.e1 - p1.e1 * p2.e0;
		result.e02 = p1.e0 * p2.e2 - p1.e2 * p2.e0;
		result.e03 = p1.e0 * p2.e3 - p1.e3 * p2.e0;
		result.e0123 = 0.0f;
		return result;
	}

	// Turns out the inverse of a plane is the exact same plane, so A / B = A * B
	inline const DualQuat operator / (const Plane& a, const Plane& b) {
		return a * b;
	}

	// Point 1 * Point 2 = a dual quaternion
	// This one wasn't too bad to figure out on paper either, just end up with the infinite line 
	// and a term for w
	// DERIVATION: https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Multiplying_points.pdf
	inline const DualQuat operator * (const MATH::Vec4& p1, const MATH::Vec4& p2) {
		DualQuat result;
		result.real = -p1.e123 * p2.e123;
		result.e23 = 0.0f;
		result.e31 = 0.0f;
		result.e12 = 0.0f;
		result.e01 = p1.e032 * p2.e123 - p1.e123 * p2.e032;
		result.e02 = p1.e013 * p2.e123 - p1.e123 * p2.e013;
		result.e03 = p1.e021 * p2.e123 - p1.e123 * p2.e021;
		result.e0123 = 0.0f;
		return result;
	}

	// The inverse of a point [x, y, z, w] is [-x, -y, -z, -w]
	// So A / B = A * (-B)
	inline const DualQuat operator / (const MATH::Vec4& a, const MATH::Vec4& b) {
		return a * (-b);
	}
	// TODO (UN)
	// I'm guessing this one... Not tested yet
	inline const DualQuat operator * (const Flector& f, const MATH::Vec4& point) {
		return f.point * point + f.plane * point;
	}
}
#endif