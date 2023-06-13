#ifndef POINCAREDUALITY_H
#define POINCAREDUALITY_H
#include "DualQuat.h"
#include "Plane.h"
#include "Vector.h"
namespace MATHEX {

	// The Poincare duality returns whatever e basis vectors are missing
	// So basically whatever it is not. The not operator ! seems like a good option here
	inline const DualQuat operator ! (DualQuat dq) {
		DualQuat result;
		result.real = dq.e0123;
		result.e23 = dq.e01;
		result.e31 = dq.e02;
		result.e12 = dq.e03;
		result.e01 = dq.e23;
		result.e02 = dq.e31;
		result.e03 = dq.e12;
		result.e0123 = dq.real;
		return result;
	}

	// Its crazy, the dual of a point returns a plane
	inline const MATH::Plane operator ! (const MATH::Vec4& point){
		MATH::Plane result;
		result.e0 = point.e123;
		result.e1 = point.e032;
		result.e2 = point.e013;
		result.e3 = point.e021;
		return result;
	}

	// The dual of a plane is a point. The world makes sense
	inline const MATH::Vec4 operator ! (const MATH::Plane& plane) {
		MATH::Vec4 result;
		result.e032 = plane.e1;
		result.e013 = plane.e2;
		result.e021 = plane.e3;
		result.e123 = plane.e0;
		return result;
	}
}
#endif