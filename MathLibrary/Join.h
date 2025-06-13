#ifndef JOIN_H
#define JOIN_H
#include <Vector.h>
#include "DualQuat.h"
#include "Plane.h"
#include "Dual.h"
#include "Meet.h"
namespace MATHEX {

	// The join represents when you "join" two things together. Get it?
	// For example if you "join" two point, it gives you the line that goes through them
	// WHY?     https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Why_combine_things.pdf
	// More thoughts at the end of this file
	
	// Two points join to make a line
	// Look how easy that is now that we have the Poincare Duality (!) and the Meet (^)
	// EXAMPLE: https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Example_joining_points.pdf
	inline const DualQuat join(const MATH::Vec4& p1, const MATH::Vec4& p2) {
		// The mess below means: dual(meet(dual(p1), dual(p2)))
		return !(!p1 ^ !p2);
	}
	inline const DualQuat operator & (const MATH::Vec4& p1, const MATH::Vec4& p2) {
		return join(p1, p2);
	}

	// A line and a point join to make a plane
	inline const Plane join(const DualQuat& q, const MATH::Vec4& p) {
		return !(!q ^ !p);
	}

	inline const Plane operator & (const DualQuat& q, const MATH::Vec4& p) {
		return join(q, p);
	}

	// Joining a line and a point the other way around
	inline const Plane join(const MATH::Vec4& p, const DualQuat& q) {
		return !(!p ^ !q);
	}
	inline const Plane operator & (const MATH::Vec4& p, const DualQuat& q) {
		return join(p, q);
	}
}
#endif

// Joining two points is handy as it is difficult to see how dual quaternions relate to lines
// It's written as an upside down "^" in  math, but unfortunately that is not available on my keyboard
// Consensus seems to be to go with overloading "&". I guess because you are joining this "AND" that. Get it?
// The join is also known as the regressive product
// I like to think of it as a complement to the meet. That's why there are so many duals.