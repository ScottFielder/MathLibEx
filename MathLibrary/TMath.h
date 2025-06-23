#ifndef TMATH_H
#define TMATH_H
#include "Triangle.h"
#include "Plane.h"
#include "Join.h"
#include "DQMath.h"
#include "PMath.h"

namespace MATHEX {

	class TMath {
	public:

		// UN - Tested 2025-02-24 for Sphere-Triangle collision assignment
		static const Plane getPlane(const Triangle& t) {
			// Join three points to get a plane
			return Vec4(t.getV0()) & Vec4(t.getV1()) & Vec4(t.getV2());

			/// Would not this be more intuitive? SSF
			//return Plane(t.getV0(), t.getV1(), t.getV2());
		}

		// UN - Tested 2025-02-24 for Sphere-Triangle collision assignment
		static const MATH::Vec3 getNormal(const Triangle& t) {
			MATH::Vec3 normal = getPlane(t).n;
#ifdef _DEBUG  /// If in debug mode let's worry about divide by zero or nearly zero!!! 	
			if (VMath::mag(normal) < VERY_SMALL) {
				std::string errorMsg = __FILE__ + __LINE__;
				throw errorMsg.append(": Divide by nearly zero! ");
			}
#endif
			return VMath::normalize(normal);
		}

		// UN - Tested 2025-02-24 for Sphere-Triangle collision assignment
		static const bool isPointOnPlane(const MATH::Vec3& v, const Triangle& t) {
			float distFromPlane = PMath::orientedDist(v, getPlane(t));
			if (fabs(distFromPlane) > VERY_SMALL) return false;
			return true;
		}

		// UN - Tested 2025-02-24 for Sphere-Triangle collision assignment
		static const bool isPointInside(const MATH::Vec3& v, const Triangle& t) {
			// Are we in the plane of the triangle at least?	
			if (!isPointOnPlane(v, t)) return false;
			// Ok we are in the plane at least, now let's check if we are inside the triangle
			float orientedDist0 = DQMath::orientedDist(Vec4(v), join(Vec4(t.getV0()), Vec4(t.getV1())));
			float orientedDist1 = DQMath::orientedDist(Vec4(v), join(Vec4(t.getV1()), Vec4(t.getV2())));
			float orientedDist2 = DQMath::orientedDist(Vec4(v), join(Vec4(t.getV2()), Vec4(t.getV0())));

			// Is the point on the left side of all edges? Or the right side of all the edges?
			// Leave a bit of wiggle room for numerical error
			const float epsilon = VERY_SMALL * 10.0f;
			if (orientedDist0 >= -epsilon && orientedDist1 >= -epsilon && orientedDist2 >= -epsilon) {
				return true;
			}
			else if (orientedDist0 <= epsilon && orientedDist1 <= epsilon && orientedDist2 <= epsilon) {
				return true;
			}
			// If not then we are outside the triangle
			return false;
		}


		// This method is just like isPointInside, but also takes into account the radius of the circle
		static const bool isCircleTouchingTriangle(const MATH::Vec3& centre, const float radius, const Triangle& t) {
			// Are we in the plane of the triangle at least?	
			if (!isPointOnPlane(centre, t)) return false;
			// Ok we are in the plane at least, now let's check if we are inside the triangle
			float orientedDist0 = DQMath::orientedDist(Vec4(centre), join(Vec4(t.getV0()), Vec4(t.getV1())));
			float orientedDist1 = DQMath::orientedDist(Vec4(centre), join(Vec4(t.getV1()), Vec4(t.getV2())));
			float orientedDist2 = DQMath::orientedDist(Vec4(centre), join(Vec4(t.getV2()), Vec4(t.getV0())));

			// Is the point on the left side of all edges? Or the right side of all the edges?
			// Take into account the radius
			// TODO (UN) I think I need to consider the voronoi region outside the corners of the triangles
			if (orientedDist0 >= -radius && orientedDist1 >= -radius && orientedDist2 >= -radius) {
				return true;
			}
			else if (orientedDist0 <= radius && orientedDist1 <= radius && orientedDist2 <= radius)
			{
				return true;
			}
			// If not then we are outside the triangle
			return false;
		}

		static const bool areAllVerticesInsideSphere(const MATH::Vec3& centre, float radius, const Triangle& t) {
			if ((VMath::distance(t.getV0(), centre) < radius) && (VMath::distance(t.getV1(), centre) < radius) && (VMath::distance(t.getV2(), centre))) {
				return true;
			}
			return false;
		}
	};
}
#endif // !TMATH_H

