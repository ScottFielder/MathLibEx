#ifndef TMATH_H
#define TMATH_H
#include "Triangle.h"
#include "Plane.h"
#include "Join.h"
#include "DQMath.h"

namespace MATHEX {

	class TMath {
	public:

		// UN - Tested 2025-02-24 for Sphere-Triangle collision assignment
		static const Plane getPlane(const Triangle& t) {
			// Join three points to get a plane
			return t.getV0() & t.getV1() & t.getV2();

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
		static const bool isPointOnTrianglePlane(const MATH::Vec3& v, const Triangle& t) {
			float distFromPlane = DQMath::orientedDist(v, getPlane(t));
			if (fabs(distFromPlane) > VERY_SMALL) {
				return false;
			}
			else {
				return true;
			}
		}

		// UN - Tested 2025-02-24 for Sphere-Triangle collision assignment
		static const bool isPointInsideTriangle(const MATH::Vec3& v, const Triangle& t) {
			// Are we in the plane of the triangle at least?	
			if (!isPointOnTrianglePlane(v, t)) return false;
			// Ok we are in the plane at least, now let's check if we are inside the triangle
			float orientedDist0 = DQMath::orientedDist(v, join(t.getV0(), t.getV1()));
			float orientedDist1 = DQMath::orientedDist(v, join(t.getV1(), t.getV2()));
			float orientedDist2 = DQMath::orientedDist(v, join(t.getV2(), t.getV0()));

			// Is the point on the left side of all edges? Or the right side of all the edges?
			const float BIGGER_THAN_VERY_SMALL = 1.0e-6f;
			// Are all distances basically positive give or take a tiny bit
			if (orientedDist0 >= -BIGGER_THAN_VERY_SMALL && orientedDist1 >= -BIGGER_THAN_VERY_SMALL && orientedDist2 >= -BIGGER_THAN_VERY_SMALL) {
				return true;
			}
			// Or are all the distances negative give or take a tiny bit
			else if (orientedDist0 < BIGGER_THAN_VERY_SMALL && orientedDist1 < BIGGER_THAN_VERY_SMALL && orientedDist2 < BIGGER_THAN_VERY_SMALL)
			{
				return true;
			}
			// If not then we are outside the triangle
			return false;
		}

		// This method is just like isPointInsideTriangle, but also takes into account the radius of the circle
		static const bool isCircleTouchingTriangle(const MATH::Vec3& centre, const float radius, const Triangle& t) {
			// Are we in the plane of the triangle at least?	
			if (!isPointOnTrianglePlane(centre, t)) return false;
			// Ok we are in the plane at least, now let's check if we are inside the triangle
			float orientedDist0 = DQMath::orientedDist(centre, join(t.getV0(), t.getV1()));
			float orientedDist1 = DQMath::orientedDist(centre, join(t.getV1(), t.getV2()));
			float orientedDist2 = DQMath::orientedDist(centre, join(t.getV2(), t.getV0()));

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

