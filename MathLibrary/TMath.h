#ifndef TMATH_H
#define TMATH_H
#include "Triangle.h"
#include "Plane.h"
#include "Join.h"
#include "DQMath.h"

namespace MATHEX {

	class TMath {
	public:
		static const Plane getPlane(const Triangle& t) {
			// Join three points to get a plane
			return t.getV0() & t.getV1() & t.getV2();
		}

		static const MATH::Vec3 getNormal(const Triangle& t) {
			Plane plane = getPlane(t);
#ifdef _DEBUG  /// If in debug mode let's worry about divide by zero or nearly zero!!! 	
			Vec3 normal = Vec3(plane.e1, plane.e2, plane.e3);
			if (VMath::mag(normal) < VERY_SMALL) {
				std::string errorMsg = __FILE__ + __LINE__;
				throw errorMsg.append(": Divide by nearly zero! ");
			}
#endif
			return VMath::normalize(normal);
		}

		static const bool isPointOnTrianglePlane(const MATH::Vec3& v, const Triangle& t) {
			float distFromPlane = DQMath::orientedDist(v, getPlane(t));
			if (fabs(distFromPlane) > VERY_SMALL) return false;
			return true;
		}

		static const bool isPointInsideTriangle(const MATH::Vec3& v, const Triangle& t) {
			// Are we in the plane of the triangle at least?	
			if (!isPointOnTrianglePlane(v, t)) return false;
			// Ok we are in the plane at least, now let's check if we are inside the triangle
			float orientedDist0 = DQMath::orientedDist(v, join(t.getV0(), t.getV1()));
			float orientedDist1 = DQMath::orientedDist(v, join(t.getV1(), t.getV2()));
			float orientedDist2 = DQMath::orientedDist(v, join(t.getV2(), t.getV0()));

			// Is the point on the left side of all edges? Or the right side of all the edges?
			if (orientedDist0 >= 0.0f && orientedDist1 >= 0.0f && orientedDist2 >= 0.0f) {
				return true;
			}
			else if (orientedDist0 < 0.0f && orientedDist1 < 0.0f && orientedDist2 < 0.0f)
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

