#ifndef QUADMATH_H
#define QUADMATH_H
#include "Quad.h"
#include "PMath.h"
#include "Join.h"
#include "DQMath.h"

namespace MATHEX {

	class QuadMath {
	public:
		// Join three points to get a plane
		static const Plane getPlane(const Quad& quad) {
			return quad.getV0() & quad.getV1() & quad.getV2();

			/// Would not this be more intuitive? SSF
			//return Plane(t.getV0(), t.getV1(), t.getV2());
		}

		// REFERENCE: https://github.com/ScottFielder/MathLibEx/blob/master/Literature/3D_PGA_Cheat_Sheet_2019_siggraph.pdf
		static const float getArea(const Quad& quad) {
			// Find the edge lines that go all the way round the quad
			DualQuat line01 = quad.getV0() & quad.getV1();
			DualQuat line12 = quad.getV1() & quad.getV2();
			DualQuat line23 = quad.getV2() & quad.getV3();
			DualQuat line30 = quad.getV3() & quad.getV0();
			// Add them up
			DualQuat sum = line01 + line12 + line23 + line30;
			// Now take the magnitude of the infinite part of the sum and divide by two
			return 0.5f * DQMath::magGrade2Infinity(sum);
		}

		static const MATH::Vec3 getNormal(const Quad& quad) {
			Plane plane = getPlane(quad);
#ifdef _DEBUG  /// If in debug mode let's worry about divide by zero or nearly zero!!! 	
			Vec3 normal = Vec3(plane.e1, plane.e2, plane.e3);
			if (VMath::mag(normal) < VERY_SMALL) {
				std::string errorMsg = __FILE__ + __LINE__;
				throw errorMsg.append(": Divide by nearly zero! ");
			}
#endif
			return VMath::normalize(normal);
		}

		static const bool isPointOnPlane(const MATH::Vec3& point, const Quad& quad) {
			float distFromPlane = PMath::orientedDist(point, getPlane(quad));
			if (fabs(distFromPlane) > VERY_SMALL) return false;
			return true;
		}

		static const bool isPointInside(const MATH::Vec3& point, const Quad& quad) {
			// Are we in the plane of the quad at least?	
			if (!isPointOnPlane(point, quad)) return false;
			// Ok we are in the plane at least, now let's check if we are inside the quad
			float orientedDist0 = DQMath::orientedDist(point, join(quad.getV0(), quad.getV1()));
			float orientedDist1 = DQMath::orientedDist(point, join(quad.getV1(), quad.getV2()));
			float orientedDist2 = DQMath::orientedDist(point, join(quad.getV2(), quad.getV3()));
			float orientedDist3 = DQMath::orientedDist(point, join(quad.getV3(), quad.getV0()));

			// Is the point on the left side of all edges? Or the right side of all the edges?
			// Leave a bit of wiggle room for numerical error
			const float smallNumber = VERY_SMALL * 10.0f;
			if (orientedDist0 >= smallNumber && orientedDist1 >= smallNumber && orientedDist2 >= smallNumber && orientedDist3 >= smallNumber) {
				return true;
			}
			else if (orientedDist0 <= smallNumber && orientedDist1 <= smallNumber && orientedDist2 <= smallNumber && orientedDist3 <= smallNumber) {
				return true;
			}
			// If not then we are outside the triangle
			return false;
		}

	};
}
#endif // !QUADMATH_H

