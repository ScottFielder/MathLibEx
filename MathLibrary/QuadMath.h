#ifndef QUADMATH_H
#define QUADMATH_H
#include "Quad.h"
#include "PMath.h"
#include "Join.h"
#include "DQMath.h"
#include <map>

namespace MATHEX {

	class QuadMath {
	public:
		// Join three points to get a plane
		static const Plane getPlane(const Quad& quad) {
			return quad.getV0() & quad.getV1() & quad.getV2();
		}

		// Find the surface area of the quad
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
			Vec3 normal = getPlane(quad).n;
#ifdef _DEBUG  /// If in debug mode let's worry about divide by zero or nearly zero!!! 	
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

		// Check if the point is on the left of all the edges or the right of all the edges
		static const bool isPointInside(const MATH::Vec3& point, const Quad& quad) {
			// Are we in the plane of the quad at least?	
			if (!isPointOnPlane(point, quad)) return false;
			// Ok we are in the plane at least, now let's check if we are inside the quad
			float orientedDist0 = DQMath::orientedDist(point, quad.getV0() & quad.getV1());
			float orientedDist1 = DQMath::orientedDist(point, quad.getV1() & quad.getV2());
			float orientedDist2 = DQMath::orientedDist(point, quad.getV2() & quad.getV3());
			float orientedDist3 = DQMath::orientedDist(point, quad.getV3() & quad.getV0());

			// Is the point on the left side of all edges? Or the right side of all the edges?
			// Leave a bit of wiggle room for numerical error
			const float smallNumber = VERY_SMALL * 10.0f;
			if (orientedDist0 >= -smallNumber && orientedDist1 >= -smallNumber && orientedDist2 >= -smallNumber && orientedDist3 >= -smallNumber) {
				return true;
			}
			else if (orientedDist0 <= smallNumber && orientedDist1 <= smallNumber && orientedDist2 <= smallNumber && orientedDist3 <= smallNumber) {
				return true;
			}
			// If not then we are outside the quad
			return false;
		}

		// Returns the closest point on the quad based on the position given
		// Seems to work as long as you are kinda close to the quad
		// UN - Tested 2025-03-07 
		static const Vec3 closestPointOnQuad(const MATH::Vec3& pos, const Quad& quad) {
			// Project position onto the plane of the quad
			Vec4 pointOnPlane = PMath::project(pos, getPlane(quad));
			// Ensure w is one by dividing it out.
			pointOnPlane = VMath::perspectiveDivide(pointOnPlane);

			// If the projected point on the plane is actually in the quad, we are done. Hurrah!
			if (QuadMath::isPointInside(pointOnPlane, quad)) {
				return Vec3(pointOnPlane);
			}
			else {
				// Now we need to consider the outside edges of the quad
				// I overloaded the "&" operator to join two points into a line
				// Notice I am being careful in the order of the points to wind around the quad consistently
				DualQuat line01 = quad.getV0() & quad.getV1();
				DualQuat line12 = quad.getV1() & quad.getV2();
				DualQuat line23 = quad.getV2() & quad.getV3();
				DualQuat line30 = quad.getV3() & quad.getV0();

				// What are the two closest edges to the point on the plane?
				float dist01 = fabs(DQMath::orientedDist(pointOnPlane, line01));
				float dist12 = fabs(DQMath::orientedDist(pointOnPlane, line12));
				float dist23 = fabs(DQMath::orientedDist(pointOnPlane, line23));
				float dist30 = fabs(DQMath::orientedDist(pointOnPlane, line30));

				// Project the point on the plane to all four edge lines
				Vec4 pointOnLine01 = VMath::perspectiveDivide(DQMath::project(pointOnPlane, line01));
				Vec4 pointOnLine12 = VMath::perspectiveDivide(DQMath::project(pointOnPlane, line12));
				Vec4 pointOnLine23 = VMath::perspectiveDivide(DQMath::project(pointOnPlane, line23));
				Vec4 pointOnLine30 = VMath::perspectiveDivide(DQMath::project(pointOnPlane, line30));

				// Put all the distances and pointsOnLine in a std::map
					// It will automatically sort them by distance from smallest to largest
					// It will also delete any duplicate distances, so it's size might be smaller than 4. Be careful!
				std::map<float, Vec3> distanceToLines = {
					{ dist01, pointOnLine01 },
					{ dist12, pointOnLine12 },
					{ dist23, pointOnLine23 },
					{ dist30, pointOnLine30 }
				};

				// Loop through the map to find the closest line that has a projected point inside the quad
				int counter = 0;
				for (const auto& pair : distanceToLines) {
					// Check just the first and second closest lines
					if (QuadMath::isPointInside(pair.second, quad)) {
						return Vec3(pair.second);
					}
					// If we have already checked the two closest lines, then we are probably outside a corner
					// Or the rare case of only two elements in distanceToLines
					// Just pick the closest vertex and be done with it
					if (counter > 0 ) {
						float dist0 = VMath::distance(pos, quad.getV0());
						float dist1 = VMath::distance(pos, quad.getV1());
						float dist2 = VMath::distance(pos, quad.getV2());
						float dist3 = VMath::distance(pos, quad.getV3());
						// This map will order the distances from smallest to largest
						std::map<float, Vec3> distanceToVertices = {
							{ dist0, quad.getV0() },
							{ dist1, quad.getV1() },
							{ dist2, quad.getV2() },
							{ dist3, quad.getV3() }
						};
						// Pick the closest vertex
						return Vec3(distanceToVertices.begin()->second);
					}
					counter++;
				}
			}
		}

	};
}
#endif // !QUADMATH_H

