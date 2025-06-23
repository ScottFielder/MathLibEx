#ifndef QUADMATH_H
#define QUADMATH_H
#include "Quad.h"
#include "PMath.h"
#include "Join.h"
#include "DQMath.h"
#include <map>
#include <string>

namespace MATHEX {

	class QuadMath {
	public:
		// Join three points to get a plane
		static const Plane getPlane(const Quad& quad) {
			return Vec4(quad.getV0()) & Vec4(quad.getV1()) & Vec4(quad.getV2());
		}

		// Find the surface area of the quad
		// REFERENCE: https://github.com/ScottFielder/MathLibEx/blob/master/Literature/3D_PGA_Cheat_Sheet_2019_siggraph.pdf
		static const float getArea(const Quad& quad) {
			// Find the edge lines that go all the way round the quad
			DualQuat line01 = Vec4(quad.getV0()) & Vec4(quad.getV1());
			DualQuat line12 = Vec4(quad.getV1()) & Vec4(quad.getV2());
			DualQuat line23 = Vec4(quad.getV2()) & Vec4(quad.getV3());
			DualQuat line30 = Vec4(quad.getV3()) & Vec4(quad.getV0());
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
			float orientedDist0 = DQMath::orientedDist(Vec4(point), Vec4(quad.getV0()) & Vec4(quad.getV1()));
			float orientedDist1 = DQMath::orientedDist(Vec4(point), Vec4(quad.getV1()) & Vec4(quad.getV2()));
			float orientedDist2 = DQMath::orientedDist(Vec4(point), Vec4(quad.getV2()) & Vec4(quad.getV3()));
			float orientedDist3 = DQMath::orientedDist(Vec4(point), Vec4(quad.getV3()) & Vec4(quad.getV0()));

			// Is the point on the left side of all edges? Or the right side of all the edges?
			// Leave a bit of wiggle room for numerical error
			const float epsilon = VERY_SMALL * 10.0f;
			if (orientedDist0 >= -epsilon && orientedDist1 >= -epsilon && orientedDist2 >= -epsilon && orientedDist3 >= -epsilon) {
				return true;
			}
			else if (orientedDist0 <= epsilon && orientedDist1 <= epsilon && orientedDist2 <= epsilon && orientedDist3 <= epsilon) {
				return true;
			}
			// If not then we are outside the quad
			return false;
		}

		// Returns the closest point on the quad based on the position given
		// UN - Tested 2025-03-10
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
				DualQuat line01 = Vec4(quad.getV0()) & Vec4(quad.getV1());
				DualQuat line12 = Vec4(quad.getV1()) & Vec4(quad.getV2());
				DualQuat line23 = Vec4(quad.getV2()) & Vec4(quad.getV3());
				DualQuat line30 = Vec4(quad.getV3()) & Vec4(quad.getV0());

				// Project the point on the plane to all four edge lines
				Vec4 pointOnLine01 = VMath::perspectiveDivide(DQMath::project(pointOnPlane, line01));
				Vec4 pointOnLine12 = VMath::perspectiveDivide(DQMath::project(pointOnPlane, line12));
				Vec4 pointOnLine23 = VMath::perspectiveDivide(DQMath::project(pointOnPlane, line23));
				Vec4 pointOnLine30 = VMath::perspectiveDivide(DQMath::project(pointOnPlane, line30));

				// Are we in one of those pesky Voronoi regions?
				// PIC: https://github.com/ScottFielder/MathLibEx/blob/master/Images/quad_voronoi_regions.png
				// The directions of the lines in my pic are kinda important (though ultimately arbitrary)
				// I'll need to check the directions in relation to the sky at some point
				Vec4 dir;
				const Plane sky = Plane(0, 0, 0, 1);

				// Let's start with region 0 outside V0
				DualQuat lineVoronoi0_0 = pointOnLine30 & pointOnPlane;
				DualQuat lineVoronoi0_1 = pointOnLine01 & pointOnPlane;

				// Lines need to go through vertex V0, so project it
				lineVoronoi0_0 = DQMath::project(lineVoronoi0_0, quad.getV0());
				lineVoronoi0_1 = DQMath::project(lineVoronoi0_1, quad.getV0());

				// To match my pic 
				// Ensure lineVoronoi0_0 is pointing roughly right
				// and lineVoronoi0_1 is pointing roughly down
				// projecting the line onto the point sometimes switches direction. So best to do this check after projecting
				dir = lineVoronoi0_0 ^ sky;
				if (dir.x < 0) {
					lineVoronoi0_0 = lineVoronoi0_0 * -1;
				}
				dir = lineVoronoi0_1 ^ sky;
				if (dir.y > 0) {
					lineVoronoi0_1 = lineVoronoi0_1 * -1;
				}

				// We are in Voronoi region if the point on the plane
				// is on the right of both lines
				float orientedDist0_0 = DQMath::orientedDist(pointOnPlane, lineVoronoi0_0);
				float orientedDist0_1 = DQMath::orientedDist(pointOnPlane, lineVoronoi0_1);
				// In this case, the oriented distances will both be negative for the right side. 
				if (orientedDist0_0 < VERY_SMALL &&
					orientedDist0_1 < VERY_SMALL) {
					return quad.getV0();
				}

				// Region 1 outside V1
				DualQuat lineVoronoi1_0 = pointOnLine01 & pointOnPlane;
				DualQuat lineVoronoi1_1 = pointOnLine12 & pointOnPlane;

				// Lines need to go through vertex V1, so project it
				lineVoronoi1_0 = DQMath::project(lineVoronoi1_0, quad.getV1());
				lineVoronoi1_1 = DQMath::project(lineVoronoi1_1, quad.getV1());

				// To match my pic 
				// Ensure lineVoronoi1_0 is pointing roughly up
				// and    lineVoronoi1_1 is pointing roughly right
				dir = lineVoronoi1_0 ^ sky;
				if (dir.y < 0) {
					lineVoronoi1_0 = lineVoronoi1_0 * -1;
				}
				dir = lineVoronoi1_1 ^ sky;
				if (dir.x < 0) {
					lineVoronoi1_1 = lineVoronoi1_1 * -1;
				}

				// We are in Voronoi region if the point on the plane
				// is on the right of both lines
				float orientedDist1_0 = DQMath::orientedDist(pointOnPlane, lineVoronoi1_0);
				float orientedDist1_1 = DQMath::orientedDist(pointOnPlane, lineVoronoi1_1);
				if (orientedDist1_0 < VERY_SMALL &&
					orientedDist1_1 < VERY_SMALL) {
					return quad.getV1();
				}

				// Region 2 outside V2
				DualQuat lineVoronoi2_0 = pointOnLine12 & pointOnPlane;
				DualQuat lineVoronoi2_1 = pointOnLine23 & pointOnPlane;

				// Lines need to go through vertex V2, so project it
				lineVoronoi2_0 = DQMath::project(lineVoronoi2_0, quad.getV2());
				lineVoronoi2_1 = DQMath::project(lineVoronoi2_1, quad.getV2());

				// To match my pic 
				// Ensure lineVoronoi2_0 is pointing roughly left
				// and    lineVoronoi2_1 is pointing roughly up
				dir = lineVoronoi2_0 ^ sky;
				if (dir.x > 0) {
					lineVoronoi2_0 = lineVoronoi2_0 * -1;
				}
				dir = lineVoronoi2_1 ^ sky;
				if (dir.y < 0) {
					lineVoronoi2_1 = lineVoronoi2_1 * -1;
				}

				float orientedDist2_0 = DQMath::orientedDist(pointOnPlane, lineVoronoi2_0);
				float orientedDist2_1 = DQMath::orientedDist(pointOnPlane, lineVoronoi2_1);
				if (orientedDist2_0 < VERY_SMALL &&
					orientedDist2_1 < VERY_SMALL) {
					return quad.getV2();
				}

				// Last Voronoi region to check
				// Region 3 outside V3
				DualQuat lineVoronoi3_0 = pointOnLine23 & pointOnPlane;
				DualQuat lineVoronoi3_1 = pointOnLine30 & pointOnPlane;

				// Lines need to go through vertex V3, so project it
				lineVoronoi3_0 = DQMath::project(lineVoronoi3_0, quad.getV3());
				lineVoronoi3_1 = DQMath::project(lineVoronoi3_1, quad.getV3());

				// To match my pic
				// Ensure lineVoronoi3_0 is pointing roughly down
				// and    lineVoronoi3_1 is pointing roughly left
				dir = lineVoronoi3_0 ^ sky;
				if (dir.y > 0) {
					lineVoronoi3_0 = lineVoronoi3_0 * -1;
				}
				dir = lineVoronoi3_1 ^ sky;
				if (dir.x > 0) {
					lineVoronoi3_1 = lineVoronoi3_1 * -1;
				}

				float orientedDist3_0 = DQMath::orientedDist(pointOnPlane, lineVoronoi3_0);
				float orientedDist3_1 = DQMath::orientedDist(pointOnPlane, lineVoronoi3_1);
				if (orientedDist3_0 < VERY_SMALL &&
					orientedDist3_1 < VERY_SMALL) {
					return quad.getV3();
				}

				// All done checking the four Voronoi regions
				// if we werent in a voronoi region after all, then use the closest point on line edges
				float orientedDist01 = DQMath::orientedDist(pointOnPlane, line01);
				float orientedDist12 = DQMath::orientedDist(pointOnPlane, line12);
				float orientedDist23 = DQMath::orientedDist(pointOnPlane, line23);
				float orientedDist30 = DQMath::orientedDist(pointOnPlane, line30);

				// Put all the distances and pointsOnLine in a std::map
				// It will automatically sort them by distance from smallest to largest
				// It will also delete any duplicate distances, so it's size might be smaller than 4. Be careful!
				std::map<float, Vec3> distanceToLines = {
					{ fabs(orientedDist01), pointOnLine01 },
					{ fabs(orientedDist12), pointOnLine12 },
					{ fabs(orientedDist23), pointOnLine23 },
					{ fabs(orientedDist30), pointOnLine30 }
				};

				for (const auto& pair : distanceToLines) {
					// Just use the closest edge
					if (QuadMath::isPointInside(pair.second, quad)) {
						return Vec3(pair.second);
					}

				}
			}
		}
	};
}
#endif // !QUADMATH_H

