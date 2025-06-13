#ifndef QUAD_H
#define QUAD_H
#include "DQMath.h"
#include "Join.h"	
#include "PMath.h"
#include "Dot.h"

namespace  MATHEX {
	class Quad {
	private:
		// A quadrilateral is made up of 4 vertices
		// I'm keeping them private as not all 4 vertices make a valid quad
		MATH::Vec3 v0, v1, v2, v3;

		// A valid quad has an area that is not zero. Use the area of the edge loop
		// REFERENCE: https://github.com/ScottFielder/MathLibEx/blob/master/Literature/3D_PGA_Cheat_Sheet_2019_siggraph.pdf
		// Also, all four vertices should reside on the same plane (no bent quads)
		inline bool isValid() const {
			// Find the edge lines that go all the way around the quad using the join operator (&)
			DualQuat line01 = Vec4(v0) & Vec4(v1);
			DualQuat line12 = Vec4(v1) & Vec4(v2);
			DualQuat line23 = Vec4(v2) & Vec4(v3);
			DualQuat line30 = Vec4(v3) & Vec4(v0);
			// Add them up
			DualQuat sum = line01 + line12 + line23 + line30;
			// Now take the magnitude of the infinite line part of the sum
			float doubleArea = DQMath::magGrade2Infinity(sum);
			if (doubleArea < VERY_SMALL) {
				return false;
			}
			// Now check all vertices are on the same plane
			Plane p1 = line01 & v2; // join line from vertex 0 to 1 with vertex 2
			Plane p2 = line23 & v0; // join line from vertex 2 to 3 with vertex 0
			// Are these planes basically the same thing?
			return PMath::similar(p1, p2);
		}

	public:
		inline void set(MATH::Vec3 v0_, MATH::Vec3 v1_, MATH::Vec3 v2_, MATH::Vec3 v3_) {
			v0 = v0_;
			v1 = v1_;
			v2 = v2_;
			v3 = v3_;
#ifdef _DEBUG  /// If in debug mode let's worry if this triangle makes sense
			if (!isValid()) {
				std::string errorMsg = __FILE__ + __LINE__;
				throw errorMsg.append(": This is not a valid quad");
			}
#endif // DEBUG
		}

		// Wind the vertices anti-clockwise please
		// Otherwise my QuadMath::closestPointOnQuad function might not work...
		inline Quad(MATH::Vec3 v0_, MATH::Vec3 v1_, MATH::Vec3 v2_, MATH::Vec3 v3_) {
			set(v0_, v1_, v2_, v3_);
		}

		// A copy constructor
		inline Quad(const Quad& quad) {
			set(quad.v0, quad.v1, quad.v2, quad.v3);
		}

		// An assignment operator   
		inline Quad& operator = (const Quad& quad) {
			set(quad.v0, quad.v1, quad.v2, quad.v3);
			return *this;
		}

		inline const MATH::Vec3 getV0() const {
			return v0;
		}

		inline const MATH::Vec3 getV1() const {
			return v1;
		}

		inline const MATH::Vec3 getV2() const {
			return v2;
		}

		inline const MATH::Vec3 getV3() const {
			return v3;
		}


		void print(const char* comment = nullptr) const {
			if (comment) printf("%s\n", comment);
			printf("v0: %1.8f %1.8f %1.8f\nv1: %1.8f %1.8f %1.8f\nv2: %1.8f %1.8f %1.8f\nv3: %1.8f %1.8f %1.8f\n",
				v0.x, v0.y, v0.z,
				v1.x, v1.y, v1.z,
				v2.x, v2.y, v2.z,
				v3.x, v3.y, v3.z);
		}

	};
}
#endif // !QUAD_H

