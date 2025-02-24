#ifndef TRIANGLE_H
#define TRIANGLE_H
#include <VMath.h>
#include "Plane.h"
#include "DQMath.h"
#include "Join.h"

namespace  MATHEX {
	class Triangle {
	private:
		// A triangle is made up of three vertices
		// I'm keeping them private as not all three vertices make a valid triangle
		MATH::Vec3 v0, v1, v2;

		inline bool isValid() const {
			// A valid triangle has edges that are not along the same line
			MATH::Vec3 edge1 = v1 - v0;
			MATH::Vec3 edge2 = v2 - v0;
			MATH::Vec3 crossP = MATH::VMath::cross(edge1, edge2);
			if(MATH::VMath::mag(crossP) < VERY_SMALL) {
				return false;
			}
			return true;
		}

	public:
		inline void set(MATH::Vec3 v0_, MATH::Vec3 v1_, MATH::Vec3 v2_) {
			v0 = v0_;
			v1 = v1_;
			v2 = v2_;
#ifdef _DEBUG  /// If in debug mode let's worry if this triangle makes sense
			if (!isValid()) {
				std::string errorMsg = __FILE__ + __LINE__;
				throw errorMsg.append(": This is not a valid triangle");
			}
#endif // DEBUG
		}

		inline Triangle(MATH::Vec3 v0_, MATH::Vec3 v1_, MATH::Vec3 v2_) {
			set(v0_, v1_, v2_);
		}

		// A copy constructor
		inline Triangle(const Triangle& t) {
			set(t.v0, t.v1, t.v2);
		}

		// An assignment operator   
		inline Triangle& operator = (const Triangle& t) {
			set(t.v0, t.v1, t.v2);
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




		void print(const char* comment = nullptr) const {
			if (comment) printf("%s\n", comment);
			printf("v0: %1.8f %1.8f %1.8f\nv1: %1.8f %1.8f %1.8f\nv2: %1.8f %1.8f %1.8f\n",
				v0.x, v0.y, v0.z, 
				v1.x, v1.y, v1.z, 
				v2.x, v2.y, v2.z);
		}

	};
}
#endif // !TRIANGLE_H

