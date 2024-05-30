#ifndef PLANE_H
#define PLANE_H
#include <iostream>
#include "VMath.h"


namespace  MATH {
	union Plane {
		struct { /// Standard Euclidean definition ax + by + cz - d = 0 
			float x,y,z,d;
		};

		/// See note 1 at the end of this file
		struct { /// A plane in projective geometric algebra 	
			float e1, e2, e3, e0;
		};

		struct {
			Vec3 n;
			float negativeDist;  /// Negative signed distance from the orgin along the normal;
		};


		/// Just a little utility to populate a Plane
		inline void set(float x_, float y_, float z_, float d_) {
			x = x_, y = y_, z = z_, d = d_;
		}

		/// Here's a set of constructors:
		/// If the plane is defined by a normal, 
		/// the equation of a Plane is ax + by + cz - d = 0; where 
		/// a,b,c are the values of the normal n <a,b,c> and the -d part is the 
		/// negative signed distance from the origin to the plane
		Plane(Vec3 n, float d) {
			set(n.x, n.y, n.z, d);

#ifdef _DEBUG  /// If in debug mode let's worry the normal being normalized
			float mag = VMath::mag(n);
			if (std::fabs(mag) - 1.0f > VERY_SMALL) {
				std::string errorMsg = __FILE__ + __LINE__;
				throw errorMsg.append(": The normal in the Plane constructor was not normalized");
			}
#endif // DEBUG
		}
		Plane() {
			set(0.0f, 0.0f, 0.0f, 0.0f);
		}

		/// If the plane is defined by three points v0, v1, v2 
		/// then the equation of a Plane is ax + by + cz - d = 0.
		Plane(const Vec3& v0, const Vec3& v1, const Vec3& v2) {
			Vec3 a = v1 - v0;
			Vec3 b = v2 - v0;
			Vec3 n = VMath::normalize(VMath::cross(a, b));
			float signedDistance = VMath::dot(n, v0);
			set(n.x, n.y, n.z, -signedDistance);
		}

		/// A copy constructor
		Plane(const Plane& p) {
			set(p.x, p.y, p.z, p.d);
		}


		/// These just set numbers - be careful.  
		Plane(float x, float y, float z, float d) {
			set(x, y, z, d);
		}

		float magGrade1() const {
			return sqrt(e1 * e1 + e2 * e2 + e3 * e3);
		}

		float magGrade1Infinity() const {
			return sqrt(e0 * e0);
		}

		///////////////////////// Operator overloads ////////////////////////////

		inline Plane& operator = (const Plane& p) {
			set(p.x, p.y, p.z, p.d);
			return *this;
		}

		const Plane operator * (float s) const {
			return Plane(e1 * s, e2 * s, e3 * s, e0 * s);
		}

		friend const Plane operator * (const float s, const Plane& p) {
			return p * s;
		}

		const Plane operator / (float s) const {
			return Plane(e1 / s, e2 / s, e3 / s, e0 / s);
		}

		friend const Plane operator / (const float s, const Plane& p) {
			return p / s;
		}

		const Plane operator + (const Plane& p) const {
			return Plane(e1 + p.e1, e2 + p.e2, e3 + p.e3, e0 + p.e0);
		}

		const Plane operator - (const Plane& p) const {
			return Plane(e1 - p.e1, e2 - p.e2, e3 - p.e3, e0 - p.e0);
		}

		Plane& operator += (const Plane& p) {
			e1 += p.e1;
			e2 += p.e2;
			e3 += p.e3;
			e0 += p.e0;
			return *this;
		}

		Plane& operator -= (const Plane& p) {
			e1 -= p.e1;
			e2 -= p.e2;
			e3 -= p.e3;
			e0 -= p.e0;
			return *this;
		}


		/// print the values of the plane and add a comment if you wish
		void print(const char* comment = nullptr) const {
			if (comment) printf("%s\n", comment);
			printf("%f %f %f %f\n", x, y, z, d);
		}


	};
}
/*** Note 1
// 2024 Feb - Umer Noor
// A mirror plane in projective geometric algebra is just a like a regular plane
// REFERENCE: 2023 GDC Talks by Hamish Todd and information at bivector.net
//            Find the talks at https://library.humber.ca/atoz_landing/G
***/

#endif