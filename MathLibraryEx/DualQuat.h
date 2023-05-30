// 2023 May - Umer Noor
// A dual quaternion can handle rotations and translations! Take that 4x4 matrices!
// REFERENCE: 2023 GDC Talks by Hamish Todd and information at bivector.net
//            Find the talks at https://library.humber.ca/atoz_landing/G
#ifndef DUALQUAT_H
#define DUALQUAT_H
#include <QMath.h>

namespace MATHEX {
	struct DualQuat
	{
		// A dual quaternion is made up of 8 floats
		// The first four numbers handle rotation and the second group of four numbers handles translation
		// I like to think of the first four numbers like a quaternion that rotates about the origin
		// and the second group of four numbers like an even stranger quaternion that rotates about infinity
		//
		// w is just like the one in a regular quaternion. A real number
		float w;
		// e23 is -i for a quaternion. It squares to -1. 
		// It's called e23 as a rotation about x gets you from y (2) to z (3)
		// I've seen the coefficient written as yz
		// If you are wondering what e23 really is, it is e2 * e3. 
		// e2 is a reflection in y plane. e3 is a reflection in the z plane
		// e2 and e3 square to 1, just like a real mirror.
		float e23;
		// e31 is -j for a quaternion. It squares to -1. 
		// It's called e31 as a rotation about y gets you from z (3) to x (1)
		// Coefficient is written as zx
		// e31 is actually e3 * e1, where e1 is a reflection in the x plane. It squares to 1 too
		float e31;
		// e12 is -k for a quaternion. Guess what it squares to?
		// It's called e12 as a rotation about z gets you from x (1) to y (2)
		// Coefficient is written as xy
		float e12;
		//
		// e01 is change in x position / 2. It squares to zero. I know, it's crazy.
		// Coefficient is written as dx
		// e01 = e0 * e1, where e0 is a reflection in a plane infinitely far away. e0 squares to zero
		float e01;
		// e02 is change in y position / 2. It squares to zero too.
		// Coefficient is written as dy
		float e02;
		// e03 is change in z position / 2. You know what, whenever you see a zero, just assume it squares to zero.
		// Coefficient is written as dz
		float e03;
		// e0123 is a combo of rotationa and translation. Hamish Todd calls it "screwiness"
		// Coefficient is written as dxyz
		float e0123;

		DualQuat() {
			// Set to the identity. Doesn't rotate or translate
			w = 1.0f;
			e23 = e31 = e12 = e01 = e02 = e03 = e0123 = 0.0f;
		}

		DualQuat(MATH::Quaternion rotation) {
			// No translation, but use the quaternion to build the first four floats
			// Remember e23 = -i, e31 = -j, e12 = -k
			w = rotation.w;
			e23 = -rotation.ijk.x;
			e31 = -rotation.ijk.y;
			e12 = -rotation.ijk.z;
			e01 = e02 = e03 = e0123 = 0.0f;
		}

		DualQuat(MATH::Vec3 translation) {
			// No rotation, but set the last four floats to be half the translation
			w = 1.0f;
			e23 = e31 = e12 = 0.0f;
			e01 = translation.x / 2.0f;
			e02 = translation.y / 2.0f;
			e03 = translation.z / 2.0f;
			e0123 = 0.0f;
		}

		inline const DualQuat operator * (const DualQuat& b) const {
			DualQuat result;
			// I wrote this out on paper. My wrist is still hurting.
			// You don't need the brackets, it just helps my eyes
			result.w = (w * b.w) - (e23 * b.e23) - (e31 * b.e31) - (e12 * b.e12);
			result.e23 = (w * b.e23) + (e23 * b.w) - (e31 * b.e12) + (e12 * b.e31);
			result.e31 = (w * b.e31) + (e23 * b.e12) + (e31 * b.w) - (e12 * b.e23);
			result.e12 = (w * b.e12) - (e23 * b.e31) + (e31 * b.e23) + (e12 * b.w);
			result.e01 = (w * b.e01) - (e23 * b.e0123) - (e31 * b.e03) + (e12 * b.e02)
				+ (e01 * b.w) - (e02 * b.e12) + (e03 * b.e31) - (e0123 * b.e23);
			result.e02 = (w * b.e02) + (e23 * b.e03) - (e31 * b.e0123) - (e12 * b.e01)
				+ (e01 * b.e12) - (e03 * b.e23) + (e02 * b.w) - (e0123 * b.e31);
			result.e03 = (w * b.e03) - (e23 * b.e02) + (e31 * b.e01) - (e12 * b.e0123)
				- (e01 * b.e31) + (e02 * b.e23) + (e03 * b.w) - (e0123 * b.e12);
			result.e0123 = (w * b.e0123) + (e23 * b.e01) + (e31 * b.e02) + (e12 * b.e03)
				+ (e01 * b.e23) + (e02 * b.e31) + (e03 * b.e12) + (e0123 * b.w);
			return result;
		}

		inline const DualQuat operator * (float c) const {
			DualQuat result;
			result.w = w * c;
			result.e23 = e23 * c;
			result.e31 = e31 * c;
			result.e12 = e12 * c;
			result.e01 = e01 * c;
			result.e02 = e02 * c;
			result.e03 = e03 * c;
			result.e0123 = e0123 * c;
			return result;
		}


		/// print the values of the sphere and add a comment if you wish
		void print(const char* comment = nullptr) const {
			if (comment) printf("%s\n", comment);
			printf("%1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f\n", w, e23, e31, e12, e01, e02, e03, e0123);
		}
	};
}
#endif

