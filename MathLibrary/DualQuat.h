#ifndef DUALQUATH
#define DUALQUAT_H
#include "QMath.h"

/// A dual quaternion can handle rotations and translations. Contains 8 floats

namespace MATH {
	struct DualQuat
	{
		float w;   /// real number, just like the w in a regular quaternion
		float e23; /// This is like -i for a regular quaternion
		float e31; /// -j
		float e12; /// -k
		float e01; /// Every term with a zero in it squares to zero. It's how translations are handled. 
		float e02;
		float e03;
		float e0123; /// Hamish Todd calls this one "screwiness", for a rotation + translation type motion

		inline void set(float w_, float e23_, float e31_, float e12_, float e01_, float e02_, float e03_, float e0123_){
			w = w_; e23 = e23_; e31 = e31_; e12 = e12_; e01 = e01_; e02 = e02_; e03 = e03_; e0123 = e0123_;
		}

		/// This is the unit dual quaternion. Doesn't rotate or translate
		/// It is literally the number 1
		inline DualQuat() {
			set(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		}

		inline DualQuat(float w_, float e23_, float e31_, float e12_, float e01_, float e02_, float e03_, float e0123_) {
			set(w_, e23_, e31_, e12_, e01_, e02_, e03_, e0123_);
		}

		/// A copy constructor
		inline DualQuat(const DualQuat& dq) {
			set(dq.w, dq.e23, dq.e31, dq.e12, dq.e01, dq.e02, dq.e03, dq.e0123);
		}

		/// An assignment operator   
		inline DualQuat& operator = (const DualQuat& dq) {
			set(dq.w, dq.e23, dq.e31, dq.e12, dq.e01, dq.e02, dq.e03, dq.e0123);
			return *this;
		}

		/// I wrote this out on paper. My wrist is still hurting.
		/// You don't need the brackets, it just helps my eyes
		inline const DualQuat operator * (const DualQuat& b) const {
			DualQuat result;
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

		inline const DualQuat operator / (float c) const {
#ifdef DEBUG 
			if (std::fabs(c) < VERY_SMALL) {
				std::string errorMsg = __FILE__ + __LINE__;
				throw errorMsg.append(": Divide by nearly zero! ");
			}
#endif
			float r = 1.0f / c;
			return *this * r;
		}

		inline const DualQuat operator + (const DualQuat& dq) const {
			DualQuat result;
			result.w = w + dq.w;
			result.e23 = e23 + dq.e23;
			result.e31 = e31 + dq.e31;
			result.e12 = e12 + dq.e12;
			result.e01 = e01 + dq.e01;
			result.e02 = e02 + dq.e02;
			result.e03 = e03 + dq.e03;
			result.e0123 = e0123 + dq.e0123;
			return result;
		}

		inline const DualQuat operator - (const DualQuat& dq) const {
			DualQuat result;
			result.w = w - dq.w;
			result.e23 = e23 - dq.e23;
			result.e31 = e31 - dq.e31;
			result.e12 = e12 - dq.e12;
			result.e01 = e01 - dq.e01;
			result.e02 = e02 - dq.e02;
			result.e03 = e03 - dq.e03;
			result.e0123 = e0123 - dq.e0123;
			return result;
		}

		void print(const char* comment = nullptr) const {
			if (comment) printf("%s\n", comment);
			printf("%1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f\n", w, e23, e31, e12, e01, e02, e03, e0123);
		}

	};
}

#endif

/*** Note 1. Umer Noor June 2023
REFERENCE: 2023 GDC Talks by Hamish Todd and information at bivector.net
           Find the talks at https://library.humber.ca/atoz_landing/G
***/

/*** Note 2. Umer Noor June 2023
I like to think of the first four numbers like a quaternion that rotates about the origin
and the second group of four numbers like an even stranger quaternion that rotates about infinity

e23 squares to -1. 
It's called e23 as a rotation about x gets you from y (2) to z (3)
I've seen the coefficient written as yz
If you are wondering what e23 really is, it is e2 * e3.
e2 is a reflection in y plane. e3 is a reflection in the z plane
e2 and e3 square to 1, just like a real mirror. You can find them in the Plane class
In a similar way e31 is a rotation about y that gets you from z (3) to x (1)
and e12 as a rotation about z gets you from x (1) to y (2). The coefficients for these can be written as zx and xy

For a pure translation, with no rotation, e01 is change in x position / 2. 
It squares to zero. I know, it's crazy. Imagine rotating about an axis infinitely far away...
Coefficient for e01 is written as dx
e01 is actually e0 * e1, where e0 is a reflection in a plane infinitely far away. e0 squares to zero
In a similar way, e02 and e03 are the changes in y or z position / 2. You can write them as dy and dz
Funny how the "divide by 2" matches what you do to the angle in a regular quaternion. There is something deep here...
The last element e0123 is a bit weird. It encodes a screw type motion that combines rotation and translation
***/