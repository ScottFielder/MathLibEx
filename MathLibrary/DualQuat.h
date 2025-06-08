#ifndef DUALQUAT_H
#define DUALQUAT_H
#include "QMath.h"

using namespace MATH;

/// A dual quaternion can handle rotations and translations. Contains 8 floats

namespace MATHEX {
	union DualQuat {
	private:
		float  dq[8]; /// The DQ is the size of 2 Quaternions = 8 floats
	public:
		struct {
			float real; 
			float e23;  /// This is like -i for a regular quaternion. Squares to -1
			float e31;  /// -j
			float e12;  /// -k
			float e01;  /// Every term with a zero in it squares to zero. It's how translations are handled. 
			float e02;
			float e03;
			float e0123;
		};

		struct {
			Quaternion oppositeRotation; // real, e23, e31 and e12 are the same as w, -i, -j and -k in a regular quaternion. 
			float dx, dy, dz, screw;     // Translations are encoded in dx, dy and dz. The last element combines rotation and translation just like a screw motion
		};

		inline void set(float real_, float e23_, float e31_, float e12_, float e01_, float e02_, float e03_, float e0123_){
			real = real_; e23 = e23_; e31 = e31_; e12 = e12_; e01 = e01_; e02 = e02_; e03 = e03_; e0123 = e0123_;
		}

		/// This is the unit dual quaternion. Doesn't rotate or translate
		/// It is literally just the number 1
		DualQuat() {
			set(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		}

		DualQuat(float real_, float e23_, float e31_, float e12_, float e01_, float e02_, float e03_, float e0123_) {
			set(real_, e23_, e31_, e12_, e01_, e02_, e03_, e0123_);
		}

		
		DualQuat(float angle, const Vec3 &axis, const Vec3 & translation){
			
			Vec3 rotationAxis = VMath::normalize(axis);
			float theta = angle * DEGREES_TO_RADIANS;
			float cosVal = cos(theta / 2.0f);
			float sinVal = sin(theta / 2.0f);

			real = cosVal;
			e23 = rotationAxis.x * sinVal;
			e31 = rotationAxis.y * sinVal;;
			e12 = rotationAxis.z * sinVal;;
			e01 = translation.x / 2.0f;
			e02 = translation.y / 2.0f;
			e03 = translation.z / 2.0f;
			e0123 = 0.0f;
		}

		/// A copy constructor
		inline DualQuat(const DualQuat& dq) {
			set(dq.real, dq.e23, dq.e31, dq.e12, dq.e01, dq.e02, dq.e03, dq.e0123);
		}

		/// An assignment operator   
		inline DualQuat& operator = (const DualQuat& dq) {
			set(dq.real, dq.e23, dq.e31, dq.e12, dq.e01, dq.e02, dq.e03, dq.e0123);
			return *this;
		}

		/// Now I can use the structure itself as an array.
		/// When overloading the [] operator you need to declare one
		/// to read the array and one to write to the array. 
		///  Returns a const - the rvalue
		inline const float operator [] (int index) const {
			return *(dq + index);
		}

		/// This one is for writing to the structure as if where an array 
		/// it returns a modifiable lvalue
		inline float& operator [] (int index) {
			return *(dq + index);
		}

		inline const DualQuat operator * (float c) const {
			DualQuat result;
			result.real = real * c;
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
			result.real = real + dq.real;
			result.e23 = e23 + dq.e23;
			result.e31 = e31 + dq.e31;
			result.e12 = e12 + dq.e12;
			result.e01 = e01 + dq.e01;
			result.e02 = e02 + dq.e02;
			result.e03 = e03 + dq.e03;
			result.e0123 = e0123 + dq.e0123;
			return result;
		}

		// Add to the real part of the dual quaternion
		inline const DualQuat operator + (float w_) const {
			DualQuat result;
			result.real = real + w_;
			result.e23 = e23;
			result.e31 = e31;
			result.e12 = e12;
			result.e01 = e01;
			result.e02 = e02;
			result.e03 = e03;
			result.e0123 = e0123;
			return result;
		}

		// Add the other way around too
		friend const DualQuat operator + (const float w_, const DualQuat& dq) {
			return dq + w_;
		}

		inline const DualQuat operator - (const DualQuat& dq) const {
			DualQuat result;
			result.real = real - dq.real;
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
			printf("real = %1.4f e23 = %1.4f e31 = %1.4f e12 = %1.4f e01 = %1.4f e02 = %1.4f e03 = %1.4f e0123 = %1.4f\n", 
				real, e23, e31, e12, e01, e02, e03, e0123);
		}

	};
}

#endif

/*** Note 1. Umer Noor Sept 2024
REFERENCE: 2023 GDC Talks by Hamish Todd:
		   Math in Game Development Summit: A Visual Guide to Quaternions and Dual Quaternions
		   https://www.gdcvault.com/play/1029233/Math-in-Game-Development-Summit
           Math in Game Development Summit: Quaternions to Homogeneous Points, Lines, and Planes
		   https://www.gdcvault.com/play/1029237
		   
		   and great stuff at bivector.net like this:
		   Dual Quaternions Demystified: https://www.youtube.com/watch?v=ichOiuBoBoQ&t=1s

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