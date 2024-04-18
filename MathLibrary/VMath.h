#ifndef VMATH_H
#define VMATH_H
#include <cmath>
#include <iostream>
#include <string>
#include "Vector.h"

namespace MATH {
	
	class VMath {

	public:
		/// Calculate the dot product between Vec3s a & b 
		inline static float dot(const Vec3 &a, const Vec3 &b){
			return(a.x * b.x + a.y * b.y + a.z * b.z);
		}

		/// Calculate the dot product between Vec4s a & b 
		inline static float dot(const Vec4 &a, const Vec4 &b){
			return(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
		}

		/// Calulate the cross product with Vec3
		inline static const Vec3 cross(const Vec3 &a, const Vec3 &b){
			return Vec3(a.y * b.z - a.z * b.y,
						a.z * b.x - a.x * b.z,
						a.x * b.y - a.y * b.x);
		}

		/// Calulate the cross product with Vec4
		inline static const Vec4 cross(const Vec4 &a, const Vec4 &b){
			return Vec4(a.y * b.z - a.z * b.y,
						a.z * b.x - a.x * b.z,
						a.x * b.y - a.y * b.x,
						0.0f);
		}

		inline static float mag(const Vec3 &a) {
			return(sqrt(a.x * a.x + a.y * a.y + a.z * a.z));
		}


		///  Angle-Axis rotation
		static Vec3 rotate(const Vec3 &n, float theta, const Vec3 &v) {
			return v * cos(theta) + VMath::dot(v, n) * n * (1.0f - cos(theta)) + VMath::cross(n, v) * sin(theta);
		}

		/// Return a normalized Vec3
		static Vec3 normalize(const Vec3 &a){
			float magnitude = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
#ifdef _DEBUG  /// If in debug mode let's worry about divide by zero or nearly zero!!! 	
			if (magnitude < VERY_SMALL) {
				std::string errorMsg = __FILE__ + __LINE__;
				throw errorMsg.append(": Divide by nearly zero! ");
			}
#endif
			return Vec3(a.x / magnitude, a.y / magnitude, a.z / magnitude);
		}
	
		/// Reflect off a normal  
		static Vec3 reflect(const Vec3 &v, const Vec3 &n){
			float lamda = -2.0f * VMath::dot(n, v);
			return v + lamda * n;
		}

		/// Get the distance between two Vec3s 
		static float distance(const Vec3 &a, const Vec3 &b){
			Vec3 r  = a - b;
			return(mag(r));
		}

		/// This is a basic Linear Interpolation function.  It takes v1 and moves it 
		/// to v2 in a linear fashion based on the value of t which goes from 
		/// 0.0 - 1.0.  This is a simple example of a parametric equation. The parameter
		/// the variable t can mean time but does not need be
		static Vec3 lerp(const Vec3 &v1, const Vec3 &v2, float t) {
			return (v1 + t * (v2 - v1));

		}

		static Vec4 inverse(const Vec4& v) {
			// Flip all the signs, except for w
			return Vec4(-v.x, -v.y, -v.z, v.w);
		}

		/// Return a Vec4 with the w component divided out
		static Vec4 perspectiveDivide(const Vec4& v) {
#ifdef _DEBUG  /// If in debug mode let's worry about divide by zero or nearly zero!!! 	
			if (fabs(v.w) < VERY_SMALL) {
				std::string errorMsg = __FILE__ + __LINE__;
				throw errorMsg.append(": Divide by nearly zero! ");
			}
#endif
			return Vec4(v.x / v.w, v.y / v.w, v.z / v.w, 1.0f);
		}
	};

}

#endif