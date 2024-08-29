#ifndef SPHERE_H
#define SPHERE_H
#include <iostream>
#include "ConstantsConversions.h"
#include <Vector.h>
namespace  MATHEX {
	struct Sphere {
		MATH::Vec3 center;
		float r;

		/// Just a little utility to populate a Sphere
		inline void set(float x_, float y_, float z_, float r_) {
			center.set(x_, y_, z_);
			r = r_;
		}

		Sphere() { 
			center.set(0.0f, 0.0f, 0.0f);
			r = 0.0f;
		}

		Sphere(const MATH::Vec3 &center, const float r_) {
			set(center.x, center.y, center.z, r_);
		}

		Sphere(float x, float y, float z, float r_) {
			set(x, y, z, r_);
		}

		Sphere(const Sphere& s) {
			center = s.center;
			r = s.r;
		}

		/// print the values of the sphere and add a comment if you wish
		void print(const char* comment = nullptr) const {
			if (comment) printf("%s\n", comment);
			printf("%1.4f %1.4f %1.4f %1.4f\n", center.x, center.y, center.z, r);
		}
	};


	

}
#endif