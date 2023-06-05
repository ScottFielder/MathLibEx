#ifndef PMATH_H
#define PMATH_H
#include "Plane.h"
#include <cmath>
namespace MATH {
	class PMath {
	public:
		static Plane normalize(const Plane &p){
			float mag = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
			return Plane(p.x / mag, p.y / mag, p.z / mag, p.d / mag);
		}

		/// Get the distance from a point (Vec3) to a plane
		static float distance(const Vec3 &q, const Plane &p){
			return VMath::dot(q,p.n) - p.d;
		}

		static Vec3 reflect(const Vec3 &v, const Plane &p){
			return v - (2.0f * VMath::dot(p.n, v)) * p.n;
		}
	};
}

#endif
