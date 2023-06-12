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

		/// Get the distance form a point (Vec3) to a plane
		static float distance(const Vec3 &v, const Plane &p){
			return (p.x*v.x + p.y*v.y + p.z*v.z + p.d);
		}

		static Vec3 reflect(const Vec3 &v, const Plane &p){
			return v - (2.0f * VMath::dot(p.n, v)) * p.n;
		}

		// It's wild, but the inverse of a plane, is the same plane?
		// Planar reflections square to 1, so I guess so
		static const Plane inverse(const Plane& p) {
			return p;
		}

		// Return plane in between the two arguments by distance and angle
		// Be careful, you need to normalize the plane afterwards 
		// to convert to Standard Euclidean definition 
		static const Plane midPlane(const Plane& p1, const Plane& p2) {
			return normalize(p1) + normalize(p2);
		}
	};
}

#endif
