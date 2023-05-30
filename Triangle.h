/// Umer Noor 2023
/// Represent a triangle by the three vertex positions a, b, c
/// REFERENCE: Real Time Collision Detection by Ericson
/// Ericson says barycentric coordinates are the way to go (u, v, w)

#ifndef TRIANGLE_H
#define TRIANGLE_H
#include "Shape.h"
#include "VMath.h"
#include <array>

namespace GEOMETRY {
	// REFERENCE: Ch. 3 Real Time Collision Detection by Ericson
	// Barycentric coords are handy for ray-triangle intersection and many other things...
	struct BarycentricCoords {
		float u = 0.0f;
		float v = 0.0f;
		float w = 0.0f;
	};

	struct Triangle : public Shape {
		// I'm imagining these as anti-clockwise winding
		// TODO: Does that matter?
		MATH::Vec3 a;
		MATH::Vec3 b;
		MATH::Vec3 c;

		Triangle() {
			// Make a default triangle with normal along z axis
			a.set(-1.0f, -1.0f, 0.0f);
			b.set(1.0f, -1.0f, 0.0f);
			c.set(1.0f, 1.0f, 0.0f);
			generateVerticesAndNormals();
			StoreMeshData(GL_TRIANGLES);
		}

		Triangle(MATH::Vec3 a_, MATH::Vec3 b_, MATH::Vec3 c_) {
			set(a_, b_, c_);
		}

		void set(MATH::Vec3 a_, MATH::Vec3 b_, MATH::Vec3 c_) {
			a = a_;
			b = b_;
			c = c_;
			generateVerticesAndNormals();
			StoreMeshData(GL_TRIANGLES);
		}
		void generateVerticesAndNormals() override;

		/// REFERENCE: Ch. 5 Real Time Collision Detection by Ericson
		RayIntersectionInfo rayIntersectionInfo(const Ray& ray) const override;

		// Calc normal assuming a, b, c have CCW winding order
		MATH::Vec3 calcNormal() const { return MATH::VMath::normalize(MATH::VMath::cross(b - a, c - b)); }

		/// REFERENCE: Ch. 3 Real Time Collision Detection by Ericson
		// Compute barycentric coordinates (u, v, w) for
		// point p with respect to triangle (a, b, c)
		BarycentricCoords calcBarycentric(const MATH::Vec3& p) const;
		// Test if point p is contained in triangle (a, b, c)
		bool isPointInTriangle(const MATH::Vec3& p) const;
	};



}
#endif 
