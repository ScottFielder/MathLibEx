#ifndef POINT2D_H
#define POINT2D_H

// The flatlanders that live in a 2D plane have no idea what life is like in 3D
// They guess this imagined third dimension is perpendicular to their familiar x and y directions
// They grasp at straws and decide to call this magical third dimension "w"

namespace MATHEX {
	union Point2d {
	public:
		struct {
			float x; // x and y look familiar
			float y; 
			float w; // w is the projection into the third dimension for our flatlanders
		};
		// Points are also the intersection of two 2D lines written as 
		// ay  + bx  + c   = 0
		// ae1 + be2 + ce0 = 0
		// where e1 and e2 square to one. And e0 squares to zero
		// REFERENCE: https://bivector.net/PROJECTIVE_GEOMETRIC_ALGEBRA.pdf
		struct {
			float e20; // Anything with a zero squares to zero
			float e01;
			float e12; // e12 squares to -1, as e1212 = -e1221 = -e11 = -1
		};

		inline void set(float x_, float y_, float w_) {
			x = x_; y = y_; w = w_;
		}

		// Our default constructor builds a point at the origin
		// [0, 0] is written as e12
		inline Point2d() {
			set(0.0f, 0.0f, 1.0f);
		}
	};


#endif // !POINT2D_H

