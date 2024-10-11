#ifndef POINT2D_H
#define POINT2D_H
#include <cmath>    // Used for fabs
#include <string>   // Used for stringing exception messages together
#include <Vector.h> // Used for VERY_SMALL

// Flatlanders live in a 2D plane have no idea what life is like in 3D
// They guess this imagined third dimension is perpendicular to their familiar x and y directions
// They grasp at straws and decide to call this mysterious third dimension "w"

namespace MATHEX {
	union Point2d {
	public:
		struct {
			float x; // x and y look familiar
			float y;
			float w; // w is the projection into the third dimension for our flatlanders
		};
		// Points are also the intersection of two 2D lines written as 
		// ax  + by  + c   = 0
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

		inline Point2d(float x_, float y_, float w_) {
			set(x_, y_, w_);
		}

		/// A copy constructor
		inline Point2d(const Point2d& p) {
			set(p.x, p.y, p.w);
		}

		/// An assignment operator   
		inline Point2d& operator = (const Point2d& p) {
			set(p.x, p.y, p.w);
			return *this;
		}

		// Multiply a point by a scalar
		inline const Point2d operator * (const float c) const {
			Point2d result;
			result.x = x * c;
			result.y = y * c;
			result.w = w * c;
			return result;
		}

		// Multiply the other way around too
		friend inline const Point2d operator * (const float c, const Point2d& p) {
			return p * c;
		}

		// Multiply itself by a scalar
		inline Point2d& operator *= (const float c) {
			x *= c;
			y *= c;
			w *= c;
			return *this;
		}

		// Divide a point by a scalar
		inline const Point2d operator / (const float c) const {
#ifdef _DEBUG 
			if (std::fabs(c) < VERY_SMALL) {
				std::string errorMsg = __FILE__ + __LINE__;
				throw errorMsg.append(": Divide by nearly zero! ");
			}
#endif
			float r = 1.0f / c;
			return *this * r;
		}

		// Divide itself by a scalar
		inline Point2d& operator /= (const float c) {
#ifdef DEBUG 
			if (std::fabs(c) < VERY_SMALL) {
				std::string errorMsg = __FILE__ + __LINE__;
				throw errorMsg.append(": Divide by nearly zero! ");
			}
#endif
			x /= c;
			y /= c;
			w /= c;
			return *this;
		}

		// Add two points
		inline const Point2d operator + (const Point2d& p) const {
			return Point2d(x + p.x, y + p.y, w + p.w);
		}

		// Add a point to itself
		inline Point2d& operator += (const Point2d& p) {
			x += p.x;
			y += p.y;
			w += p.w;
			return *this;
		}

		// Take the negative of a point
		inline const Point2d operator - () const {
			return Point2d(-x, -y, -w);
		}

		// Subract two points
		inline const Point2d operator - (const Point2d& p) const {
			return Point2d(x - p.x, y - p.y, w - p.w);
		}

		// Subtract a point from itself
		inline Point2d& operator -= (const Point2d& p) {
			x -= p.x;
			y -= p.y;
			w -= p.w;
			return *this;
		}

		void print(const char* comment = nullptr) const {
			if (comment) printf("%s\n", comment);
			printf("%1.8f %1.8f %1.8f\n", x, y, w);
		}

	};
}

#endif // !POINT2D_H

