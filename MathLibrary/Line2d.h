#ifndef LINE2D_H
#define LINE2D_H
#include <cmath>    // Used for fabs
#include <string>   // Used for stringing exception messages together
#include <Vector.h> // Used for VERY_SMALL

// Remember y = mx + b from high school (secondary school for me)
// m is the slope and b is the y-intercept
//  
//    y   
//     |   / slope m = rise / run
//     |  /
//     | /
//     |/ <-- y-intecept b
//     / 
//    /|_______________ 
//                      x
//
// The problem with y = mx + b, is how do you represent a line going straight up? 
// A rise of infinity and a run of zero?
// 
// A more general way to represent a line is to rearrange the equation so that everything is on the left side
// y = mx + b
// -mx + y - b = 0
// and then rename the coefficients as a, b, and c to get the general equation of a line
// ax + by + c = 0

namespace MATHEX {
	union Line2d {
	public:
		struct {
			float x; // ax +
			float y; // by +
			float c; // c  = 0
		};
		// The line equation can also be represented using geometric numbers
		// ax  + by  + c   = 0
		// ae1 + be2 + ce0
		// where e1 and e2 square to one. And e0 squares to zero
		// A line that goes through the origin would only have e1 and e2 components
		// Only when you shift the line do you see e0 in action (just like the y-intercept term b)
		// REFERENCE: https://bivector.net/PROJECTIVE_GEOMETRIC_ALGEBRA.pdf
		struct {
			float e1; // ae1 +
			float e2; // be2 +
			float e0; // ce0
		};

		inline void set(float x_, float y_, float c_) {
			x = x_;
			y = y_;
			c = c_;
		}

		// Our default constructor builds a line x = 0
		inline Line2d() {
			set(1.0f, 0.0f, 0.0f);
		}

		inline Line2d(float x_, float y_, float c_) {
			set(x_, y_, c_);
		}

		/// An assignment operator   
		inline Line2d& operator = (const Line2d& p) {
			set(p.x, p.y, p.c);
			return *this;
		}

		// Multiply a line by a scalar
		inline const Line2d operator * (const float scalar) const {
			Line2d result;
			result.x = x * scalar;
			result.y = y * scalar;
			result.c = c * scalar;
			return result;
		}

		// Multiply the other way around too
		friend inline const Line2d operator * (const float scalar, const Line2d& l) {
			return l * scalar;
		}

		// Multiply itself by a scalar
		inline Line2d& operator *= (const float scalar) {
			x *= scalar;
			y *= scalar;
			c *= scalar;
			return *this;
		}

		// Divide a line by a scalar
		inline const Line2d operator / (const float scalar) const {
#ifdef _DEBUG 
			if (std::fabs(scalar) < VERY_SMALL) {
				std::string errorMsg = __FILE__ + __LINE__;
				throw errorMsg.append(": Divide by nearly zero! ");
			}
#endif
			float r = 1.0f / scalar;
			return *this * r;
		}

		// Divide itself by a scalar
		inline Line2d& operator /= (const float scalar) {
#ifdef DEBUG 
			if (std::fabs(c) < VERY_SMALL) {
				std::string errorMsg = __FILE__ + __LINE__;
				throw errorMsg.append(": Divide by nearly zero! ");
			}
#endif
			x /= scalar;
			y /= scalar;
			c /= scalar;
			return *this;
		}

		// Add two lines to get a weighted average
		inline const Line2d operator + (const Line2d& l) const {
			return Line2d(x + l.x, y + l.y, c + l.c);
		}

		// Add a line to itself
		inline Line2d& operator += (const Line2d& l) {
			x += l.x;
			y += l.y;
			c += l.c;
			return *this;
		}

		// Take the negative of a line
		inline const Line2d operator - () const {
			return Line2d(-x, -y, -c);
		}

		// Subract two lines
		inline const Line2d operator - (const Line2d& l) const {
			return Line2d(x - l.x, y - l.y, c - l.c);
		}

		// Subtract a line from itself
		inline Line2d& operator -= (const Line2d& l) {
			x -= l.x;
			y -= l.y;
			c -= l.c;
			return *this;
		}

		void print(const char* comment = nullptr) const {
			if (comment) printf("%s\n", comment);
			printf("%1.8f %1.8f %1.8f\n", x, y, c);
		}

	};
}


#endif
