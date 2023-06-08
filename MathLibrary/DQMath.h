#ifndef DQMATH_H
#define DQMATH_H
#include "DualQuat.h"
#include "Vector.h"

namespace MATH {

	class DQMath {
	public:

		/// Flip the sign on the axis of rotation and translation bivectors
		/// TODO: Not sure if I need to flip e0123 as well?
		static const DualQuat inverse(const DualQuat& dq) {
			DualQuat result = dq;
			result.e23 *= -1.0f;
			result.e31 *= -1.0f;
			result.e12 *= -1.0f;
			result.e01 *= -1.0f;
			result.e02 *= -1.0f;
			result.e03 *= -1.0f;
			return result;
		}

	};
}
#endif