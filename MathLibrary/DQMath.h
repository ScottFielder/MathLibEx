#ifndef DQMATH_H
#define DQMATH_H
#include "DualQuat.h"
#include "Vector.h"
#include "Quaternion.h"

namespace MATHEX {

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

		/// Return a pure rotation dual quaternion
		static const DualQuat rotate(const Quaternion& rotation) {
			// No translation, but use the quaternion to build the first four floats
			// Remember e23 = -i, e31 = -j, e12 = -k
			DualQuat result;
			result.w = rotation.w;
			result.e23 = -rotation.ijk.x;
			result.e31 = -rotation.ijk.y;
			result.e12 = -rotation.ijk.z;
			result.e01 = 0.0f;
			result.e02 = 0.0f;
			result.e03 = 0.0f;
			result.e0123 = 0.0f;
			return result;
		}

		/// Return a pure translation dual quaternion
		static const DualQuat translate(const Vec3& translation) {
			// No rotation, but set the last four floats to be half the translation
			DualQuat result;
			result.w = 1.0f;
			result.e23 = 0.0f; 
			result.e31 = 0.0f; 
			result.e12 = 0.0f;
			result.e01 = translation.x / 2.0f;
			result.e02 = translation.y / 2.0f;
			result.e03 = translation.z / 2.0f;
			result.e0123 = 0.0f;
			return result;
		}

		/// A rigid transform is a rotate and/or translate. Dual quats can't scale reliably
		/// REFERENCE: https://bivector.net/PROJECTIVE_GEOMETRIC_ALGEBRA.pdf
		static const Vec4 rigidTransformation(const DualQuat& dq, const Vec4& p){
			// Turns out the translation part is 1 - delta/2 * (e01, 2, 3) rather that 1 + delta/2 * (e01, 2, 3)
			DualQuat fix = dq;
			fix.e01 *= -1.0f;
			fix.e02 *= -1.0f;
			fix.e03 *= -1.0f;
			// Note that the sandwich needs the inverse rather than the congujate in geometric algebra vs traditional dual quat math
			return (fix * p * inverse(fix)).point;
		}

	};
}
#endif