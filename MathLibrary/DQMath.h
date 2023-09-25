#ifndef DQMATH_H
#define DQMATH_H
#include "DualQuat.h"
#include "Vector.h"
#include "QMath.h"
#include "GeometricProduct.h"

namespace MATHEX {

	class DQMath {
	public:

		/// Flip the sign on the axis of rotation and translation bivectors
		/// TODO: Not sure if I need to flip e0123 as well?
 		/// If I do flip, the rigid transform seems to work for slerping
		/// If I don't then dq * dq.inverse = 1 which is the definition
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
		static const DualQuat rotate(const MATH::Quaternion& rotation) {
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
		static const DualQuat translate(const MATH::Vec3& translation) {
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

		/// Return just the translation parts of a dual quaternion and the screwiness
		static const DualQuat translate(const DualQuat& dq) {
			DualQuat result;
			result.w = 1.0f;
			result.e23 = 0.0f;
			result.e31 = 0.0f;
			result.e12 = 0.0f;
			result.e01 = dq.e01;
			result.e02 = dq.e02;
			result.e03 = dq.e03;
			result.e0123 = dq.e0123;
			return result;
		}

		/// A rigid transform is a rotate and/or translate. Dual quats can't scale reliably
		/// REFERENCE: https://bivector.net/PROJECTIVE_GEOMETRIC_ALGEBRA.pdf
		static const MATH::Vec4 rigidTransformation(const DualQuat& dq, const MATH::Vec4& p){
			// Turns out the translation part is 1 - delta/2 * (e01, 2, 3) rather that 1 + delta/2 * (e01, 2, 3)
			DualQuat fix = dq;
			fix.e01 *= -1.0f;
			fix.e02 *= -1.0f;
			fix.e03 *= -1.0f;
			// Note that the sandwich needs the inverse rather than the congujate in geometric algebra vs traditional dual quat math
			// TODO: For some reason, I need to flip the sign on the e0123 part when doing the inverse
			// to have a pure point flector result. Otherwise, I need to combine the plane and the point in the flector
			// somehow to get the right answer. Need to figure this out...
			// Flector result = (fix * p * inverse(fix));
			DualQuat inverseFix = inverse(fix);
			inverseFix.e0123 *= -1.0f;
			Flector result = (fix * p * inverseFix);
			return result.point;
		}

		static const MATH::Quaternion getRotation(const DualQuat& dq)
		{
			// Find the rotation using the first four elements of the dual quaternion
			MATH::Quaternion rot;
			rot.w = dq.w;
			rot.ijk.x = -dq.e23;
			rot.ijk.y = -dq.e31;
			rot.ijk.z = -dq.e12;
			return rot;
		}

		static const MATH::Vec3 getTranslation(const DualQuat& dq)
		{
			// We are doing old school dual quaternion math here
			// Where a dual quaternion is literally made up of two quaternions
			// A real one (that holds the rotation), and a dual one (that encodes translation)
			// So dual quaternion = q_rot + dualBasis * q_t * q_rot
			// Find translation from the last bit of the dual quaternion
			DualQuat dualPart = translate(dq);
			DualQuat realPart = rotate(getRotation(dq));
			// Rebuild the translation using t * r.conjugate * 2
			// To conjugate our dual quaternion, we flip the sign on the axis of the real part 
			realPart.e23 *= -1.0f;
			realPart.e31 *= -1.0f;
			realPart.e12 *= -1.0f;
			DualQuat transformed = dualPart * realPart * 2.0f;
			MATH::Vec3 translation(transformed.e01, transformed.e02, transformed.e03);
			return translation;
		}

		static const MATH::Matrix4 toMatrix4(const DualQuat& dq)
		{
			// Old school dual quaternion math gives us a nice way of building a transformation matrix
			return MATH::MMath::translate(getTranslation(dq)) * MATH::MMath::toMatrix4(getRotation(dq));
		}

		/// Slerp from one translation and orientation to another translation and orientation
		/// Just like the regular quaternion slerp, but now we can include position too!
		static const DualQuat slerp(const DualQuat& start, const DualQuat& end, float t) {
			// The slerp is written as 
			// exp(t * log(end/start)) * start
			// Let's turn this into something human readable
			DualQuat endOverStart = end * inverse(start);

			// The log brings out the rotation axis & angle/2 
			// and the translation axis and displacement/2
			MATH::Vec3 translation = getTranslation(endOverStart);
			MATH::Quaternion rotation = getRotation(endOverStart);
			float angle = acos(rotation.w) * 2.0f;
			MATH::Vec3 rotAxis(0, 1, 0); // pick a random rotn axis just in case angle is zero
			if (fabs(angle) > VERY_SMALL) {
				rotAxis = MATH::VMath::normalize(rotation.ijk);
			}
			// Now multiply the angle and translation by t
			translation *= t;
			angle *= t;

			// The exp means turn it all back into a dual quaternion
			DualQuat transformedDqRot = rotate(MATH::QMath::angleAxisRotation(angle * RADIANS_TO_DEGREES, rotAxis));
			DualQuat transformedDqTra = translate(translation);
			DualQuat transformedDq = transformedDqTra * transformedDqRot;

			// Lastly multiply it with the start
			DualQuat result = transformedDq * start;
			return result;
		}

	};
}
#endif