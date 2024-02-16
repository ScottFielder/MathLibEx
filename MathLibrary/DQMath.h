#ifndef DQMATH_H
#define DQMATH_H
#include "DualQuat.h"
#include "Vector.h"
#include "QMath.h"
#include "GeometricProduct.h"
#include "MMath.h"

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

		/// Return a pure rotation dual quaternion. This time, build the quaternion from the argument list
		static const DualQuat rotate(float angleDeg, const MATH::Vec3& axis) {
			MATH::Quaternion quat = MATH::QMath::angleAxisRotation(angleDeg, axis);
			return rotate(quat);
		}

		/// Return a pure translation dual quaternion from a Vec3
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

		// TODO (UN): Why do the eZero and eOneTwoThree do the magic?
		//	Reference: https://www.youtube.com/watch?v=2DgxeizE3E8	New Hope I
		/// Return a pure translation dual quaternion using a distance and a Dual Quat line
		static const DualQuat translateAlongLine(float dist, const DualQuat& line) {
			MATH::Plane eZero(0.0f, 0.0f, 0.0f, 1.0f);
			MATH::Vec4 eOneTwoThree(0.0f, 0.0f, 0.0f, 1.0f);
			return DualQuat() - eZero * (normalize(line) * dist / 2.0f) * eOneTwoThree;
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

		// Return a pure rotation dual quaternion from inside a general dual quaternion (that could have translation too)
		static const DualQuat getRotationDualQuat(const DualQuat& dq) {
			return rotate(getRotation(dq));
		}

		// Return the translation vector from the dual quaternion
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

		// Return a pure translation dual quaternion from inside a general dual quaternion (that could have rotation too)
		static const DualQuat getTranslationDualQuat(const DualQuat& dq) {
			return translate(getTranslation(dq));
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

		// Extract grades out of the dual quaternion
		// grade 0 is the w component
		// grade 2 is the e23, e31, e12 parts
		// grade 2 at infinity is the e01, e02, e03 parts
		// grade 4 is the e0123 part
		// Line part of the dual quaternion is just the two grade 2 parts
		static const float magGrade0(const DualQuat& dq) {
			return fabs(dq.w);
		}

		static const float magGrade2(const DualQuat& dq) {
			return sqrt(dq.e23 * dq.e23 + dq.e31 * dq.e31 + dq.e12 * dq.e12);
		}

		static const float magGrade2Infinity(const DualQuat& dq) {
			return sqrt(dq.e01 * dq.e01 + dq.e02 * dq.e02 + dq.e03 * dq.e03);
		}

		static const float magGrade4Infinity(const DualQuat& dq) {
			return fabs(dq.e0123);
		}

		// WHAT? https://github.com/ScottFielder/MathLibrary/blob/master/Notes/What_is_a_line.pdf
		static const DualQuat extractLine(const DualQuat& dq) {
			DualQuat result;
			result.w = 0.0f;
			result.e23 = dq.e23;
			result.e31 = dq.e31;
			result.e12 = dq.e12;
			result.e01 = dq.e01;
			result.e02 = dq.e02;
			result.e03 = dq.e03;
			result.e0123 = 0.0f;
			return result;
		}

		// TODO (UN)
		// Wonder if it only makes sense to normalize a line rather than the whole dual quat?
		static const DualQuat normalize(const DualQuat& dq)
		{
			// Just care about the line part of the dual quat
			DualQuat line = extractLine(dq);
			// Figure out whether this is a Euclidean line or one in the horizon at infinity
			float mag = magGrade2(line);
			if (mag < VERY_SMALL) {
				mag = magGrade2Infinity(line);
			}
			return line / mag;
		}
	};
}
#endif