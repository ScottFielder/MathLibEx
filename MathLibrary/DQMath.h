#ifndef DQMATH_H
#define DQMATH_H
#include <Vector.h>
#include "QMath.h"
#include "MMath.h"
#include "DualQuat.h"
#include "GeometricProduct.h"
#include "Meet.h"
#include "PMath.h"
#include "Join.h"
#include "Dot.h"

namespace MATHEX {

	struct DQMath {

		/// Flip the sign on the axis of rotation and translation bivectors
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
			result.real = rotation.w;
			result.e23 = -rotation.e32;
			result.e31 = -rotation.e13;
			result.e12 = -rotation.e21;
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
			result.real = 1.0f;
			result.e23 = 0.0f; 
			result.e31 = 0.0f; 
			result.e12 = 0.0f;
			result.e01 = -translation.x / 2.0f;
			result.e02 = -translation.y / 2.0f;
			result.e03 = -translation.z / 2.0f;
			result.e0123 = 0.0f;
			return result;
		}

		// TODO (UN): Why do the eZero and eOneTwoThree do the magic?
		//	Reference: https://www.youtube.com/watch?v=2DgxeizE3E8	New Hope I
		/// Return a pure translation dual quaternion using a distance and a Dual Quat line
		static const DualQuat translateAlongLine(float dist, const DualQuat& line) {
			Plane eZero(0.0f, 0.0f, 0.0f, 1.0f);
			MATH::Vec4 eOneTwoThree(0.0f, 0.0f, 0.0f, 1.0f);
			// DualQuat() is just the number one
			// UN - I needed a plus here rather than a minus in the video
			return DualQuat() + eZero * (normalize(line) * dist / 2.0f) * eOneTwoThree;
		}

		/// Return just the translation parts of a dual quaternion and the screwiness
		static const DualQuat translate(const DualQuat& dq) {
			DualQuat result;
			result.real = 1.0f;
			result.e23 = 0.0f;
			result.e31 = 0.0f;
			result.e12 = 0.0f;
			result.e01 = dq.e01;
			result.e02 = dq.e02;
			result.e03 = dq.e03;
			result.e0123 = dq.e0123;
			return result;
		}

		/// A rigid transform is a rotate, a translate, or a combination of both.
		/// REFERENCE: https://bivector.net/PROJECTIVE_GEOMETRIC_ALGEBRA.pdf
		static const MATH::Vec4 rigidTransformation(const DualQuat& dq, const MATH::Vec4& p){
			// The famous sandwich
			Flector result = (dq * p * inverse(dq));
			return result.point;
		}

		static const MATH::Quaternion getRotation(const DualQuat& dq) {
			// Find the rotation using the first four elements of the dual quaternion
			MATH::Quaternion rot;
			rot.w   =  dq.real;
			rot.e32 = -dq.e23;
			rot.e13 = -dq.e31;
			rot.e21 = -dq.e12;
			return rot;
		}

		// Return a pure rotation dual quaternion from inside a general dual quaternion (that could have translation too)
		static const DualQuat getRotationDualQuat(const DualQuat& dq) {
			// Rotation is always the first four elements
			// even if the original transform is, for example, T * R or R * T
			// Not so easy for translations as you'll see in getTranslation
			DualQuat result;
			result.real  = dq.real;
			result.e23   = dq.e23;
			result.e31   = dq.e31;
			result.e12   = dq.e12;
			result.e01   = 0.0f;
			result.e02   = 0.0f;
			result.e03   = 0.0f;
			result.e0123 = 0.0f;
			return result;
		}

		// Return a pure translation dual quaternion from inside a general dual quaternion (that could have rotation too)
		static const DualQuat getTranslationDualQuat(const DualQuat& dq) {
			return translate(getTranslation(dq));
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
			// Remember a translation dual quat is 1 - deltaX/2 e01 - deltaY/2 e02 - deltaZ/2 e03
			// So we need minus signs for the next step
			MATH::Vec3 translation(-transformed.e01, -transformed.e02, -transformed.e03);
			return translation;
		}


		// Slerp from one translation and orientation to another translation and orientation
		// Just like the regular quaternion slerp, but now we can include position too!
		// EXAMPLE:   https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Dual_quat_slerp.pdf
		// IN ACTION: https://youtu.be/deX-1AAbifA
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
			return fabs(dq.real);
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
			result.real = 0.0f;
			result.e23 = dq.e23;
			result.e31 = dq.e31;
			result.e12 = dq.e12;
			result.e01 = dq.e01;
			result.e02 = dq.e02;
			result.e03 = dq.e03;
			result.e0123 = 0.0f;
			return result;
		}

		// Return magnitude of the rotational part or the infinite part
		static const float mag(const DualQuat& dq) {
			// Figure out whether this is a pure rotation or not
			float quatMag = MATH::QMath::magnitude(getRotation(dq));
			if (quatMag < VERY_SMALL) {
				// return infinite mag instead here
				float infiniteMag = sqrt(dq.e01 * dq.e01 + dq.e02 * dq.e02 + dq.e03 * dq.e03 +
					dq.e0123 * dq.e0123);
				return infiniteMag;
			}
			// Otherwise just use the quaternion magnitude
			return quatMag;
		}

		// Divide by the magnitude of the rotational part or the infinite part
		static const DualQuat normalize(const DualQuat& dq)
		{
			return dq / mag(dq);
		}

		// Oriented distance between a point and a line (sign tells you which side of the line)
		// EXAMPLE: https://github.com/ScottFielder/MathLibrary/blob/master/Notes/Oriented_distance_point_and_line.pdf
		static const float orientedDist(const MATH::Vec4& v, const DualQuat& q) {
			// First normalize the point and plane 
			MATH::Vec4  vNormalized = VMath::perspectiveDivide(v);
			DualQuat    qNormalized = normalize(q);
			// Then use the formula for the oriented distance from https://bivector.net/3DPGA.pdf
			// TODO (UN): Is the magnitude in the formula page just one of the e1, e2, or e3 parts??
			Plane plane = join(vNormalized, qNormalized); 
			// TODO (UN): This would be nice to have in PMath. 
			float dist = sqrt(plane.e1 * plane.e1 + plane.e2 * plane.e2 + plane.e3 * plane.e3);
			// This is oriented distance, so choose the sign on e1, e2 or e3
			if(fabs(plane.e1) > VERY_SMALL) {
				return plane.e1 > 0.0f ? dist : -dist;
			}
			else if(fabs(plane.e2) > VERY_SMALL) {
				return plane.e2 > 0.0f ? dist : -dist;
			}
			else {
				return plane.e3 > 0.0f ? dist : -dist;
			}
		}

		// Building a lookAt DualQuat that you can convert into the view matrix
		// Eye position has to be a point, but at and up can be points or directions
		// That's why I bring in the eye as a Vec3 (w is set to 1), but others as Vec4 (w can be 1 or 0)
		// REFERENCE: https://observablehq.com/@enkimute/glu-lookat-in-3d-pga
		// For some reason I had to swap the meaning of q and p compared to article
		// Tested Dec 13 2024 UN
		static const DualQuat lookAt(const Vec3& eye, const Vec4& at, const Vec4& up) {
			Vec3 originalEye;                                // Camera starts at the origin
			Vec4 originalAt = Vec4(0.0f, 0.0f, -1.0f, 0.0f); // Looking down the -z axis
			Vec4 originalUp = Vec4(0.0f, 1.0f, 0.0f, 0.0f);  // With up along the y axis

			DualQuat result;
			// First iteration where P and Q are points to align the positions
			result = motorAtoB(Vec4(eye), Vec4(originalEye));

			// Second iteration where P and Q are lines to align the targets
			DualQuat P = Vec4(originalEye) & rigidTransformation(result, at);
			DualQuat Q = Vec4(originalEye) & originalAt;
			result = motorAtoB(normalize(P), normalize(Q)) * result;

			// Third and final iteration where P and Q are planes to align the up directions
			Plane Pplane = Q & rigidTransformation(result, up);
			Plane Qplane = Q & originalUp;
			result = motorAtoB(PMath::normalize(Pplane), PMath::normalize(Qplane)) * result;
			return result;
		}

		// The squareRoot is defined as an even mixture with the number 1, then normalized
		// That gives you a transform halfway from start to end
		// I named it squareRoot to avoid any clashes with std::sqrt in user code
		static const DualQuat squareRoot(const DualQuat& dq) {
			return normalize(1.0f + dq);
		}

		// A motor that takes a point, line, plane a to a point, line, plane b is:
		// motor = sqrt(b/a)
		// REFERENCE: https://observablehq.com/@enkimute/glu-lookat-in-3d-pga
		static const DualQuat motorAtoB(const Plane& a, const Plane& b) {
			return squareRoot(b / a);
		}

		static const DualQuat motorAtoB(const DualQuat& a, const DualQuat& b) {
			return squareRoot(b / a);
		}

		static const DualQuat motorAtoB(const Vec4& a, const Vec4& b) {
			return squareRoot(b / a);
		}

		// Projecting a point onto a line
		// REFERENCE: Equations from the 58:51 min mark here https://www.gdcvault.com/play/1029237/
		// UN - Tested 2025-02-24 for Sphere-Triangle collision assignment in Game Physics 2
		static const MATH::Vec4 project(const MATH::Vec4& point, const DualQuat& line) {
			return (line | point) ^ line;
		}

		// Projecting a line onto a point
		// UN - Tested 2025-03-10 for my Voronoi madness in QuadMath::closestPointOnQuad
		static const DualQuat project(const DualQuat& line, const MATH::Vec4& point) {
			// Ooh, looks like we might actually need the geometric product here compared to the project point onto line
			return (point | line) * point;
		}
	};
}
#endif