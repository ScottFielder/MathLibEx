#include "DQMath.h"
#include <MMath.h>
#include <QMath.h>
using namespace MATHEX;

DualQuat DQMath::conjugate(const DualQuat& dq)
{
	// Flip the sign on all the elements that square to - 1
	DualQuat result = dq;
	result.e23 *= -1.0f;
	result.e31 *= -1.0f;
	result.e12 *= -1.0f;
	return result;
}

DualQuat DQMath::inverse(const DualQuat& dq)
{
	// Flip the sign on the axis of rotation and translation bivectors
	// TODO: Not sure if I need to flip e0123 as well?
	DualQuat result = dq;
	result.e23 *= -1.0f;
	result.e31 *= -1.0f;
	result.e12 *= -1.0f;
	result.e01 *= -1.0f;
	result.e02 *= -1.0f;
	result.e03 *= -1.0f;
	return result;
}

Vec3 DQMath::rigidTransformation(const DualQuat& dq, const Vec3& v)
{
	// Make a dual quaternion version of the vector
	DualQuat vDq;
	vDq.e01 = v.x;
	vDq.e02 = v.y;
	vDq.e03 = v.z;
	// Transform using the sandwich
	DualQuat transfomation = dq * vDq * conjugate(dq);
	// Grab the vector from the translation part of the dual quaternion
	return Vec3(transfomation.e01, transfomation.e02, transfomation.e03);
}

Matrix4 DQMath::toMatrix4(const DualQuat& dq)
{
	return MMath::translate(getTranslation(dq)) * MMath::toMatrix4(getRotation(dq));
}

Quaternion DQMath::getRotation(const DualQuat& dq)
{
	// Find the rotation using the first four elements
	Quaternion rot;
	rot.w = dq.w;
	rot.ijk.x = -dq.e23;
	rot.ijk.y = -dq.e31;
	rot.ijk.z = -dq.e12;
	return rot;
}

Vec3 DQMath::getTranslation(const DualQuat& dq)
{
	// Find translation from the last bit of the dual quaternion
	DualQuat dualPart(Vec3(2.0f * dq.e01, 2.0f * dq.e02, 2.0f * dq.e03));
	DualQuat rotDq(getRotation(dq));
	// Rebuild the translation using t * r.conjugate * 2
	DualQuat transformed = dualPart * DQMath::conjugate(rotDq) * 2.0f;
	Vec3 translation(transformed.e01, transformed.e02, transformed.e03);
	return translation;
}

DualQuat DQMath::slerp(const DualQuat& start, const DualQuat& end, float t)
{
	// The slerp is written as 
	// exp(t * log(end/start)) * start
	// Let's turn this into something human readable
	DualQuat endOverStart = end * inverse(start);

	// The log brings out the rotation axis & angle/2 
	// and the translation axis and displacement/2
	Vec3 translation = getTranslation(endOverStart);
	Quaternion rotation = getRotation(endOverStart);
	float angle = acos(rotation.w) * 2.0f;
	Vec3 rotAxis(0, 1, 0); // pick a random rotn axis just in case angle is zero
	if (fabs(angle) > VERY_SMALL) {
		rotAxis = VMath::normalize(rotation.ijk);
	}
	// Now multiply the angle and translation by t
	translation *= t;
	angle *= t;

	// The exp means turn it all back into a dual quaternion
	DualQuat transformedDqRot(QMath::angleAxisRotation(angle * RADIANS_TO_DEGREES, rotAxis));
	DualQuat transformedDqTra(translation);
	DualQuat transformedDq = transformedDqTra * transformedDqRot;

	// Lastly multiply it with the start
	return transformedDq * start;
}
