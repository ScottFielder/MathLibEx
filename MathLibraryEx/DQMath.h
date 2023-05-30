// 2023 May - Umer Noor
// We want our dual quaternions to do cool things like rotate and translate vectors
// REFERENCE: 2023 GDC Talks by Hamish Todd
//            Find the talks at https://library.humber.ca/atoz_landing/G
#ifndef DQMATH_H
#define DQMATH_H
#include "DualQuat.h"
using namespace MATH;
namespace MATHEX {
	namespace DQMath
	{
		DualQuat conjugate(const DualQuat& dq);
		DualQuat inverse(const DualQuat& dq);
		Vec3 rigidTransformation(const DualQuat& dq, const Vec3& v);
		Matrix4 toMatrix4(const DualQuat& dq);
		Quaternion getRotation(const DualQuat& dq);
		Vec3 getTranslation(const DualQuat& dq);
		DualQuat slerp(const DualQuat& start, const DualQuat& end, float t);
	}
}
#endif
