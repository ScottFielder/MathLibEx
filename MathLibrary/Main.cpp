/// This is the test bed for my math library - SSF
/// This is currently version 1.40 - SSF May 2024

#include <iostream>
#include <fstream>
#include <algorithm> 
#include <chrono> // for timing code

#include <MMath.h>
#include <QMath.h>
#include <EMath.h>
#include <AAMath.h>
#include "Sphere.h"
#include <Hash.h>

#include "PMath.h"
#include "Quadratic.h"
#include "RMath.h"
#include "DualQuat.h"
#include "Flector.h"
#include "GeometricProduct.h"
#include "DQMath.h"
#include "PoincareDuality.h"
#include "Meet.h"
#include "Join.h"
#include "Dot.h"
#include "Point2d.h"
#include "Triangle.h"
#include "TMath.h"

#include <glm/vec3.hpp> /// glm::vec3
#include <glm/vec4.hpp> /// glm::vec4, glm::ivec4
#include <glm/mat4x4.hpp> /// glm::mat4
#include <glm/gtc/matrix_transform.hpp> /// glm::translate, glm::rotate, glm::scale, glm::perspective
#include <glm/gtc/type_ptr.hpp> /// glm::value_ptr
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/dual_quaternion.hpp> 

#include <glm/gtx/hash.hpp>

/// MathLib tests
void LookAtTest();
void inverseTest();
void UnOrthoTest();
void RotationTest();
void Vec3MultiplyMat4Test();
void multiplyMatrixTest();
void viewportNDCTest();
void moveCopyConstructors();
void rotationIsOrthogonal();
void quaternionTest();
void hashTest();
void determinantTest();
void slerpTest();

/// MathLibEx tests
void dqLookAtTest();
void planeTest();
void QuadraticTest();
void RaySphereTest();
void RayTest();
void dualQuatTest();
void flectorTest();
void intersectionTest();
void poincareDualityTest();
void meetTest();
void joinTest();
void dualQuatSlerpTest();
void rotateTest();
void gradeTest();
void normalizeLineTest();
void translateAlongLineTest();
void rayPlaneTest();
void dotTest();
void dualQuatSlerpVectorTest();
void dualQuatMatrixTest();
void point2dTest();
void triangleTest();
void sphereTest();


/// Utility print() calls for glm to math library format 
void glmPrintM4(glm::mat4  mat, const char* comment = nullptr);
void glmPrintM3(glm::mat3  mat, const char* comment = nullptr);

void glmPrintQ(glm::quat q, const char* comment = nullptr);
void glmPrintDQ(glm::dualquat dq, const char* comment = nullptr);
void glmPrintV3(glm::vec3 v, const char* comment = nullptr);
void glmPrintV4(glm::vec4 v, const char* comment = nullptr);

using namespace MATH;
using namespace MATHEX;
using namespace glm;
using namespace std;


int main(int argc, char* argv[]) {
	dqLookAtTest();
	//LookAtTest();
	//sphereTest();
	//triangleTest();
	//point2dTest();
	//planeTest();
	//QuadraticTest();
	//RaySphereTest();
	//RayTest();
	//dualQuatTest();
	//flectorTest();
	//intersectionTest();
	//poincareDualityTest();
	//meetTest();
	//joinTest();
	//dualQuatSlerpTest();
	//rotateTest();
	//gradeTest();
	//normalizeLineTest();
	//translateAlongLineTest();
	//rayPlaneTest();
	//dotTest();
	//dualQuatSlerpVectorTest();
	//dualQuatMatrixTest();
}

void dqLookAtTest() {
	Vec4 eye = Vec4(0, 0, 20, 1); // camera is 20 units along the z-axis
	Vec4 at =  Vec4(1, 0, 20, 1); // Looking to the right 
	Vec4 up =  Vec4(0, 1, 0, 0);  // Up is along positive y

	DualQuat viewDq1 = DQMath::lookAt(eye, at, up);

	// check against building the view matrix using R^(-1) * T^(-1)
	float rotationAngleDeg = -90.0f;
	Vec3 rotationAxis = Vec3(0.0f, 1.0f, 0.0f);
	Quaternion cameraOrientation = QMath::angleAxisRotation(rotationAngleDeg, rotationAxis);
	DualQuat viewDq2 = DQMath::rotate(QMath::inverse(cameraOrientation)) * DQMath::translate(-eye);

	// And check against the original lookAts
	Matrix4 view = MMath::lookAt(eye, at, up);
	glm::mat4 mt = glm::lookAt(
		glm::vec3(eye.x, eye.y, eye.z),
		glm::vec3(at.x, at.y, at.z),
		glm::vec3(up.x, up.y, up.z));

	printf("************************************************************\n\n");
	printf("First check with eye (0, 0, 20), at (1, 0, 20), up (0, 1, 0)\n\n");
	DQMath::toMatrix4(viewDq1).print("view matrix using DQMath::lookAt");
	DQMath::toMatrix4(viewDq2).print("view matrix using R^(-1) * T^(-1)");
	glmPrintM4(mt, "view matrix using glm::lookAt");
	printf("\n");
	view.print("view matrix using MMath::lookAt");
	printf("************************************************************\n");
	printf("************************************************************\n\n");
	printf("Now check again, but up is along positive z now\nI don't know how to build a R^(-1) * T^(-1) for that, but I'll check the lookAts\n\n");
	up = Vec4(0, 0, 1, 0);
	viewDq1 = DQMath::lookAt(eye, at, up);
	view = MMath::lookAt(eye, at, up);
	mt = glm::lookAt(
		glm::vec3(eye.x, eye.y, eye.z),
		glm::vec3(at.x, at.y, at.z),
		glm::vec3(up.x, up.y, up.z));
	DQMath::toMatrix4(viewDq1).print("view matrix using DQMath::lookAt");
	glmPrintM4(mt, "view matrix using glm::lookAt");
	printf("\n");
	view.print("view matrix using MMath::lookAt");
	printf("************************************************************\n");

}

void sphereTest() {
	// Imagine you are filling up the vertices to make a sphere shape to render to screen
	// That's one of Umer's assignments in Game Physics 3
	// What's the performance difference between doing this problem with or without PGA?

	auto start = chrono::high_resolution_clock::now();
	std::vector<Vec3> verticesWithoutPGA;
	// We need to fill the vertices and normals arrays with the correct data for a sphere
	// deltaTheta governs how many points per ring. Try messing with it
	const float deltaTheta = 3.6f;
	// deltaPhi governs how many rings there are in total. Try messing with it
	const float deltaPhi = 3.6f;
	const float r = 1.0f;
	for (float thetaDeg = 0.0f; thetaDeg <= 360.0f; thetaDeg += deltaTheta)
	{
		// Build a ring
		Vec3 circle(r * sin(thetaDeg * DEGREES_TO_RADIANS), r * cos(thetaDeg * DEGREES_TO_RADIANS), 0.0f);
		for (float phiDeg = 0.0f; phiDeg <= 360.0f; phiDeg += deltaPhi) {
			// Rotate a point in the ring around the y-axis to build a sphere!
			Matrix3 rotationMatrix = MMath::rotate(deltaPhi, Vec3(0.0f, 1.0f, 0.0f));
			circle = rotationMatrix * circle;
			// Push the circle point to our vertices array
			verticesWithoutPGA.push_back(circle);
		}
	}
	auto stop = chrono::high_resolution_clock::now();
	auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
	cout << duration.count() <<" microseconds for a sphere built the usual way with " << verticesWithoutPGA.size() << " points" << endl;


	start = chrono::high_resolution_clock::now();
	std::vector<Vec3> verticesWithPGA;
	float dist = 1.0f;
	float angle = 360.0f;
	DualQuat T(1.0f, 0.0f, 0.0f, 0.0f, dist / 2.0f, 0.0f, 0.0f, 0.0f); // built the translate by hand to see if that speeds anything up

	for (float t1 = 0.0f; t1 < 1.0f; t1 += 0.0101f) {
		DualQuat R1 = DQMath::rotate(angle * t1 / 2.0f, Vec3(0.0f, 0.0f, 1.0f));
		DualQuat R1_times_T; // build out the geometric product by hand just to see if that speeds anything up
		R1_times_T.real = R1.real;
		R1_times_T.e12 = R1.e12;
		R1_times_T.e01 = R1.real * T.e01;
		R1_times_T.e02 = -R1.e12 * T.e01;
		for (float t2 = 0.0f; t2 < 1.0f; t2 += 0.0101f) {

			DualQuat R2 = DQMath::rotate(angle * t2, Vec3(1.0f, 0.0f, 0.0f));
			DualQuat sphereMotor = R2 * R1_times_T;

			Vec4 vertex = DQMath::rigidTransformation(sphereMotor, Vec4(0.0f, 0.0f, 0.0f, 1.0f));
			verticesWithPGA.push_back(vertex);
		}
	}
	stop = chrono::high_resolution_clock::now();
	duration = chrono::duration_cast<chrono::microseconds>(stop - start);
	cout << duration.count() << " microseconds for a sphere built the PGA way with " << verticesWithPGA.size() << " points" << endl;
}

void triangleTest() {
	// Line below shld throw if you uncomment it
	// Triangle t(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(2, 0, 0));

	Triangle tri1(Vec3(0, 0, 0), Vec3(0, 1, 0), Vec3(1, 0, 0));
	tri1.print("Triangle 1");
	TMath::getNormal(tri1).print("Normal of Triangle 1");


	TMath::getPlane(tri1).print("Plane of Triangle 1");
	Plane p(Vec3(0, 0, 0), Vec3(0, 1, 0), Vec3(1, 0, 0));
	p.print("Plane of Triangle 1 SSF");

	Vec3 p1(0, 0, 0);
	p1.print("Point 1");
	Vec3 p2(2, 0, 0);
	p2.print("Point 2");

	if(TMath::isPointInsideTriangle(p1, tri1)){
		printf("Point 1 is in Triangle 1\n");
	}
	else{
		printf("Point 1 is not in Triangle 1\n");
	}

if (TMath::isPointInsideTriangle(p2, tri1)) {
		printf("Point 2 is in Triangle 1\n");
	}
	else {
		printf("Point 2 is not in Triangle 1\n");
	}

}

void point2dTest() {
	Point2d p1;
	p1.print("Point2d p1");
	Point2d p2(1.0f, 2.0f, 1.0f);
	p2.print("Point2d p2(1.0f, 2.0f, 1.0f)");
	p1 = p2;
	p1.print("p1 = p2");
	Point2d p3 = p1 + p2;
	p3.print("p3 = p1 + p2");
	p3 /= 2.0f;
	p3.print("p3 /= 2.0f");
	Point2d p4(p3);
	p4.print("Point2d p4(p3)");
	p4 *= 2.0f;
	p4.print("p4 *= 2.0f");
	Point2d p5 = -p4;
	p5.print("Point2d p5 = -p4");
	p5.e20 = 7.0f;
	p5.print("p5.e20 = 7.0f");

}

void dualQuatMatrixTest(){
	glm::quat rotationQuaternion1 = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
	glm::vec3 translationVector1(0.0f, 0.0f, 0.0f);
	glm::dualquat glCombinned(rotationQuaternion1,translationVector1);
	
	/// As far as I can tell I'm stuck here
}
void dualQuatSlerpVectorTest() {
	Vec3 initialVector(0, 0, 0);
	Vec3 startTranslation(2, 0, 0);
	Quaternion endRot = QMath::angleAxisRotation(180, Vec3(0, 0, 1));
	DualQuat startDQ = DQMath::translate(startTranslation);
	DualQuat endDQ = DQMath::rotate(endRot) * startDQ;
	DualQuat slerpDQStart = DQMath::slerp(startDQ, endDQ, 0.0f);
	DualQuat slerpDQMiddle = DQMath::slerp(startDQ, endDQ, 0.5f);
	DualQuat slerpDQEnd = DQMath::slerp(startDQ, endDQ, 1.0f);

	slerpDQStart.print("Slerp with t = 0");
	DQMath::getTranslation(slerpDQStart).print("pos");
	DQMath::getRotation(slerpDQStart).print("rot");
	DQMath::rigidTransformation(slerpDQStart, initialVector).print("New Vector");
	std::cout << "***********************\n";
	slerpDQMiddle.print("Slerp with t = 0.5");
	DQMath::getTranslation(slerpDQMiddle).print("pos");
	DQMath::getRotation(slerpDQMiddle).print("rot");
	DQMath::rigidTransformation(slerpDQMiddle, initialVector).print("New Vector");
	std::cout << "***********************\n";
	slerpDQEnd.print("Slerp with t = 1");
	DQMath::getTranslation(slerpDQEnd).print("pos");
	DQMath::getRotation(slerpDQEnd).print("rot");
	DQMath::rigidTransformation(slerpDQEnd, initialVector).print("New Vector");

}


void dotTest() {
	DualQuat line1 = Vec4(0, 0, 0, 1) & Vec4(1, 0, 0, 1);
	DualQuat line2 = Vec4(0, 0, 0, 1) & Vec4(-0.7071, 0.7071, 0, 1);
	line1.print("line 1");
	line2.print("line 2");
	float angleDegrees = acos(line1 | line2) * RADIANS_TO_DEGREES;
	std::cout << "Angle between lines: " << angleDegrees << "\n";

}

void rayPlaneTest() {
	// Test out intersection of a ray and a plane two ways
	// First the traditional way using regular old vectors
	Vec3 rayStart(-5, 0, 0);
	Vec3 rayDir(0.7071, 0.7071, 0);
	rayStart.print("Ray start");
	rayDir.print("Ray direction");

	Vec3 planeNormal(-1, 0, 0);
	Vec3 pointOnPlane(10, 0, 0);
	float planeD = VMath::dot(planeNormal, pointOnPlane);
	planeNormal.print("Plane normal");
	pointOnPlane.print("Point on plane");
	std::cout << "Plane D: " << planeD << "\n\n";

	// Shove the ray equation (S +Dt) into the plane equation (ax + by + cz - d = 0) 
	// to find t
	float t = (planeD - VMath::dot(planeNormal, rayStart)) / VMath::dot(planeNormal, rayDir);
	Vec3 intersectionPoint = rayStart + t * rayDir;
	intersectionPoint.print("Intersection point using the regular way");

	// Now try using geometric algebra
	// Ray is a line joining two points
	Vec3 pointOnRay = rayStart + rayDir;
	DualQuat rayDualQuat = pointOnRay & rayStart;
	// Intersection point is the meet of the ray and the plane
	// TODO (UN) I wonder if the Plane constructor is ambiguous here on whether it is +d or -d
	Vec4 intersectionPoint2 = rayDualQuat ^ Plane(planeNormal, -planeD);
	// Normalize the point by dividing by the w component
	intersectionPoint2 = intersectionPoint2 / intersectionPoint2.w;
	intersectionPoint2.print("Intersection point using Geometric Algebra");
	// Now try the same thing but building a plane using three points
	// Say (10,0,0), (10,1,0), (10,0,1)
	Plane plane = Vec4(10, 0, 0, 1) & Vec4(10, 1, 0, 1) & Vec4(10, 0, 1, 1);
	plane.print("Plane built using three points");
	Vec4 intersectionPoint3 = rayDualQuat ^ plane;
	intersectionPoint3 = intersectionPoint3 / intersectionPoint3.w;
	intersectionPoint3.print("Intersection point using Geometric Algebra with a plane built using three points");
}


void translateAlongLineTest() {
	Vec4 point1(-1, -1, -1, 1);
	Vec4 point2(1, 1, 1, 1);
	DualQuat line = point1 & point2;
	DualQuat tr = DQMath::translateAlongLine(5, line);
	Vec4 pointToTranslate(0, 0, 0, 1);
	pointToTranslate.print("Point to translate");
	Vec4 translatedPoint = DQMath::rigidTransformation(tr, pointToTranslate);
	translatedPoint.print("Translated point");
}
void normalizeLineTest() {
	DualQuat dq(1, 2, 3, 4, 5, 6, 7, 8);
	dq.print("whole dual quaternion (Euclidean)");
	DualQuat normalized = DQMath::normalize(dq);
	normalized.print("normalized Euclidean line");

	dq.set(0, 0, 0, 0, 5, 6, 7, 8);
	dq.print("whole dual quaternion (Ideal)");
	normalized = DQMath::normalize(dq);
	normalized.print("normalized Ideal line");
}

void gradeTest() {
	DualQuat dq(1, 2, 3, 4, 5, 6, 7, 8);
	dq.print("whole dual quaternion");
	std::cout << "Grade 0 part (w part): " << DQMath::magGrade0(dq) << "\n";
	std::cout << "Grade 2 part (e23, e31, e12 parts): " << DQMath::magGrade2(dq) << " --> Check it matches " << VMath::mag(Vec3(2, 3, 4))<< "\n";
	std::cout << "Grade 2 part at infinity (e01, e02, e03 parts): " << DQMath::magGrade2Infinity(dq) << " --> Check it matches " << VMath::mag(Vec3(5, 6, 7)) << "\n";
	std::cout << "Grade 4 part (e0123 part): " << DQMath::magGrade4Infinity(dq) << "\n";
	DQMath::extractLine(dq).print("Line part of the dual quaternion");
}

void rotateTest() {
	DualQuat dq1 = DQMath::rotate(QMath::angleAxisRotation(90, Vec3(0, 1, 0)));
	DualQuat dq2 = DQMath::rotate(90, Vec3(0, 1, 0));
	dq1.print("Rotated dq using a quaternion");
	dq2.print("Rotated dq using angle and vector");
}

void dualQuatSlerpTest() {
	Vec3 startPos(-1, -2, -3);
	Vec3 endPos(1, 2, 3);
	Quaternion startRot = QMath::angleAxisRotation(0, Vec3(0, 1, 0));
	Quaternion endRot = QMath::angleAxisRotation(90, Vec3(0, 1, 0));
	DualQuat startDQ = DQMath::translate(startPos) * DQMath::rotate(startRot);
	DualQuat endDQ = DQMath::translate(endPos) * DQMath::rotate(endRot);
	DualQuat slerpDQStart  = DQMath::slerp(startDQ, endDQ, 0.0f);
	DualQuat slerpDQMiddle = DQMath::slerp(startDQ, endDQ, 0.5f);
	DualQuat slerpDQEnd    = DQMath::slerp(startDQ, endDQ, 1.0f);

	startPos.print("Starting pos");
	startRot.print("Starting rot");
	endPos.print("Ending pos");
	endRot.print("Ending rot");
	std::cout << "***********************\n";
	slerpDQStart.print("Slerp with t = 0");
	DQMath::getTranslation(slerpDQStart).print("pos");
	DQMath::getRotation(slerpDQStart).print("rot");
	std::cout << "***********************\n";
	slerpDQMiddle.print("Slerp with t = 0.5");
	DQMath::getTranslation(slerpDQMiddle).print("pos");
	DQMath::getRotation(slerpDQMiddle).print("rot");
	std::cout << "***********************\n";
	slerpDQEnd.print("Slerp with t = 1");
	DQMath::getTranslation(slerpDQEnd).print("pos");
	DQMath::getRotation(slerpDQEnd).print("rot");
}

void joinTest() {
	Vec4 pointA(-5, -5, 0, 1);
	Vec4 pointB(-5, 5, 0, 1);
	DualQuat lineJoiningPoints = pointA & pointB;
	pointA.print("Point A");
	pointB.print("Point B");
	lineJoiningPoints.print("Line AB");
	Plane yPlane(0, 1, 0, 0);
	(yPlane ^ lineJoiningPoints).print("Line pierces y plane this this point");

}

void meetTest() {
	Plane yPlane(0, 1, 0, -5);
	DualQuat yLine(0, 0, 1, 0, 0, 0, 0, 0);
	yPlane.print("y plane");
	yLine.print("y line");
	Vec4 meetPoint = yPlane ^ yLine;
	meetPoint.print("They meet at this point");
	PMath::intersection(yPlane, yLine).print("Checking with PMath call too");
}

void poincareDualityTest() {
	Plane xPlane(1, 0, 0, 0);
	Vec4 xPlaneDual = !xPlane;
	xPlane.print("x plane");
	xPlaneDual.print("the dual of the plane");
	std::cout << "***********************\n";
	Vec4 point(1, 2, 3, 1);
	Plane pointDual = !point;
	point.print("Original point");
	pointDual.print("Dual of the point");
	std::cout << "***********************\n";
	DualQuat dq = DQMath::rotate(QMath::angleAxisRotation(90, Vec3(1, 0, 0)));
	DualQuat dqDual = !dq;
	dq.print("Dual quaternion that rotates 90 degrees abt x");
	dqDual.print("Dual of the dual quaternion. Sounds weird like that");
}

void intersectionTest() {
	Plane xPlane(1, 0, 0, 0);
	Plane yPlane(0, 1, 0, 0);;
	DualQuat dqIntersection = yPlane * xPlane;
	xPlane.print("x plane");
	yPlane.print("y plane");
	dqIntersection.print("intersection line of both planes");
}

void flectorTest() {
	Flector f1;
	f1.plane = Plane(1.0f, 0.0f, 0.0f, 0.0f);
	f1.point = Vec4(0.0f, 1.0f, 0.0f, 1.0f);

	Flector f2;
	f2.plane = Plane(1.0f, 0.0f, 0.0f, 0.0f);
	f2.point = Vec4(0.0f, 1.0f, 0.0f, 1.0f);

	f1.print("Flector 1");
	f2.print("Flector 2");
	f1 += f2;
	f1.print("f1 after adding f2");
}

void dualQuatTest() {
	/// Ok I'm not doing this right 
	DualQuat identity;
	identity.print("Identity of Umer's dual quaternion");
	
	glm::quat rotationQuaternion = glm::angleAxis(glm::radians(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
	glm::vec3 translationVector(0.0f, 0.0f, 0.0f);
	glm::dualquat glmIdeniity(rotationQuaternion,translationVector);
	glmPrintDQ( glmIdeniity, "GLM's identity (I think) ");
	printf("\n\n");
	
	/// Translate test 
	DualQuat translate2(1.0f, 0.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f);
	translate2.print("Translate 10 units along x");
	
	glm::quat rotationQuaternion2 = glm::angleAxis(glm::radians(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
	glm::vec3 translationVector2(10.0f, 0.0f, 0.0f);
	glm::dualquat glmTranslate2(rotationQuaternion2,translationVector2);
	glmPrintDQ(glmTranslate2, "GLM tranlate by 10");
	printf("\n\n");

	/// Rotate test Oh No, they don't match. If I reverse the axis of rotation Vec3(0.0f, -1.0f, 0.0f)
	/// it all works. I dumped into this before. 
	DualQuat pureRotate = DQMath::rotate(QMath::angleAxisRotation(90.0f, Vec3(0.0f, 1.0f, 0.0f)));
	pureRotate.print("90 rotate along Y");

	glm::quat rotationQuaternion3 = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
	glm::vec3 translationVector3(0.0f, 0.0f, 0.0f);
	glm::dualquat glmTranslate3(rotationQuaternion3,translationVector3);
	glmPrintDQ(glmTranslate3, "GLM 90 along Y");
	printf("\n\n");

	//DualQuat identity;
	//DualQuat translate1(1.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);
	//translate1.print("Translate 4 units along x");
	//DualQuat combine = translate1 * translate2;
	//combine.print("Multiply two translate dual quaternions to get 14 units along x");
	//DualQuat pureRotation = DQMath::rotate(QMath::angleAxisRotation(90, Vec3(0, 1, 0)));
	//pureRotation.print("Pure rotation of 90 degrees about y axis");
	//DualQuat pureTranslation = DQMath::translate(Vec3(1, 2, 3));
	//pureTranslation.print("Pure translation by (1, 2, 3)");
	//Vec4 initialPos(1, 0, 0, 1);
	//initialPos.print("Initial position");
	//DQMath::rigidTransformation(pureRotation, initialPos).print("Rotated position");
	//DQMath::rigidTransformation(pureTranslation, initialPos).print("Translated position");
}

void QuadraticTest() {
    /// Two root solution 
    Roots root = Quadratic::findRoots(1.0f, -5.0f, 6.0f);
    root.print("Quadratic roots: should be 2.0,3.0");

    /// Another two root solution 
    root = Quadratic::findRoots(5.0f, 6.0f, 1.0f);
    root.print("Quadratic roots: should be -0.2,-1.0");

    /// One root solution 
    root = Quadratic::findRoots(1.0f, 4.0f, 4.0f);
    root.print("Quadratic roots: should be -2.0,-2.0");

    /// No solutions (all imaginary -1 + 2i, -1 - 2i)  
    root = Quadratic::findRoots(1.0f, 2.0f, 5.0f);
    root.print("Quadratic roots: 0, 0.0, 0.0,");
}


void RayTest() {
    Vec3 v1(0.0f, 0.0f, 0.0f);
    Vec3 v2(1.0f, 0.0f, 0.0f);
    Ray ray(v1, v2);
    Vec3 v3 = ray.getPos(0.5f);
    v3.print();
    Plane plane(0.0f, 1.0f, 0.0f, 10.0f);
    //Vec3 intersection = RMath::rayPlaneIntersection(ray,plane);
    //intersection.print();
}


void RaySphereTest() {
    /// Two roots 
    Ray r(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f));
    Sphere s(Vec3(3.0f, 0.0f, 0.0), 1.0f);
    Roots intersection = RMath::intersection(r, s);
    intersection.print();

    /// One root 
    Ray r1(Vec3(0.0f, -1.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f));
    Sphere s1(Vec3(3.0f, 0.0f, 0.0), 1.0f);
    Roots intersection1 = RMath::intersection(r1, s1);
    intersection1.print();

    /// No roots - no intersection
    Ray r2(Vec3(0.0f, -1.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f));
    Sphere s2(Vec3(3.0f, 0.0f, 0.0), 1.0f);
    Roots intersection2 = RMath::intersection(r1, s1);
    intersection2.print();

}

void slerpTest() {
	Euler e1(90.0f, 0.0f, 0.0f);
	Quaternion q1 = QMath::toQuaternion(e1);
	q1.print("Start");

	Euler e2(0.0f, 90.0f, 0.0f);
	Quaternion q2 = QMath::toQuaternion(e2);
	q2.print("End");

	for (float t = 0.0f; t < 1.1f; t+=0.1f) {
		Quaternion q = QMath::slerp(q1, q2, t);
		q.print("slerping");
	}

}

void determinantTest(){
	/// These vectors should return a value of 30 - it does.
	/// Swap any two and the sign should change - it does
	Matrix4 m4;
	m4.setColumn(Matrix4::Colunm::zero,Vec4(1, 0, 2, -1));
	m4.setColumn(Matrix4::Colunm::one, Vec4(3, 0, 0, 5));
	m4.setColumn(Matrix4::Colunm::two, Vec4(2, 1, 4, -3));
	m4.setColumn(Matrix4::Colunm::three,Vec4(1, 0, 5, 0));
	
	printf("%f\n",MMath::determinate(m4));

	/// deternimant of the identity matrix = 1.0 - it is 
	Matrix3 m3;
	printf("%f\n", MMath::determinate(m3));


	Matrix4 perspectiveM = MMath::perspective(45.0f, (16.0f / 9.0f), 0.5f, 100.0f);
	printf("determinant of the perspective Matrix %f\n", MMath::determinate(perspectiveM));
}

void hashTest(){
	Vec3 v1(1.1f, 1.0f, 1.0f);
	Vec3 v2(1.1f, 0.0f, 1.0f);
	Vec3 v3(1.1f, 0.0f, 1.0f);
	Vec3 v4(1.1f, 1.0f, 1.0f);

	vec3 glmV1(1.1f, 1.0f, 1.0f);
	vec3 glmV2(1.1f, 1.0f, 1.0f);
	bool t = (v1 == v2);
	bool glmt = (glmV1 == glmV2);
	hash<Vec3> hasher;
	size_t myHash = hasher(v1);
	
	hash<vec3> glmHasher;
	size_t glmHash = glmHasher(glmV1);
	printf("%x vs. %x\n", myHash, glmHash);

	std::unordered_map<Vec3, uint32_t> uniqueVerts;
	static uint32_t count = 0;
	if (uniqueVerts.count(v1) == 0) {
		uniqueVerts[v1] = count;
		++count;
	}
	printf("%d\n", uniqueVerts.size());

	if (uniqueVerts.count(v2) == 0) {
		uniqueVerts[v2] = count;
		++count;
	}
	printf("%d\n", uniqueVerts.size());

	if (uniqueVerts.count(v3) == 0) {
		uniqueVerts[v3] = count;
		++count;
	}
	printf("%d\n", uniqueVerts.size());


	if (uniqueVerts.count(v4) == 0) {
		uniqueVerts[v4] = count;
		++count;
	}
	printf("%d\n\n", uniqueVerts.size());

	/////////// glm ///////////////
	std::unordered_map<vec3, uint32_t> glmUniqueVerts;
	static uint32_t glmCount = 0;
	if(glmUniqueVerts.count(glmV1) == 0){
		glmUniqueVerts[glmV1] = glmCount;
		++glmCount;
	}
	printf("%d\n", glmUniqueVerts.size());
	if (glmUniqueVerts.count(glmV2) == 0) {
		glmUniqueVerts[glmV2] = glmCount;
		++glmCount;
	}
	printf("%d\n", glmUniqueVerts.size());
}

void quaternionTest() {
	Quaternion qLookat = QMath::lookAt(Vec3(1.0f, 0.0f, 1.0f),Vec3(0.0f, 1.0f, 0.0));
	qLookat.print();
	
	glm::quat glmlookat = glm::quatLookAt(normalize(vec3(1.0f, 0.0f, 1.0)), vec3(0.0f, 1.0f, 0.0));
	glmPrintQ(glmlookat,"glm lookat");


	Matrix3 rm = Matrix3 (MMath::rotate(-270.0f, Vec3(1.0f, 0.0f, 0.0f)));
	rm.print("My rotation matrix");
	Quaternion qm = QMath::toQuaternion(rm);
	qm.print("Quaternion rotate");
	Matrix4 meMat = MMath::toMatrix4(qm);
	meMat.print("My matrix from Q");

	mat3 glmrot = mat3(rotate(glm::radians(-270.0f), vec3(1.0f,0.0f,0.0f)));
	glmPrintM4(glmrot, "GLM rot");
	glm::quat glmqm= glm::quat_cast(glmrot);
	glmPrintQ(glmqm,"glm matrix from Q");

	glm::mat4 glmRot = glm::toMat4(glmqm);
	glmPrintM4(glmRot,"glm rotation Matrix from Q");
	

	Quaternion q3 = QMath::angleAxisRotation(-90.0,Vec3(1.0f,0.0f,0.0f));
	q3.print("my Quat");
	glm::quat myQuaternion = glm::angleAxis(glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	glmPrintQ(myQuaternion, "glm Quat");


	
	/// Lets say I have a unit vector along the x-axis 1,0,0. 
	 /*Rotate 45.0 degree around the z-axis
	 The resulting vector should be 0.70711, 0.70711, 0.0 
	 Let's test this in every way I can think of*/
	Vec3 v(1.0, 0.0, 0.0);
	Quaternion q = QMath::angleAxisRotation(90.0,Vec3(0.0,1.0,0.0));
	Euler e2 = EMath::toEuler(q);
	e2.print("from Q");
	
	q.print("The rotation Quaternion");
	Euler e(0.0, 0.0, 45.0);

	Quaternion qe = QMath::toQuaternion(e);
	
	qe.print("from Euler");
	Vec3 v2 = qe * v * ~qe;
	v2.print("The slow way");



	Vec3 v3 = QMath::rotate(v, qe);
	v3.print("faster way");

	Matrix3 m3 = MMath::toMatrix3(qe);
	Vec3 v4 = m3 * v;
	v4.print("Mat3");

	Matrix4 m4 = MMath::toMatrix4(qe);
	Vec3 v5 = m4 * v;
	v5.print("Mat4");

	Quaternion q2 = QMath::angleAxisRotation(90.0, Vec3(0.0, 0.0, 1.0));
	q2 = QMath::pow(q2, 0.5);
	Vec3 v6 = QMath::rotate(v, q2);
	v6.print("Using the pow function");

	printf("Magnitude of q \n%f\n", QMath::magnitude(q));
	Quaternion conj_q = QMath::conjugate(q);
	conj_q.print("conjugate of q");

	Quaternion inv_q = QMath::inverse(q);
	inv_q.print("inv of q");

	Quaternion q4 = q * inv_q;
	q4.print("q * q-1 is the identity");

}


void inverseTest(){
	Matrix4 rot = MMath::rotate(90.0f, Vec3(0.0f,1.0f,0.0f));
	Matrix4 invRot = MMath::inverse(rot);
	Matrix4 product = rot * invRot;
	product.print();

	Matrix3 rot3 = MMath::rotate(45.0f, Vec3(0.0f, 1.0f, 0.0f));
	Matrix3 invRot3 = MMath::inverse(rot);
	Matrix3 product3 = rot * invRot;
	product3.print();

	
}


void planeTest() {
	/*Plane p1(2.0f, -2.0f, 5.0f, 8.0f);
	p1.print();
	Vec3 v = Vec3(4.0f, -4.0f, 3.0f);
	v.print();
	float distance = PMath::distance(v, p1);
	printf("%f vs. 6.79\n", distance);
	Plane p2 = PMath::normalize(p1);
	p2.print();
	float distance2 = PMath::distance(v, p2);
	printf("%f vs. 6.79\n", distance2);

	Plane p3(1, 2, 2, -6);
	Vec3 v3(-1, -2, -3);
	float distance3 = PMath::distance(v3, p3);
	printf("%f vs. %f\n", distance3, -17.0/3.0);
	Plane p4 = PMath::normalize(p3);
	p4.print();
	float distance4 = PMath::distance(v3, p4);
	printf("%f vs. %f\n", distance4, -17.0 / 3.0);*/

	Matrix4 proj = MMath::perspective(52.8f, 1.0f, 1.0f, 10.0f) * MMath::lookAt(Vec3(0.0f, 0.0f, 8.0f), Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f));
	
	proj.print();

#define INDEX(column,row) (proj[(row-1) * 4 + (column-1)])

	
	Plane near(INDEX(4,1) + INDEX(3,1),
			INDEX(4,2) + INDEX(3,2),
			INDEX(4,3) + INDEX(3,3),
			INDEX(4,4) + INDEX(3,4));
	near = PMath::normalize(near);
	near.print();
	Plane far(INDEX(4,1) - INDEX(3,1),
			INDEX(4,2) - INDEX(3,2),
			INDEX(4,3) - INDEX(3,3),
			INDEX(4,4) - INDEX(3,4));
	far = PMath::normalize(far);
	far.print();
	
	Vec3 v5(0.0, 0.0, 0.0);
	float distance5 = PMath::distance(v5, near);
	printf("near %f\n", distance5);
	distance5 = PMath::distance(v5, far);
	printf("far %f\n", distance5);

	Plane p1(1, 0, 0, 4);
	Plane p1Inverse = PMath::inverse(p1);
	p1.print("Plane before inverse");
	p1Inverse.print("Plane after inverse");

	Plane p2(4, 0, 0, 2);
	Plane p2Normalized = PMath::normalize(p2);
	p2.print("Plane before normalizing");
	p2Normalized.print("Plane after normalizing");
	std::cout << "-------------------\n";
	p2.set(1, 0, 0, 8);
	p1.print("Starting plane");
	p2.print("Ending plane");
	PMath::normalize(PMath::midPlane(p1, p2)).print("Mid plane");
}


void rotationIsOrthogonal() {
	Matrix4 M = MMath::rotate(180.0f, Vec3(0, 1, 0));
	Vec4 v0 = M.getColumn(Matrix4::Colunm::zero);
	Vec4 v1 = M.getColumn(Matrix4::Colunm::one);
	Vec4 v2 = M.getColumn(Matrix4::Colunm::two);
	Vec4 v3 = M.getColumn(Matrix4::Colunm::three);
	printf("%f\n", VMath::dot(v3, v0));
	printf("%f\n", VMath::dot(v3, v1));
	printf("%f\n", VMath::dot(v3, v2));
	printf("%f\n", VMath::dot(v3, v3));
	printf("If all the values are zero, the matrix is orthogonal\n");
}

void moveCopyConstructors() {
	
}

void viewportNDCTest() {
	Matrix4 m = MMath::viewportNDC(1024, 1024);
	m.print();
	Vec3 pos0(0, 0, 0);
	Vec3 result0 = m * pos0;
	result0.print();

	Vec3 pos1(-1, 1, 1);
	Vec3 result1 = m * pos1;
	result1.print();
}

void multiplyMatrixTest() {
	Matrix4 tmSSF = MMath::translate(10.0f, 10.0f, 10.0f);
	Matrix4 rmSSF = MMath::rotate(90.0f, 0.0f, 1.0f, 0.0f);
	Matrix4 smSSF = MMath::scale(0.75f, 0.75f, 0.75f);
	Matrix4 resultSSF = tmSSF * rmSSF * smSSF;
	resultSSF.print();

	glm::mat4 mt = glm::translate(glm::mat4(1.0f), glm::vec3(10.0f, 10.0f, 10.0f));
	glm::mat4 mr = glm::rotate(mat4(), glm::radians(90.0f), vec3(0, 1.0f, 0));
	glm::mat4 ms = glm::scale(mat4(), glm::vec3(0.75f, 0.75f, 0.75f));
	glm::mat4 result = mt * mr * ms;
	
	glmPrintM4(result);
}
void Vec3MultiplyMat4Test() {
	Matrix4 translate = MMath::rotate(90.0, 0.0, 1.0,0.0);
	Vec3 pos(5.0, 0.0, 0.0);
	Vec4 xxx = pos;
	xxx.print();
	Vec3 result = translate * pos;
	result.print();
}
void RotationTest(){
	mat4 rot2 = rotate(mat4(), 3.141592654f/2.0f, vec3(1.0f,0.0f,0.0f));
	float  m[16] = {0.0};

	const float *pSource = (const float*)glm::value_ptr(rot2);
	for (int i = 0; i < 16; ++i)
		m[i] = pSource[i];
	printf("%1.8f %1.8f %1.8f %1.8f\n%1.8f %1.8f %1.8f %1.8f\n%1.8f %1.8f %1.8f %1.8f\n%1.8f %1.8f %1.8f %1.8f\n\n",
				m[0], m[4], m[8],  m[12],
				m[1], m[5], m[9],  m[13],
				m[2], m[6], m[10], m[14],
				m[3], m[7], m[11], m[15]);

}
void UnOrthoTest() {
	/// Just seeing if I can deconstruct the the ortho matrix
	int w = 800, h = 600;
	Matrix4 ndc = MMath::viewportNDC(w,h);
	
	float xMax = 10.0, xMin = -10.0, yMax = 10.0, yMin = -10.0, zMax = 1.0, zMin = -10.0;
	Matrix4 ortho = MMath::orthographic(xMin, xMax, 
										yMin, yMax, 
										zMin, zMax);

	Matrix4 projection = ortho * ndc;
	projection.print();
	
	Matrix4 m;
	/// This is the ortho * ndc matrix broken down into its parts 
	Matrix4 m1 = MMath::scale(2.0f / (xMax - xMin), 2.0f / (yMax - yMin),-2.0f / (zMax - zMin));
	Matrix4 m2 = MMath::translate( -(xMax + xMin) / (xMax - xMin), -(yMax + yMin) / (yMax - yMin), -(zMax + zMin) / (zMax - zMin)); 
	Matrix4 m3 = MMath::scale(1.0f, -1.0f, 1.0f);
	Matrix4 m4 = MMath::scale(float(w)/2.0f, float(h)/2.0f, 1 - 0);
	Matrix4 m5 = MMath::translate(float(w)/2.0f,float(h)/2.0f, 0);

	/// Here they are in their inverse 
	Matrix4 m6 = MMath::inverse(m1);
	Matrix4 m7 = MMath::translate( (xMax + xMin) / (xMax - xMin), (yMax + yMin) / (yMax - yMin), (zMax + zMin) / (zMax - zMin)); 
	Matrix4 m8 = MMath::scale(1.0f, -1.0f, 1.0f);
	Matrix4 m9 = MMath::inverse(MMath::scale(float(w)/2.0f, float(h)/2.0f, 1 - 0));
	Matrix4 m10 = MMath::translate(-float(w)/2.0f,-float(h)/2.0f, 0);

	m = m1*m2*m3*m4*m5;  /// creates the ortho * ndc
	m *= m10 *m9 *m8 *m7 *m6; /// Now back out 
	m.print(); /// Should be an identity matrix
	/// It is!!!
		
	Matrix4 invOrtho = MMath::unOrtho(projection );
	invOrtho.print();
	Vec3 v1(400.0,300.0,0.0);
	Vec3 v2(10.0,0.0,10.0);
	Vec3 result1 = invOrtho  * v1;
	result1.print();	
}


void LookAtTest(){
	glm::mat4 mt = glm::lookAt(glm::vec3(0.0f, 0.0f, -10.0f),
								glm::vec3(0.0f, 0.0f, 0.0f), 
								glm::vec3(1.0f, 0.0f, 0.0f));
	
	Vec3 v = Vec3(0, 0, 0);
	Matrix4 lookat = MMath::lookAt(Vec3(0.0,0.0,-10.0), Vec3(0,0,0), Vec3(1,0,0));
	
	lookat.print();
	glmPrintM4(mt);
	Vec3 v1 = lookat * v;
	v1.print();
}



///////////////////////////////////////////////////////////////////////////////////////////////
/// These are print statements for glm - they don't have them  
///////////////////////////////////////////////////////////////////////////////////////////////
void glmPrintM4(glm::mat4  mat, const char* comment){
	int i, j;
	if (comment) printf("%s\n", comment);
	for (j = 0; j<4; j++) {
		for (i = 0; i<4; i++) {
			printf("%1.4f ", mat[i][j]);
		}
		printf("\n");
	}
}
void glmPrintM3(glm::mat3  mat, const char* comment){
	int i, j;
	if (comment) printf("%s\n", comment);
	for (j = 0; j<3; j++) {
		for (i = 0; i<3; i++) {
			printf("%1.4f ", mat[i][j]);
		}
		printf("\n");
	}
}

void glmPrintQ(glm::quat q, const char* comment) {
	if (comment) printf("%s\n", comment);
	///                                    w     i     j     k
	printf("%1.4f %1.4f %1.4f %1.4f \n", q[3], q[0], q[1], q[2]);
}

/// This is a bit messed up in my mind right now. GLM and I write a quarternion as w, i, j, k
/// While a translate is dx,dy,dz,(w)
void glmPrintDQ(glm::dualquat dq, const char* comment) {
	if (comment) printf("%s\n", comment);
	///  real w,x,y,z  dual x,y,z,w
	printf("real: %1.4f %1.4f %1.4f %1.4f \ndual: %1.4f %1.4f %1.4f %1.4f \n",
		dq.real.w, dq.real.x,dq.real.y,dq.real.z,
		dq.dual.x, dq.dual.y, dq.dual.z,dq.dual.w);
}

void glmPrintV3(glm::vec3 v, const char* comment) {
	if (comment) printf("%s\n", comment);
	printf("%1.4f %1.4f %1.4f\n", v[0], v[1], v[2]);
}

void glmPrintV4(glm::vec4 v, const char* comment) {
	if (comment) printf("%s\n", comment);
	printf("%1.4f %1.4f %1.4f %1.4f\n", v[0], v[1], v[2], v[3]);
}