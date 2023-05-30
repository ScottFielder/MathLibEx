#include "Triangle.h"
using namespace MATH;
using namespace GEOMETRY;

void Triangle::generateVerticesAndNormals()
{
	vertices.clear();
	normals.clear();

	vertices.push_back(a);
	vertices.push_back(b);
	vertices.push_back(c);

	Vec3 normal = calcNormal();

	normals.push_back(normal);
	normals.push_back(normal);
	normals.push_back(normal);
}

RayIntersectionInfo Triangle::rayIntersectionInfo(const Ray& ray) const
{
	// First intersect ray with plane of triangle
	const Vec3 planeNormal = calcNormal();
	const float planeD = VMath::dot(planeNormal, a);
	float t = (planeD - VMath::dot(planeNormal, ray.start)) / (VMath::dot(planeNormal, ray.dir));
	Vec3 p = ray.currentPosition(t);
	// Now figure out if point p is within the bounds of the triangle
	RayIntersectionInfo rayInfo;
	if (!isPointInTriangle(p)) {
		rayInfo.isIntersected = false;
		return rayInfo;
	}
	rayInfo.isIntersected = true;
	rayInfo.t = t;
	rayInfo.intersectionPoint = p;
	return rayInfo;
}

BarycentricCoords Triangle::calcBarycentric(const Vec3& p) const
{
	// Code from section 3.4 of Real Time Collision Detection by Ericson
	Vec3 v0 = b - a; 
	Vec3 v1 = c - a;
	Vec3 v2 = p - a;
	float d00 = VMath::dot(v0, v0);
	float d01 = VMath::dot(v0, v1);
	float d11 = VMath::dot(v1, v1);
	float d20 = VMath::dot(v2, v0);
	float d21 = VMath::dot(v2, v1);
	float denom = d00 * d11 - d01 * d01;

	BarycentricCoords result;
	result.v = (d11 * d20 - d01 * d21) / denom;
	result.w = (d00 * d21 - d01 * d20) / denom;
	result.u = 1.0f - result.v - result.w;
	return result;
}

bool Triangle::isPointInTriangle(const Vec3& p) const
{
	// Code from section 3.4 of Real Time Collision Detection by Ericson
	BarycentricCoords result = calcBarycentric(p);
	return result.v >= 0.0f && result.w >= 0.0f && (result.v + result.w) <= 1.0f;
}
