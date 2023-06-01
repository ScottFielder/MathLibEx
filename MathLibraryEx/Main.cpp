#include "EMath.h"
#include "PMath.h"
#include "AAMath.h"
#include "Sphere.h"
#include "Fourier.h"
#include "Randomizer.h"
#include "Hash.h"
#include "Ray.h"
#include "RMath.h"
#include "Quadratic.h"
#include "DQMath.h"
#include "MMath.h"
using namespace MATH;
using namespace MATHEX;

void RayTest();
void QuadraticTest();
void SphereTest();
void DualQuatTest();


int main( int argc, char* argv){
    QuadraticTest();
    SphereTest();
    DualQuatTest();
 
    Matrix4 rmat = MMath::rotate(45.0f,Vec3(0.0f,0.0f,1.0f));
    rmat.print("my rotate");
    Matrix4 tmat = MMath::translate(1.0f,0.0f,0.0f);
    tmat.print("my translate");
    (rmat * tmat).print("r*t"); 
    (tmat * rmat).print("t*r"); 
    DualQuatTest();
}

void DualQuatTest(){
    DualQuat dqr(QMath::angleAxisRotation(45.0f,Vec3(0.0f,0.0f,1.0f)));
    DualQuat dqt(Vec3(1.0f,0.0f,0.0f));;
    DQMath::toMatrix4(dqt * dqr).print("dqt * dqr");
    DQMath::toMatrix4(dqr * dqt).print("dqr * dqt");
}

void SphereTest() {
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