#ifndef QUADRATIC_H
#define QUADRATIC_H

namespace MATHEX {
    using namespace MATH;
    struct Roots {
        int numRoots;
        float firstRoot, secondRoot;
        void print(const char* comment = nullptr) const {
			if (comment) printf("%s\n", comment);
			printf("%d %1.8f %1.8f\n", numRoots,firstRoot,secondRoot);		  
		}
    };

    class Quadratic {
    public: 
        static Roots findRoots(const float& a, const float& b, const float& c) {
            float discriminant = b * b - 4.0f * a * c;
            if (discriminant < 0.0f) { /// No solutions 
                return Roots{ 0, 0.0f, 0.0f };
            }
            else if (fabs(discriminant) < VERY_SMALL) { /// Only one solution
                float result =  (-b + sqrt(discriminant)) / (2.0f * a);
                return Roots{ 1,result, result };
            }
            else { /// Two solutions 
                float root1 = (-b + sqrt(discriminant)) / (2.0f * a);
                float root2 = (-b - sqrt(discriminant)) / (2.0f * a);
                return Roots{ 2,  std::min(root1, root2),std::max(root1, root2)  };
            }
        }
    };
}
#endif
