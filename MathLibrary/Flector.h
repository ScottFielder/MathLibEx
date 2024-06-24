#ifndef FLECTOR_H
#define FLECTOR_H

// A flector is short for a rotoreflection or transflection
// And that is jargon for a combination of rotation & reflection and/or translation & reflection
// Think handclap and footprints

#include <Vector.h>
#include "Plane.h"

namespace MATHEX {
	struct Flector {
		Plane plane;
		MATH::Vec4 point;

		inline Flector& operator += (const Flector& f) {
			plane += f.plane;
			point += f.point;
			return *this;
		}

		void print(const char* comment = nullptr) const {
			if (comment) printf("%s\n", comment);
			printf("Plane(%f %f %f %f) Point(%f %f %f %f)\n", 
				plane.x, plane.y, plane.z, plane.d,
				point.x, point.y, point.z, point.w
			);
		}
	};
}
#endif
