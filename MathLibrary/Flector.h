#ifndef FLECTOR_H
#define FLECTOR_H

// A flector is short for a rotoreflection or transflection
// And that is jargon for a combination of rotation & reflection and/or translation & reflection
// Think handclap and footprints

#include "Plane.h"
#include "Vector.h"

namespace MATH {
	struct Flector {
		Plane plane;
		Vec4 point;

		inline Flector& operator += (const Flector& f) {
			plane += f.plane;
			point += f.point;
			return *this;
		}
	};
}
#endif
