#pragma once

#include "components.h"

namespace lge
{
	double PythagoreanSolve(double a, double b);
	float fastInvSqrt(float n);
	double dotVec2(vec2 a, vec2 b);
	double detMat2(mat2 matrix);
	mat2 invMat2(mat2 matrix);
}