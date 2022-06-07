#pragma once

#include "components.h"

namespace lge
{
	inline extern double PythagoreanSolve(double a, double b);
	inline extern float fastInvSqrt(float n);
	inline extern double dotVec2(vec2 a, vec2 b);
	inline extern double detMat2(mat2 matrix);
	inline extern mat2 invMat2(mat2 matrix);
	inline extern double crossVec2(vec2 a, vec2 b);
	inline extern vec2 crossVec2Scalar(double s, vec2 a);
	inline extern vec2 crossVec2Scalar(vec2 a, double s);
}

namespace lge
{
	double PythagoreanSolve(double a, double b)
	{
		return sqrt(a * a + b * b);
	}

	float fastInvSqrt(float n)
	{

		const float threehalfs = 1.5F;
		float y = n;

		long i = *(long*)&y;

		i = 0x5f3759df - (i >> 1);
		y = *(float*)&i;

		y = y * (threehalfs - ((n * 0.5F) * y * y));
		y = y * (threehalfs - ((n * 0.5F) * y * y));

		return y;
	}

	double dotVec2(vec2 a, vec2 b)
	{
		return a.x * b.x + a.y * b.y;
	}

	double detMat2(mat2 matrix)
	{
		return matrix.x.x * matrix.y.y - matrix.x.y * matrix.y.x;
	}

	mat2 invMat2(mat2 matrix)
	{
		double det = detMat2(matrix);
		if (det == 0) return mat2();

		double invDet = 1 / det;
		mat2 inv = mat2(matrix.y.y, -matrix.y.x, -matrix.x.y, matrix.x.x) * invDet;
		return inv;
	}
	double crossVec2(vec2 a, vec2 b)
	{
		return (a.x * b.y) - (a.y * b.x);
	}
	vec2 crossVec2Scalar(double s, vec2 a)
	{
		return vec2(-s * a.y , s * a.x);
	}
	vec2 crossVec2Scalar(vec2 a, double s)
	{
		return vec2(s * a.y, -s * a.x);
	}
}

