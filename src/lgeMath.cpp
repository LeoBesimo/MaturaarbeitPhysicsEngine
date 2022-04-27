#include "lgeMath.h"

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
}