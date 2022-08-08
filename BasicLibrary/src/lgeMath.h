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
		return sqrt(a * a + b * b); // square root of (a^2 + b^2)
	}

	float fastInvSqrt(float n)  // fast inverse Square Root Algorithm source : "https://en.wikipedia.org/wiki/Fast_inverse_square_root"
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
		return a.x * b.x + a.y * b.y; //dot product of 2 2d Vectors
	}

	double detMat2(mat2 matrix)
	{
		return matrix.x.x * matrix.y.y - matrix.x.y * matrix.y.x;  // determinant of a 2 dimensional matrix
	}

	mat2 invMat2(mat2 matrix) //returns the inverse of a 2d matrix
	{
		double det = detMat2(matrix);
		if (det == 0) return mat2(); //check if matrix has an inverse

		double invDet = 1 / det; // inverse of determinant
		mat2 inv = mat2(matrix.y.y, -matrix.y.x, -matrix.x.y, matrix.x.x) * invDet;  //inversse of the original matrix
		return inv;
	}
	double crossVec2(vec2 a, vec2 b)
	{
		return (a.x * b.y) - (a.y * b.x);  //crossproduct of 2 2d vectors
	}
	vec2 crossVec2Scalar(double s, vec2 a)
	{
		return vec2(-s * a.y , s * a.x); //crossproduct of scalar an 2d vector
	}
	vec2 crossVec2Scalar(vec2 a, double s)
	{
		return vec2(s * a.y, -s * a.x); //crossproduct of scalar an 2d vector (other possible order)
	}
}

