#pragma once

#include <iostream>

namespace lge 
{

	struct vec3
	{
		double x = 0;
		double y = 0;
		double z = 0;

		vec3(double x = 0, double y = 0, double z = 0) :
			x(x), y(y), z(z)
		{}

		friend std::ostream& operator<<(std::ostream& os, const vec3& a)
		{
			return os << "X: " << a.x << " Y: " << a.y << " Z: " << a.z;
		}

		vec3 operator-(void) const
		{
			return vec3(-x, -y, -z);
		}

		vec3 operator-(const vec3& a) const
		{
			return vec3(x - a.x, y - a.y, z - a.z);
		}

		vec3 operator-(double a) const
		{
			return vec3(x - a, y - a, z - a);
		}

		vec3 operator+(void) const
		{
			return vec3(+x, +y, +z);
		}

		vec3 operator+(const vec3& a) const
		{
			return vec3(x + a.x, y + a.y, z + a.z);
		}

		vec3 operator+(double a) const
		{
			return vec3(x + a, y + a, z + a);
		}

		vec3 operator*(const vec3& a) const
		{
			return vec3(x * a.x, y * a.y, z * a.z);
		}

		vec3 operator*(double a) const
		{
			return vec3(x * a, y * a, z * a);
		}

		vec3 operator/(const vec3& a) const
		{
			return vec3(x / a.x, y / a.y, z / a.z);
		}

		vec3 operator/(double a) const
		{
			return vec3(x / a, y / a, z / a);
		}

		void operator-=(const vec3& a)
		{
			x -= a.x;
			y -= a.y;
			z -= a.z;
		}

		void operator+=(const vec3& a)
		{
			x += a.x;
			y += a.y;
			z += a.z;
		}

		void operator*=(const double a)
		{
			x *= a;
			y *= a;
			z *= a;
		}

		void operator*=(const vec3& a)
		{
			x *= a.x;
			y *= a.y;
			z *= a.z;
		}

		void operator/=(const double a)
		{
			x /= a;
			y /= a;
			z /= a;
		}

		void operator/=(const vec3& a)
		{
			x /= a.x;
			y /= a.y;
			z /= a.z;
		}

		void set(double x_, double y_, double z_)
		{
			x = x_;
			y = y_;
			z = z_;
		}

		double lenSqr()
		{
			return x * x + y * y + z * z;
		}

		double len()
		{
			return std::sqrt(x * x + y * y + z * z);
		}

		vec3 normalize()
		{
			double l = len();

			if (l == 0) return vec3(0, 0, 0);

			double invLen = 1.0 / l;

			x *= invLen;
			y *= invLen;
			z *= invLen;

			return *this;
		}

		void limit(double limit)
		{
			if (limit > len())
			{
				normalize();
				x *= limit;
				y *= limit;
				z *= limit;
			}
		}
	};
}