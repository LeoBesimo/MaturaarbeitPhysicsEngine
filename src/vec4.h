#pragma once

#include <iostream>
namespace lge 
{
	struct vec4
	{
		double w = 0;
		double x = 0;
		double y = 0;
		double z = 0;

		vec4(double w, double x, double y, double z) :
			w(w), x(x), y(y), z(z)
		{}

		friend std::ostream& operator<<(std::ostream& os, const vec4& a)
		{
			return os << "W: " << a.w << " X: " << a.x << " Y: " << a.y << " Z: " << a.z;
		}

		vec4 operator-(void) const
		{
			return vec4(-w, -x, -y, -z);
		}

		vec4 operator-(const vec4& a) const
		{
			return vec4(w - a.w, x - a.x, y - a.y, z - a.z);
		}

		vec4 operator-(double a) const
		{
			return vec4(w - a, x - a, y - a, z - a);
		}

		vec4 operator+(void) const
		{
			return vec4(+w, +x, +y, +z);
		}

		vec4 operator+(const vec4& a) const
		{
			return vec4(w + a.w, x + a.x, y + a.y, z + a.z);
		}

		vec4 operator+(double a) const
		{
			return vec4(w + a, x + a, y + a, z + a);
		}

		vec4 operator*(const vec4& a) const
		{
			return vec4(w * a.w, x * a.x, y * a.y, z * a.z);
		}

		vec4 operator*(double a) const
		{
			return vec4(w * a, x * a, y * a, z * a);
		}

		vec4 operator/(const vec4& a) const
		{
			return vec4(w / a.w, x / a.x, y / a.y, z / a.z);
		}

		vec4 operator/(double a) const
		{
			return vec4(w / a, x / a, y / a, z / a);
		}

		void operator-=(const vec4& a)
		{
			w -= a.w;
			x -= a.x;
			y -= a.y;
			z -= a.z;
		}

		void operator+=(const vec4& a)
		{
			w += a.w;
			x += a.x;
			y += a.y;
			z += a.z;
		}

		void operator*=(const double a)
		{
			w *= a;
			x *= a;
			y *= a;
			z *= a;
		}

		void operator*=(const vec4& a)
		{
			w *= a.w;
			x *= a.x;
			y *= a.y;
			z *= a.z;
		}

		void operator/=(const double a)
		{
			w /= a;
			x /= a;
			y /= a;
			z /= a;
		}

		void operator/=(const vec4& a)
		{
			w /= a.w;
			x /= a.x;
			y /= a.y;
			z /= a.z;
		}

		void set(double w_, double x_, double y_, double z_)
		{
			w = w_;
			x = x_;
			y = y_;
			z = z_;
		}

		double lenSqr()
		{
			return w * w + x * x + y * y + z * z;
		}

		double len()
		{
			return std::sqrt(w * w + x * x + y * y + z * z);
		}

		vec4 normalize()
		{
			double l = len();

			if (l == 0) return vec4(0, 0, 0, 0);

			double invLen = 1.0 / l;

			w *= invLen;
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
				w *= limit;
				x *= limit;
				y *= limit;
				z *= limit;
			}
		}
	};
}