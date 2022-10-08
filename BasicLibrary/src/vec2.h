#pragma once

#include <iostream>

//definition of a 2d vector object

namespace lge
{

	struct vec2
	{
		double x = 0.0;
		double y = 0.0;

		vec2(double x = 0.0, double y = 0.0) :
			x(x), y(y)
		{}

		friend std::ostream& operator<<(std::ostream& os, const vec2& a)
		{
			return os << "X: " << a.x << " Y: " << a.y;
		}

		vec2 operator-(void) const
		{
			return vec2(-x, -y);
		}

		vec2 operator-(const vec2& a) const
		{
			return vec2(x - a.x, y - a.y);
		}

		vec2 operator-(double a) const
		{
			return vec2(x - a, y - a);
		}

		vec2 operator+(void) const
		{
			return vec2(+x, +y);
		}

		vec2 operator+(const vec2& a) const
		{
			return vec2(x + a.x, y + a.y);
		}

		vec2 operator+(double a) const
		{
			return vec2(x + a, y + a);
		}

		vec2 operator*(const vec2& a) const
		{
			return vec2(x * a.x, y * a.y);
		}

		vec2 operator*(double a) const
		{
			return vec2(x * a, y * a);
		}

		vec2 operator/(const vec2& a) const
		{
			return vec2(x / a.x, y / a.y);
		}

		vec2 operator/(double a) const
		{
			return vec2(x / a, y / a);
		}

		void operator-=(const vec2& a)
		{
			x -= a.x;
			y -= a.y;
		}

		void operator+=(const vec2& a)
		{
			x += a.x;
			y += a.y;
		}

		void operator*=(const double a)
		{
			x *= a;
			y *= a;
		}

		void operator*=(const vec2& a)
		{
			x *= a.x;
			y *= a.y;
		}

		void operator/=(const double a)
		{
			x /= a;
			y /= a;
		}

		void operator/=(const vec2& a)
		{
			x /= a.x;
			y /= a.y;
		}

		bool operator==(const vec2& a)
		{
			return (x == a.x && y == a.y);
		}

		bool operator!=(const vec2& a)
		{
			return (x != a.x && y != a.y);
		}

		vec2 sub(vec2 a)
		{
			x = x - a.x;
			y = y - a.y;
			return vec2(x, y);
		}

		void set(double x_, double y_)
		{
			x = x_;
			y = y_;
		}

		double lenSqr(void) const  //returns length squared of the vector
		{
			return (x * x) + (y * y);
		}

		double len(void) const  //returns lenght of vector
		{
			return std::sqrt(x * x + y * y);
		}

		vec2 normalize(void)  // normalizes the vector
		{
			double l = len();

			if (l == 0) return vec2(0, 0);

			double invLen = 1.0 / l;

			x *= invLen;
			y *= invLen;

			return *this;
		}

		void limit(double limit)  //limits the vector to a certain length
		{
			if (limit > len()) {
				normalize();
				x *= limit;
				y *= limit;
			}
		}
	};
}