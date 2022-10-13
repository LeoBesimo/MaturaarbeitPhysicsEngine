#pragma once

#include <math.h>
#include <cmath>
#include "vec2.h"
#include "vec3.h"
#include "vec4.h"
#include "mat2.h"

namespace lge
{
	//usefull constants for working with radians
	const double PI = (double)atan(1) * 4;
	const double QUARTER_PI = (double) (PI / 4.0);
	const double HALF_PI = (double)(PI / 2.0);
	const double TWO_PI = (double) (PI * 2.0);
	const double THRESHOLD = (double)1.0;// 0.0005;

	struct PLData
	{
		vec2 closest = NULL;
		double distSqr = FLT_MAX;

		PLData() {}

	};

	namespace Color
	{
		static const vec4 WHITE(255, 255, 255, 255);
		static const vec4 RED(255, 0, 0, 255);
		static const vec4 GREEN(0, 255, 0, 255);
		static const vec4 BLUE(0, 0, 255, 255);
		static const vec4 BLACK(0, 0, 0, 255);
		static const vec4 TRANSPARENT(0, 0, 0, 0);
		static const vec4 PINK(255, 0, 255, 255);
		static const vec4 YELLOW(255, 255, 0, 255);
		static const vec4 CYAN(0, 255, 255, 255);
		static const vec4 LIGHTGRAY(160, 160, 160, 255);
		static const vec4 GRAY(64, 64, 64, 255);
		static const vec4 INDIGO(75, 0, 130, 255);
		static const vec4 ORANGE(255, 165, 0, 255);
		static const vec4 BROWN(139, 69, 19,255);
		static const vec4 DARKPURPLE(100, 0, 200, 255);

	}
}