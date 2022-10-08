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
}