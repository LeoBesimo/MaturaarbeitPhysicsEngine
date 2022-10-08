#pragma once

#include "components.h"

//define structure to hold collision information

namespace lge
{
	struct Manifold
	{
		bool collided = false;
		vec2 normal[2];
		double penetration;
	};

}