#pragma once

#include "components.h"

namespace lge
{
	struct Manifold
	{
		bool collided = false;
		vec2 normal[2];
		double penetration;
		int normalIndex[2];
		int indexP1[2];
		int indexP2[2];
	};

}