#pragma once
#include "Manifold.h"
#include "Shapes/Polygon.h"
#include "Utilities.h"

namespace lge
{
	void ResolveCollision(Manifold m, Polygon* poly1, Polygon* poly2);
}
