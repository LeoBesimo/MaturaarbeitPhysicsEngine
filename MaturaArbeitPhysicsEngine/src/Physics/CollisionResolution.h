#pragma once
#include "Manifold.h"
#include "Shapes/Polygon.h"
#include "Utilities.h"
#include "CollisionDetection.h"

namespace lge
{
	void ResolveCollision(Manifold m, Polygon* poly1, Polygon* poly2);
	void PositionalCorrection(Manifold m, Polygon* poly1, Polygon* poly2);
	void ApplyImpulse(Polygon* poly, vec2 impulse, std::vector<vec2> contactPoints);
}
