#pragma once
#include "Manifold.h"
#include "Shapes/Polygon.h"
#include "Utilities.h"
#include "CollisionDetection.h"

namespace lge
{

	struct CollisionData
	{
		CollisionData() {};

		bool separating[2];

		int contactCount;

		vec2 contactPoints[2];
		vec2 relativeVel[2];
		double contactVel[2];
		vec2 rv[2];
		vec2 ra[2];
		vec2 rb[2];
		vec2 velA[2];
		vec2 velB[2];
		double invInert[2];
		double invMass;
		double jBefore[2];
		double jAfter[2];
		vec2 impulse[2];
	};

	void ResolveCollisionWithoutRotation(Manifold m, Polygon* poly1, Polygon* poly2);
	void ResolveCollision(Manifold m, Polygon* poly1, Polygon* poly2);
	CollisionData ResolveCollisionCollisionData(Manifold m, Polygon* poly1, Polygon* poly2);
	void ResolveCollisionImproved(Manifold m, Polygon* poly1, Polygon* poly2);
	CollisionData ResolveCollisionImprovedCollisionData(Manifold m, Polygon* poly1, Polygon* poly2);
	void ResolveCollisionCombined(Manifold m, Polygon* poly1, Polygon* poly2);
	void PositionalCorrection(Manifold m, Polygon* poly1, Polygon* poly2);
	void ApplyImpulse(Polygon* poly, vec2 impulse, std::vector<vec2> contactPoints);
	void ApplyImpulseImproved(Polygon* poly, vec2 impulse, vec2 contactPoint);
}
