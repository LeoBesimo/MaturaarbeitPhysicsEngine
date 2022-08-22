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

		bool separating = false;

		std::vector<vec2> contactPoints;
		std::vector<double> contactVel;
		std::vector<vec2> rv;
		std::vector<vec2> ra;
		std::vector<vec2> rb;
		std::vector<vec2> velA;
		std::vector<vec2> velB;
		std::vector<double> invInert;
		double invMass;
		std::vector<double> jBefore;
		std::vector<double> jAfter;
		std::vector<vec2> impulse;
	};

	void ResolveCollisionWithoutRotation(Manifold m, Polygon* poly1, Polygon* poly2);
	void ResolveCollision(Manifold m, Polygon* poly1, Polygon* poly2);
	CollisionData ResolveCollisionCollisionData(Manifold m, Polygon* poly1, Polygon* poly2);
	void ResolveCollisionImproved(Manifold m, Polygon* poly1, Polygon* poly2);
	void ResolveCollisionImprovedNormalized(Manifold m, Polygon* poly1, Polygon* poly2);
	void PositionalCorrection(Manifold m, Polygon* poly1, Polygon* poly2);
	void ApplyImpulse(Polygon* poly, vec2 impulse, std::vector<vec2> contactPoints);
	void ApplyImpulseImproved(Polygon* poly, vec2 impulse, vec2 contactPoint);
}
