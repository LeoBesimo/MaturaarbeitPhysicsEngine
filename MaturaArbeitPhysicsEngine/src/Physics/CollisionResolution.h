#pragma once
#include "Manifold.h"
#include "Shapes/Polygon.h"
#include "Utilities.h"
#include "CollisionDetection.h"

namespace lge
{

	template<class T>
	struct CollisionData
	{
		CollisionData() {};

		bool separating[2];

		int contactCount;

		T contactPoints[2];
		T relativeVel[2];
		double contactVel[2];
		T rv[2];
		T ra[2];
		T rb[2];
		T velA[2];
		T velB[2];
		double invInert[2];
		double invMass;
		double jBefore[2];
		double jAfter[2];
		T impulse[2];
		double eKin[2];
		double eKinAfter[2];
		double eRot[2];
		double eRotAfter[2];
		T linImpulse[2];
		T linImpulseAfter[2];
		T rotImpulse[2];
		T rotImpulseAfter[2];
	};

	void ResolveCollisionWithoutRotation(Manifold m, Polygon* poly1, Polygon* poly2);
	void ResolveCollision(Manifold m, Polygon* poly1, Polygon* poly2);
	CollisionData<vec2> [[nodiscard]] ResolveCollisionCollisionData(Manifold m, Polygon* poly1, Polygon* poly2);
	void ResolveCollisionImproved(Manifold m, Polygon* poly1, Polygon* poly2);
	CollisionData<vec2> [[nodiscard]] ResolveCollisionImprovedCollisionData(Manifold m, Polygon* poly1, Polygon* poly2);
	void ResolveCollisionCombined(Manifold m, Polygon* poly1, Polygon* poly2);
	CollisionData<vec2> [[nodiscard]] ResolveCollisionCombinedCollisonData(Manifold m, Polygon* poly1, Polygon* poly2);
	void ResolveCollision3D(Manifold m, Polygon* poly1, Polygon* poly2);
	CollisionData<vec3> [[nodiscard]] ResolveCollision3DCollisionData(Manifold m, Polygon* poly1, Polygon* poly2);
	void PositionalCorrection(Manifold m, Polygon* poly1, Polygon* poly2);
	void ApplyImpulse(Polygon* poly, vec2 impulse, std::vector<vec2> contactPoints);
	void ApplyImpulseImproved(Polygon* poly, vec2 impulse, vec2 contactPoint);
}
