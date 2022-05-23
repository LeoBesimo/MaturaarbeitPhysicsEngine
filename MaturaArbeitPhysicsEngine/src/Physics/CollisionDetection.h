#pragma once
#include "Shapes/Polygon.h"
#include "lgeMath.h"
#include "Manifold.h"

namespace lge
{
	bool PolygonCollisionDiagonals(Polygon *poly1, Polygon *poly2);
	vec2 PolygonCollisionDiagonalsDisplacement(Polygon* poly1, Polygon* poly2);
	void PolygonCollisionDiagonalsApply(Polygon* poly1, Polygon* poly2);


	std::vector<vec2> getNormals(Polygon* poly);
	vec4 getMinMax(std::vector<vec2> points, vec2 normal);
	bool PolygonCollisonSat(Polygon* poly1, Polygon* poly2);

	Manifold PolygonCollisionSatManifold(Polygon* poly1, Polygon* poly2);
}