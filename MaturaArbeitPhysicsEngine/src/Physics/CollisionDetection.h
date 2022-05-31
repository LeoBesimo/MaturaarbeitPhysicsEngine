#pragma once
#include "Shapes/Polygon.h"
#include "lgeMath.h"
#include "Manifold.h"

namespace lge
{
	bool PolygonCollisionDiagonals(Polygon *poly1, Polygon *poly2);
	vec2 PolygonCollisionDiagonalsDisplacement(Polygon* poly1, Polygon* poly2);
	void PolygonCollisionDiagonalsApply(Polygon* poly1, Polygon* poly2);

	vec2 LineLineIntersection(vec2 p1, vec2 p2, vec2 p3, vec2 p4);

	std::vector<vec2> getNormals(Polygon* poly);
	std::vector<vec2> getNormalsTrigonometry(Polygon* poly);
	std::vector<vec2> getContactPoints(Polygon* poly1, Polygon* poly2);
	vec4 getMinMax(std::vector<vec2> points, vec2 normal);
	bool PolygonCollisonSat(Polygon* poly1, Polygon* poly2);

	Manifold PolygonCollisionSatManifold(Polygon* poly1, Polygon* poly2);
}