#pragma once
#include "Shapes/Polygon.h"
#include "lgeMath.h"
#include "Manifold.h"

namespace lge
{

	bool PolygonCollisionDiagonals(Polygon *poly1, Polygon *poly2);  //collision detection check with radii against sides
	vec2 PolygonCollisionDiagonalsDisplacement(Polygon* poly1, Polygon* poly2);
	void PolygonCollisionDiagonalsApply(Polygon* poly1, Polygon* poly2);

	bool AABBCollision(Polygon* poly1, Polygon* poly2);

	vec2 LineLineIntersection(vec2 p1, vec2 p2, vec2 p3, vec2 p4);  //checking for line intersections with intersection point calculation

	std::vector<vec2> getNormals(Polygon* poly); //returns side normals for a given polygon
	std::vector<vec2> getNormalsTrigonometry(Polygon* poly);  //redundant
	std::vector<vec2> getContactPoints(Polygon* poly1, Polygon* poly2); // calculates contact points between polygons using line intersection
	vec4 getMinMax(std::vector<vec2> points, vec2 normal);  //returns the minimal and maximal projection for a list of points onto a 2d vector
	bool PolygonCollisonSat(Polygon* poly1, Polygon* poly2);  //checks for collision between polygons using projections onto the surface normals

	Manifold PolygonCollisionSatManifold(Polygon* poly1, Polygon* poly2); //checks for collision between polygons using projections onto the surface normals, additionally generates contact data
}