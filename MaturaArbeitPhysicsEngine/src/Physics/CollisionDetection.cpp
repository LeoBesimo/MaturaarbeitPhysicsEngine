#include "CollisionDetection.h"

bool lge::PolygonCollisionDiagonals(Polygon *poly1, Polygon *poly2)
{

	Polygon p1 = *poly1;
	Polygon p2 = *poly2;

	for (unsigned int shape = 0; shape < 2; shape++)
	{
		if (shape == 1)
		{
			p1 = *poly2;
			p2 = *poly1;
		}

		std::vector<lge::vec2> pointsPoly1 = p1.m_transformedPoints;
		std::vector<lge::vec2> pointsPoly2 = p2.m_transformedPoints;

		for (unsigned int i = 0; i < pointsPoly1.size(); i++)
		{
			double x1 = p1.m_position.x;
			double y1 = p1.m_position.y;

			double x2 = pointsPoly1[i].x;
			double y2 = pointsPoly1[i].y;

			for (unsigned int j = 0; j < pointsPoly2.size(); j++)
			{
				double x3 = pointsPoly2[j].x;
				double y3 = pointsPoly2[j].y;

				double x4 = pointsPoly2[(j + 1) % pointsPoly2.size()].x;
				double y4 = pointsPoly2[(j + 1) % pointsPoly2.size()].y;

				double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

				double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator;
				double u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / denominator;

				if (0 <= t && t < 1 && 0 <= u && u < 1) {
					std::cout << "overlapping\n";
					return true;
				}
			}
		}
	}
	std::cout << "not overlapping\n";
	return false;
}

lge::vec2 lge::PolygonCollisionDiagonalsDisplacement(Polygon* poly1, Polygon* poly2)
{

	Polygon p1 = *poly1;
	Polygon p2 = *poly2;

	lge::vec2 displacement;

	for (unsigned int shape = 0; shape < 2; shape++)
	{
		if (shape == 1)
		{
			p1 = *poly2;
			p2 = *poly1;
		}

		std::vector<lge::vec2> pointsPoly1 = p1.m_transformedPoints;
		std::vector<lge::vec2> pointsPoly2 = p2.m_transformedPoints;

		for (unsigned int i = 0; i < pointsPoly1.size(); i++)
		{
			double x1 = p1.m_position.x;
			double y1 = p1.m_position.y;

			double x2 = pointsPoly1[i].x;
			double y2 = pointsPoly1[i].y;

			for (unsigned int j = 0; j < pointsPoly2.size(); j++)
			{
				double x3 = pointsPoly2[j].x;
				double y3 = pointsPoly2[j].y;

				double x4 = pointsPoly2[(j + 1) % pointsPoly2.size()].x;
				double y4 = pointsPoly2[(j + 1) % pointsPoly2.size()].y;

				double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

				double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator;
				double u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / denominator;

				if (0 <= t && t < 1 && 0 <= u && u < 1) 
				{
					displacement.x += (1.0 - u) * (x2 - x1) * (shape == 0 ? -1 : 1);
					displacement.y += (1.0 - u) * (y2 - y1) * (shape == 0 ? -1 : 1);
				}
			}
		}
	}
	return displacement;
}

void lge::PolygonCollisionDiagonalsApply(Polygon* poly1, Polygon* poly2)
{
	Polygon p1 = *poly1;
	Polygon p2 = *poly2;

	for (unsigned int shape = 0; shape < 2; shape++)
	{
		if (shape == 1)
		{
			p1 = *poly2;
			p2 = *poly1;
		}

		std::vector<lge::vec2> pointsPoly1 = p1.m_transformedPoints;
		std::vector<lge::vec2> pointsPoly2 = p2.m_transformedPoints;

		for (unsigned int i = 0; i < pointsPoly1.size(); i++)
		{
			double x1 = p1.m_position.x;
			double y1 = p1.m_position.y;

			double x2 = pointsPoly1[i].x;
			double y2 = pointsPoly1[i].y;

			vec2 displacement;

			for (unsigned int j = 0; j < pointsPoly2.size(); j++)
			{
				double x3 = pointsPoly2[j].x;
				double y3 = pointsPoly2[j].y;

				double x4 = pointsPoly2[(j + 1) % pointsPoly2.size()].x;
				double y4 = pointsPoly2[(j + 1) % pointsPoly2.size()].y;

				double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

				double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator;
				double u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / denominator;

				if (0 <= t && t < 1 && 0 <= u && u < 1)
				{
					displacement.x += (0 - u) * (x2 - x1);
					displacement.y += (0 - u) * (y2 - y1);
				}
			}

			poly1->m_position += displacement * vec2(poly1->m_scale.x.x, poly1->m_scale.y.y) * (shape == 1 ? -1 : 1);
			poly1->update(1);
		}
	}
}

bool lge::AABBCollision(Polygon* poly1, Polygon* poly2)
{

	double h1 = INFINITY;
	double h2 = - INFINITY;
	double w1 = INFINITY;
	double w2 = - INFINITY;

	double h3 = INFINITY;
	double h4 = - INFINITY;
	double w3 = INFINITY;
	double w4 = -INFINITY;

	for (auto i = 0; i < poly1->m_transformedPoints.size(); i++)
	{
		if (poly1->m_transformedPoints[i].y < h1) h1 = poly1->m_transformedPoints[i].y;
		if (poly1->m_transformedPoints[i].y > h2) h2 = poly1->m_transformedPoints[i].y;
		if (poly1->m_transformedPoints[i].x < w1) w1 = poly1->m_transformedPoints[i].x;
		if (poly1->m_transformedPoints[i].x > w2) w2 = poly1->m_transformedPoints[i].x;
	}

	for (auto i = 0; i < poly2->m_transformedPoints.size(); i++)
	{
		if (poly2->m_transformedPoints[i].y < h3) h3 = poly2->m_transformedPoints[i].y;
		if (poly2->m_transformedPoints[i].y > h4) h4 = poly2->m_transformedPoints[i].y;
		if (poly2->m_transformedPoints[i].x < w3) w3 = poly2->m_transformedPoints[i].x;
		if (poly2->m_transformedPoints[i].x > w4) w4 = poly2->m_transformedPoints[i].x;
	}

	bool a1 = h1 > h4;
	bool a2 = h2 < h3;
	bool a3 = w1 > w4;
	bool a4 = w2 < w3;

	bool separated = a1 | a2 | a3 | a4;

	return !separated;
}

lge::vec2 lge::LineLineIntersection(vec2 p1, vec2 p2, vec2 p3, vec2 p4)
{
	float x1 = p1.x;
	float y1 = p1.y;
	float x2 = p2.x;
	float y2 = p2.y;

	float x3 = p3.x;
	float y3 = p3.y;
	float x4 = p4.x;
	float y4 = p4.y;

	float den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
	if (den == 0) {
		return vec2();
	}

	float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
	float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;
	if (0 < t && t < 1 && 0 < u && u < 1) {
		vec2 pt;
		pt.x = x1 + t * (x2 - x1);
		pt.y = y1 + t * (y2 - y1);
		return pt;
	}
	else {
		return vec2();
	}
}

std::vector<lge::vec2> lge::getNormals(Polygon* poly)
{
	std::vector<vec2> normals;
	std::vector<vec2> points = poly->m_transformedPoints;

	for (unsigned int i = 0; i < points.size(); i++)
	{
		unsigned int j = (i + 1) % points.size();
		vec2 edge = points[j] - points[i];
		normals.push_back(vec2(-edge.y, edge.x));
	}

	return normals;
}

std::vector<lge::vec2> lge::getContactPoints(Polygon* poly1, Polygon* poly2)
{
	std::vector<vec2> contactPoints;
	
	std::vector<vec2> pointsA = poly1->m_transformedPoints;
	std::vector<vec2> pointsB = poly2->m_transformedPoints;

	double minDistSqr = FLT_MAX;

	unsigned int contactCount = 0;
	vec2 contact1 = NULL;
	vec2 contact2 = NULL;

	for (unsigned int i = 0; i < pointsA.size(); i++)
	{

		vec2 p = pointsA[i];

		for (unsigned int j = 0; j < pointsB.size(); j++)
		{
			vec2 a = pointsB[j];
			vec2 b = pointsB[(j + 1) % pointsB.size()];

			PLData data = distPointToLine(p, a, b);

			if (nearlyEqual(data.distSqr,minDistSqr))
			{
				if(!nearlyEqual(data.closest,contact1))
				{
					contactCount = 2;
					contact2 = data.closest;
				}
			}
			else if (data.distSqr < minDistSqr)
			{
				minDistSqr = data.distSqr;
				contactCount = 1;
				contact1 = data.closest;
			}
		}

	}

	for (unsigned int i = 0; i < pointsB.size(); i++)
	{

		vec2 p = pointsB[i];

		for (unsigned int j = 0; j < pointsA.size(); j++)
		{
			vec2 a = pointsA[j];
			vec2 b = pointsA[(j + 1) % pointsA.size()];

			PLData data = distPointToLine(p, a, b);

			if (nearlyEqual(data.distSqr, minDistSqr))
			{
				if (!nearlyEqual(data.closest, contact1))
				{
					contactCount = 2;
					contact2 = data.closest;
				}
			}
			else if (data.distSqr < minDistSqr)
			{
				minDistSqr = data.distSqr;
				contactCount = 1;
				contact1 = data.closest;
			}
		}
	}

	contactPoints.push_back(contact1);
	if (contactCount == 2) contactPoints.push_back(contact2);
	
	/*
	int size1 = poly1->m_transformedPoints.size();
	int size2 = poly2->m_transformedPoints.size();

	for (unsigned int i = 0; i < size1; i++)
	{
		for (unsigned int j = 0; j < size2; j++)
		{
			vec2 contact = LineLineIntersection(poly1->m_transformedPoints[i], poly1->m_transformedPoints[(i + 1) % size1], poly2->m_transformedPoints[j], poly2->m_transformedPoints[(j + 1) % size2]);
			if (contact.lenSqr() != 0) contactPoints.push_back(contact);
		}
	}*/

	return contactPoints;
}

lge::vec2 lge::getMinMax(std::vector<vec2> points, vec2 normal)
{

	unsigned int minIndex = 0;
	unsigned int maxIndex = 0;
	double minProj = dotVec2(points[0], normal);
	double maxProj = dotVec2(points[0], normal);

	for (unsigned int i = 1; i < points.size(); i++)
	{
		double currentProj = dotVec2(points[i], normal);
		if (currentProj < minProj)
		{
			minIndex = i;
			minProj = currentProj;
		}

		if (currentProj > maxProj)
		{
			maxIndex = i;
			maxProj = currentProj;
		}
	}

	return vec2(minProj,maxProj);
}

bool lge::PolygonCollisonSat(Polygon* poly1, Polygon* poly2)
{

	std::vector<vec2> normalsPoly1 = getNormals(poly1);
	std::vector<vec2> normalsPoly2 = getNormals(poly2);

	bool separated = false;

	for (unsigned int i = 0; i < normalsPoly1.size(); i++)
	{
		vec2 projectionPoly1 = getMinMax(poly1->m_transformedPoints, normalsPoly1[i]);
		vec2 projectionPoly2 = getMinMax(poly2->m_transformedPoints, normalsPoly1[i]);

		separated = projectionPoly1.x >= projectionPoly2.y || projectionPoly2.x >= projectionPoly1.y;
		if (separated) return false;
	}

	if (!separated)
	{
		for (unsigned int i = 0; i < normalsPoly2.size(); i++)
		{
			vec2 projectionPoly1 = getMinMax(poly1->m_transformedPoints, normalsPoly2[i]);
			vec2 projectionPoly2 = getMinMax(poly2->m_transformedPoints, normalsPoly2[i]);

			separated = projectionPoly1.x >= projectionPoly2.y || projectionPoly2.x >= projectionPoly1.y;
			if (separated) return false;
		}
	}
	

	if (!separated) std::cout << "colliding\n";
	else std::cout << "not colliding\n";
	return separated;
}

lge::Manifold lge::PolygonCollisionSatManifold(Polygon* poly1, Polygon* poly2)
{
	Manifold m;

	if (poly1->m_invMass + poly2->m_invMass == 0) return m;

	std::vector<vec2> normalsPoly1 = getNormals(poly1);
	std::vector<vec2> normalsPoly2 = getNormals(poly2);

	bool separated = false;
	
	vec2 normal;
	double minDepth = FLT_MAX;
	
	for (unsigned int i = 0; i < normalsPoly1.size(); i++)
	{
		vec2 projectionPoly1 = getMinMax(poly1->m_transformedPoints, normalsPoly1[i]);
		vec2 projectionPoly2 = getMinMax(poly2->m_transformedPoints, normalsPoly1[i]);

		separated = projectionPoly1.x >= projectionPoly2.y || projectionPoly2.x >= projectionPoly1.y;
		if (separated) break;

		float depth = lge::min(projectionPoly2.y - projectionPoly1.x, projectionPoly1.y - projectionPoly2.x);

		if (depth < minDepth)
		{
			minDepth = depth;
			normal = normalsPoly1[i];
		}
	}

	if (!separated)
	{
		for (unsigned int i = 0; i < normalsPoly2.size(); i++)
		{
			vec2 projectionPoly1 = getMinMax(poly1->m_transformedPoints, normalsPoly2[i]);
			vec2 projectionPoly2 = getMinMax(poly2->m_transformedPoints, normalsPoly2[i]);


			separated = projectionPoly1.x >= projectionPoly2.y || projectionPoly2.x >= projectionPoly1.y;
			if (separated) break;

			float depth = lge::min(projectionPoly2.y - projectionPoly1.x, projectionPoly1.y - projectionPoly2.x);

			if (depth < minDepth)
			{
				minDepth = depth;
				normal = normalsPoly2[i];
			}
		}
	}


	if (!separated)
	{
		vec2 ab = poly2->m_position - poly1->m_position;

		if (dotVec2(ab, normal) < 0) normal *= -1;

		m.penetration = minDepth / normal.len();
		m.normal = normal.normalize();
		m.collided = true;
	}
	
	return m;
}
