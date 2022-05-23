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

			poly1->m_position += displacement * poly1->m_scale * (shape == 1 ? -1 : 1);
			poly1->update();
		}
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

lge::vec4 lge::getMinMax(std::vector<vec2> points, vec2 normal)
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

	return vec4(minProj,maxProj,minIndex,maxIndex);
}

bool lge::PolygonCollisonSat(Polygon* poly1, Polygon* poly2)
{

	std::vector<vec2> normalsPoly1 = getNormals(poly1);
	std::vector<vec2> normalsPoly2 = getNormals(poly2);

	bool separated = false;

	for (unsigned int i = 0; i < normalsPoly1.size(); i++)
	{
		vec4 projectionPoly1 = getMinMax(poly1->m_transformedPoints, normalsPoly1[i]);
		vec4 projectionPoly2 = getMinMax(poly2->m_transformedPoints, normalsPoly1[i]);

		separated = projectionPoly1.x < projectionPoly2.w || projectionPoly2.x < projectionPoly1.w;
		if (separated) break;
	}

	if (!separated)
	{
		for (unsigned int i = 0; i < normalsPoly2.size(); i++)
		{
			vec4 projectionPoly1 = getMinMax(poly1->m_transformedPoints, normalsPoly2[i]);
			vec4 projectionPoly2 = getMinMax(poly2->m_transformedPoints, normalsPoly2[i]);

			separated = projectionPoly1.x < projectionPoly2.w || projectionPoly2.x < projectionPoly1.w;
			if (separated) break;
		}
	}
	

	if (!separated) std::cout << "colliding\n";
	else std::cout << "not colliding\n";
	return separated;
}

lge::Manifold lge::PolygonCollisionSatManifold(Polygon* poly1, Polygon* poly2)
{

	Manifold m;

	std::vector<vec2> normalsPoly1 = getNormals(poly1);
	std::vector<vec2> normalsPoly2 = getNormals(poly2);

	bool separated = false;

	vec2 minNormal1;
	double minPenetration1 = FLT_MAX;
	double minPenetration2 = FLT_MAX;
	vec2 minNormal2;

	for (unsigned int i = 0; i < normalsPoly1.size(); i++)
	{
		vec4 projectionPoly1 = getMinMax(poly1->m_transformedPoints, normalsPoly1[i]);
		vec4 projectionPoly2 = getMinMax(poly2->m_transformedPoints, normalsPoly1[i]);

		separated = projectionPoly1.x < projectionPoly2.w || projectionPoly2.x < projectionPoly1.w;
		if (separated) break;

		double penetration = projectionPoly2.w - projectionPoly1.x;
		if (penetration < minPenetration1)
		{
			minPenetration1 = penetration;
			minNormal1 = normalsPoly1[i];
			m.indexP1[0] = projectionPoly1.y;
			m.indexP1[1] = projectionPoly1.z;
			m.normalIndex[0] = i + floor(poly1->m_transformedPoints.size() / 2);
		}
	}

	if (!separated)
	{
		for (unsigned int i = 0; i < normalsPoly2.size(); i++)
		{
			vec4 projectionPoly1 = getMinMax(poly1->m_transformedPoints, normalsPoly2[i]);
			vec4 projectionPoly2 = getMinMax(poly2->m_transformedPoints, normalsPoly2[i]);


			separated = projectionPoly1.x < projectionPoly2.w || projectionPoly2.x < projectionPoly1.w;
			if (separated) break;

			double penetration = projectionPoly2.w - projectionPoly1.x;
			if (penetration < minPenetration2)
			{
				minPenetration2 = penetration;
				minNormal2 = normalsPoly2[i];
				m.indexP2[0] = projectionPoly2.y;
				m.indexP2[1] = projectionPoly2.z;
				m.normalIndex[1] = i + floor(poly2->m_transformedPoints.size()/2);
			}

		}
	}


	if (!separated)
	{
		//std::cout << "colliding\n";
		double minPenetration = minPenetration1 < minPenetration2 ? minPenetration1 : minPenetration2;
		m.penetration = minPenetration;
		m.normal[0] = (minNormal1).normalize() * -1;// - poly1->m_position).normalize();
		m.normal[1] = (minNormal2).normalize() * -1;// - poly2->m_position).normalize();
		m.collided = true;
	}
	//else std::cout << "not colliding\n";

	//m.collided != separated;
	

	return m;
}
