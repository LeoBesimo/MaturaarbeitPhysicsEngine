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
			poly1->update();
		}
	}
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

std::vector<lge::vec2> lge::getNormalsTrigonometry(Polygon* poly)
{
	std::vector<vec2> normals;

	double increment = lge::TWO_PI / poly->m_transformedPoints.size();
	for (double i = 0; i < lge::TWO_PI; i += increment)
	{
		vec2 normal(cos(i + (increment / 2) + poly->m_angle), sin(i + (increment / 2) + poly->m_angle));
		normal += poly->m_position;
		normals.push_back(normal);
	}

	//normals = multVec2ToVec2List(normals, vec2(poly->m_scale.x.x, poly->m_scale.y.y));

	return normals;
}

std::vector<lge::vec2> lge::getContactPoints(Polygon* poly1, Polygon* poly2)
{
	std::vector<vec2> contactPoints;

	int size1 = poly1->m_transformedPoints.size();
	int size2 = poly2->m_transformedPoints.size();

	for (unsigned int i = 0; i < size1; i++)
	{
		for (unsigned int j = 0; j < size2; j++)
		{
			vec2 contact = LineLineIntersection(poly1->m_transformedPoints[i], poly1->m_transformedPoints[(i + 1) % size1], poly2->m_transformedPoints[j], poly2->m_transformedPoints[(j + 1) % size2]);
			if (contact.lenSqr() != 0) contactPoints.push_back(contact);
		}
	}

	return contactPoints;
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

		//std::cout << minNormal1.normalize() << " " << minNormal2.normalize() << "\n";
		double minPenetration = minPenetration1 < minPenetration2 ? minPenetration1 : minPenetration2;

		vec2 normalAvg = (minNormal1 + minNormal2) / 2;
		minPenetration = minPenetration / minNormal1.len() / minNormal2.len();//(normalAvg.normalize() * minPenetration).len();
		//std::cout << minPenetration << "\n";
		m.penetration = minPenetration;
		m.normal[0] = (minNormal1).normalize() * -1;// - poly1->m_position).normalize();
		m.normal[1] = (minNormal2).normalize() * -1;// - poly2->m_position).normalize();
		m.collided = true;
	}
	//else std::cout << "not colliding\n";

	//m.collided != separated;
	

	return m;
}
