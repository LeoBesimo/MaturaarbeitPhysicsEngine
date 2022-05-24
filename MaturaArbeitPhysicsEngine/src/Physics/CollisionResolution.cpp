#include "CollisionResolution.h"

void lge::ResolveCollision(Manifold m, Polygon* poly1, Polygon* poly2)
{
	vec2 rv = poly2->m_velocity - poly1->m_velocity;

	std::cout << m.normal[0] << " " << m.normal[1] << "\n";

	vec2 normal = m.normal[0] + m.normal[1];
	std::cout << "Normal avg: " << normal << "\n";
	normal /= 2;
	normal = normal.normalize();
	double dotNormal = lge::dotVec2(m.normal[0].normalize(), m.normal[1].normalize());
	if (dotNormal < 0.1 && dotNormal > -0.1)
	{
		std::cout << "using Normal 1 "<< m.normal[1] << " " << m.normal[0] <<"\n";
		normal = m.normal[1].normalize();
		if (normal.lenSqr() == 0)
		{
			std::cout << "using normal 0\n";
			normal = m.normal[0];
		}
	}

	//std::cout << rv << " " << normal << "\n";

	double velAlongNormal = lge::dotVec2(rv, normal);
	//std::cout << velAlongNormal << "\n";
	if (velAlongNormal > 0) return;

	double e = min(poly1->m_restitution, poly2->m_restitution);


	double j = -(1 + e) * velAlongNormal;
	//std::cout << j << "\n";
	j /= ((1 / poly1->m_mass) + (1 / poly2->m_mass));

	vec2 impulse = normal * j;

	//std::cout << impulse;

	poly1->m_velocity -= (impulse * (1 / poly1->m_mass) * !poly1->m_isStatic) + (impulse * (1 / poly2->m_mass) * poly2->m_isStatic);
	poly2->m_velocity += (impulse * (1 / poly2->m_mass) * !poly2->m_isStatic) - (impulse * (1 / poly1->m_mass) * poly1->m_isStatic);
}