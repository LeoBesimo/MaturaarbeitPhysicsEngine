#include "CollisionResolution.h"

void lge::ResolveCollision(Manifold m, Polygon* poly1, Polygon* poly2)
{
	vec2 rv = poly2->m_velocity - poly1->m_velocity;

	vec2 normal = m.normal[0] + m.normal[1];
	normal /= 2;

	double velAlongNormal = lge::dotVec2(rv, normal);
	std::cout << velAlongNormal << "\n";
	if (velAlongNormal > 0) return;

	double e = min(poly1->m_restitution, poly2->m_restitution);


	double j = -(1 + e) * velAlongNormal;
	std::cout << j << "\n";
	j /= ((1 / poly1->m_mass) + (1 / poly2->m_mass));

	vec2 impulse = normal * j;

	std::cout << impulse;

	poly1->m_velocity -= impulse * (1 / poly1->m_mass);
	poly2->m_velocity += impulse * (1 / poly2->m_mass);
}
