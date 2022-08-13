#include "CollisionResolution.h"

void lge::ResolveCollisionWithoutRotation(Manifold m, Polygon* poly1, Polygon* poly2)
{
	if (poly1->m_isStatic + poly2->m_isStatic > 1) return;

	vec2 rv = poly2->m_velocity - poly1->m_velocity;
	vec2 normal = m.normal[0].normalize();

	float velAlongNormal = dotVec2(rv, normal);

	if (velAlongNormal > 0)
	{
		return; // objects are separating so no need for collision resolution;
	}

	double e = min(poly1->m_restitution, poly2->m_restitution);

	double j = -(1 + e) * velAlongNormal;
	j /= ((!poly1->m_isStatic / poly1->m_mass) + (!poly2->m_isStatic / poly2->m_mass));

	

	vec2 impulse = normal * j;

	poly1->m_velocity += (impulse * (1 / poly1->m_mass) * !poly1->m_isStatic) * (poly1->m_isStatic ? 1 : -1);
	poly2->m_velocity += (impulse * (1 / poly2->m_mass) * !poly2->m_isStatic) * (poly2->m_isStatic ? -1 : 1);
}

void lge::ResolveCollision(Manifold m, Polygon* poly1, Polygon* poly2)
{
	vec2 rv = poly2->m_velocity - poly1->m_velocity;
	std::vector<vec2> contactPoints = getContactPoints(poly1, poly2);

	//std::cout << m.normal[0] << " " << m.normal[1] << "\n";

	vec2 normal = m.normal[0] + m.normal[1];
	//std::cout << "Normal avg: " << normal << "\n";
	normal = normal.normalize();
	double dotNormal = lge::dotVec2(m.normal[0].normalize(), m.normal[1].normalize());
	//std::cout << dotNormal << "\n";
	if ((dotNormal < 0.01 && dotNormal > -0.01) || dotNormal == 1)
	{
		dotNormal = 0;

		//std::cout << dotNormal << "\n";

		//std::cout << "using Normal 1 "<< m.normal[1] << " " << m.normal[0] <<"\n";
		normal = m.normal[1].normalize();
		if (normal.lenSqr() == 0)
		{
			//std::cout << "using normal 0\n";
			normal = m.normal[0];
		}
	}

	if (poly1->m_isStatic) normal = m.normal[0].normalize();
	if (poly2->m_isStatic) normal = m.normal[1].normalize();


	//vec2 impulse;
	//bool collideOnce = false;

	//for (unsigned int i = 0; i < 2; i++)
	//{
		//vec2 normal = m.normal[i].normalize();
		//std::cout << rv << " " << normal << "\n";

	double velAlongNormal = lge::dotVec2(rv, normal);
	//std::cout << velAlongNormal << "\n";
	if (velAlongNormal > 0) {
		//collideOnce = true;
		return;
	}

	double e = min(poly1->m_restitution, poly2->m_restitution);


	double j = -(1 + e) * velAlongNormal;
	//std::cout << j << "\n";
	j /= ((1 / poly1->m_mass) + (1 / poly2->m_mass));// + (1 / poly1->m_inertia) + (1 / poly2->m_inertia));

	vec2 impulse = normal * j;

	//std::cout << impulse << "\n";

	poly1->m_velocity -= (impulse * (1 / poly1->m_mass) * !poly1->m_isStatic);// - (impulse * (1 / poly2->m_mass) * poly2->m_isStatic);
	poly2->m_velocity += (impulse * (1 / poly2->m_mass) * !poly2->m_isStatic);// + (impulse * (1 / poly1->m_mass) * poly1->m_isStatic);
	//PositionalCorrection(m, poly1, poly2);
	//std::cout << m.penetration << "\n";
	for (unsigned int j = 0; j < contactPoints.size(); j++)
	{
		if (dotNormal != 0)
		{
			vec2 contactPoint = contactPoints[j];
			poly1->m_angularVelocity += 1.0 / poly1->m_inertia * crossVec2(contactPoint.normalize(), normal.normalize()) / contactPoints.size() * !poly1->m_isStatic;
			poly2->m_angularVelocity += 1.0 / poly2->m_inertia * crossVec2(contactPoint.normalize(), normal.normalize()) / contactPoints.size() * !poly2->m_isStatic;
		}
	}
}

void lge::ResolveCollisionImproved(Manifold m, Polygon* poly1, Polygon* poly2)
{

	if (poly1->m_isStatic + poly2->m_isStatic > 1) return;

	std::vector<vec2> contactPoints = getContactPoints(poly1, poly2);
	//for (unsigned int i = 0; i < 2; i++)
	//{
		//vec2 normal = m.normal[i].normalize();
		//if (poly1->m_isStatic) normal = m.normal[0].normalize();
		//if (poly2->m_isStatic) normal = m.normal[1].normalize();

	for (unsigned int i = 0; i < 2; i++)
	{
		vec2 normal = m.normal[i];
		if (poly1->m_isStatic) normal = m.normal[0];
		if (poly2->m_isStatic) normal = m.normal[1];

		for (unsigned int k = 0; k < contactPoints.size(); k++)
		{
			vec2 ra = (contactPoints[k] - poly1->m_position);
			vec2 rb = (contactPoints[k] - poly2->m_position);

			vec2 crossAngRB = crossVec2Scalar(poly2->m_angularVelocity, rb);
			vec2 crossAngRA = crossVec2Scalar(poly1->m_angularVelocity, ra);

			crossAngRB = crossAngRB.lenSqr() < 0.01 ? lge::vec2() : crossAngRB;
			crossAngRA = crossAngRA.lenSqr() < 0.01 ? lge::vec2() : crossAngRA;

			vec2 totalVelA = poly1->m_velocity + crossAngRA;
			vec2 totalVelB = poly2->m_velocity + crossAngRB;

			//vec2 rv = vec2(totalVelB.x - totalVelA.x, totalVelB.y - totalVelA.y);

			double rx = totalVelB.x - totalVelA.x;
			double ry = totalVelB.y - totalVelA.y;

			//std::cout << -4.0000000000000000 - 5.0000000000000000 << "\n";

			//vec2 rv(rx, ry);
			vec2 rv(0, 0);
			rv.x = rx;
			rv.y = ry;

			std::cout << "rv: " << rv << "\n";
			double contactVel = dotVec2(rv, normal);
			if (contactVel > 0) return;
			double raCrossN = crossVec2(ra, normal);
			double rbCrossN = crossVec2(rb, normal);
			double invMassSum = (!poly1->m_isStatic / poly1->m_mass) + (!poly2->m_isStatic / poly2->m_mass) + (raCrossN * raCrossN * (!poly1->m_isStatic / poly1->m_inertia)) + (rbCrossN * rbCrossN * (!poly2->m_isStatic / poly2->m_inertia));

			double e = min(poly1->m_restitution, poly2->m_restitution);

			double j = -(1.0f + e) * contactVel;
			j /= invMassSum;
			j /= contactPoints.size();
			//j /= 2;

			vec2 impulse = normal * j;
			//impulse = i == 0 ? impulse : -impulse;
			std::cout << "\n";
			poly1->m_velocity -= (impulse * (1 / poly1->m_mass) * !poly1->m_isStatic);// + (impulse * (1 / poly2->m_mass) * poly2->m_isStatic);
			poly2->m_velocity += (impulse * (1 / poly2->m_mass) * !poly2->m_isStatic);// -(impulse * (1 / poly1->m_mass) * poly1->m_isStatic);

			poly1->m_angularVelocity += 1.0 / poly1->m_inertia * crossVec2(ra, -impulse) * !poly1->m_isStatic;
			poly2->m_angularVelocity += 1.0 / poly2->m_inertia * crossVec2(rb, impulse) * !poly2->m_isStatic;

		}
	}
}

void lge::ResolveCollisionImprovedNormalized(Manifold m, Polygon* poly1, Polygon* poly2)
{
	std::vector<vec2> contactPoints = getContactPoints(poly1, poly2);
	lge::vec2 normal = m.normal[0].normalize();
	
	if (poly1->m_isStatic + poly2->m_isStatic > 1) return;

	//vec2 vc = contactPoints[1] - contactPoints[0];
	//vec2 normal(-vc.y, vc.x);
	//normal = normal.normalize();
	
	for (auto i = 0; i < contactPoints.size(); i++)
	{
		vec2 ra = contactPoints[i] - poly1->m_position;
		vec2 rb = contactPoints[i] - poly2->m_position;
		vec2 va = poly1->m_velocity + crossVec2Scalar(poly1->m_angularVelocity, ra);
		vec2 vb = poly2->m_velocity + crossVec2Scalar(poly2->m_angularVelocity, rb);

		vec2 rv = vb - va;

		if (dotVec2(rv, normal) > 0) return;

		double e = min(poly1->m_restitution, poly2->m_restitution);
		double invMass = !poly1->m_isStatic / poly1->m_mass + !poly2->m_isStatic / poly2->m_mass;

		double den1 = dotVec2(normal, crossVec2Scalar(crossVec2(va, normal) * !poly1->m_isStatic / poly1->m_inertia, va)); //dotVec2(normal, crossVec2Scalar(crossVec2(va, normal),va) / poly1->m_inertia);
		double den2 = dotVec2(normal, crossVec2Scalar(crossVec2(vb, normal) * !poly2->m_isStatic / poly2->m_inertia, vb));

		double j = -(dotVec2(rv, normal) * (e + 1)) / (invMass + den1 + den2);//dotVec2(normal, crossVec2(va, normal) / poly1->m_inertia) + dotVec2(normal, crossVec2(vb, normal) / poly1->m_inertia));


		poly1->m_velocity += (normal * j) / poly1->m_mass;
		poly2->m_velocity -= (normal * j) / poly2->m_mass;

		poly1->m_angularVelocity += (crossVec2(va, normal * j) / poly1->m_inertia);
		poly2->m_angularVelocity -= (crossVec2(vb, normal * j) / poly2->m_inertia);
	}
}


//TODO: Rework this
void lge::PositionalCorrection(Manifold m, Polygon* poly1, Polygon* poly2)
{
	const double percent = 0.2;
	vec2 normal = m.normal[0] + m.normal[1];
	normal = normal.normalize();
	double dotNormal = lge::dotVec2(m.normal[0].normalize(), m.normal[1].normalize());
	if ((dotNormal < 0.01 && dotNormal > -0.01) || dotNormal == 1)
	{
		dotNormal = 0;
		normal = m.normal[1].normalize();
		if (normal.lenSqr() == 0)
		{
			normal = m.normal[0];
		}
	}

	if (poly1->m_isStatic) normal = m.normal[0];
	if (poly2->m_isStatic) normal = m.normal[1];

	normal = normal.normalize();

	vec2 correction = normal * percent * (-m.penetration / (1 / poly1->m_mass + 1 / poly2->m_mass));
	poly1->m_position -= correction * 1 / poly1->m_mass * !poly1->m_isStatic;
	poly2->m_position += correction * 1 / poly2->m_mass * !poly2->m_isStatic;
	poly1->updateSides();
	poly2->updateSides();

}

void lge::ApplyImpulse(Polygon* poly, vec2 impulse, std::vector<vec2> contactPoints)
{
	poly->m_velocity += (impulse * (1 / poly->m_mass) * !poly->m_isStatic);// + (impulse * (1 / poly2->m_mass) * poly2->m_isStatic);

	vec2 normal = impulse.normalize();
	for (unsigned int j = 0; j < contactPoints.size(); j++)
	{
		vec2 contactPoint = contactPoints[j];
		poly->m_angularVelocity += 1.0 / poly->m_inertia * crossVec2(contactPoint, normal) / contactPoints.size() * !poly->m_isStatic;
		//poly2->m_angularVelocity += 1.0 / poly2->m_inertia * crossVec2(contactPoint, normal) / contactPoints.size() * !poly2->m_isStatic;
	}
}

void lge::ApplyImpulseImproved(Polygon* poly, vec2 impulse, vec2 contactPoint)
{
	poly->m_velocity += impulse * (1 / poly->m_mass) * !poly->m_isStatic;
	poly->m_velocity += (1 / poly->m_inertia) * crossVec2(contactPoint, impulse) * !poly->m_isStatic;
}
