#include "CollisionResolution.h"

void lge::ResolveCollisionWithoutRotation(Manifold m, Polygon* poly1, Polygon* poly2)
{
	if (poly1->m_invMass + poly2->m_invMass == 0) return;

	vec2 rv = poly2->m_velocity - poly1->m_velocity;
	vec2 normal = m.normal;

	float velAlongNormal = dotVec2(rv, normal);

	if (velAlongNormal > 0)
	{
		return; // objects are separating so no need for collision resolution;
	}

	double e = min(poly1->m_restitution, poly2->m_restitution);

	double j = -(1 + e) * velAlongNormal;
	j /= (poly1->m_invMass + poly2->m_invMass);

	vec2 impulse = normal * j;

	poly1->m_velocity -= (impulse * poly1->m_invMass);
	poly2->m_velocity += (impulse * poly2->m_invMass);

	//poly1->move(-m.normal[0] * m.penetration / 2);
	//poly2->move(m.normal[0] * m.penetration / 2);
}

void lge::ResolveCollision(Manifold m, Polygon* poly1, Polygon* poly2)
{
	if (poly1->m_invMass + poly2->m_invMass == 0) return;

	std::vector<vec2> contactPoints = getContactPoints(poly1, poly2);

	vec2 normal = m.normal;

	vec2 contact = contactPoints[0];

	if (contactPoints.size() > 1)
	{
		contact = (contactPoints[0] + contactPoints[1]) / 2;
	}
	
	vec2 ra = contact - poly1->m_position;
	vec2 rb = contact - poly2->m_position;

	vec2 raPerp = vec2(-ra.y, ra.x);
	vec2 rbPerp = vec2(-rb.y, rb.x);

	vec2 angLineraVelA = raPerp * poly1->m_angularVelocity;
	vec2 angLinearVelB = rbPerp * poly2->m_angularVelocity;

	vec2 relativeVelocity = (poly2->m_velocity + angLinearVelB) - (poly1->m_velocity + angLineraVelA);

	double contactVelocityMag = dotVec2(relativeVelocity, normal);

	if (contactVelocityMag > 0)
	{
		return;
	}

	double invMass = poly1->m_invMass + poly2->m_invMass;
	
	//double invInert = crossVec2((normal * (crossVec2(ra, normal) * poly1->m_invInertia)), ra) + crossVec2((normal * (crossVec2(rb, normal) * poly2->m_invInertia)), rb);
	double invInert1 = dotVec2(normal, crossVec2(poly1->m_invInertia * crossVec2(ra, normal), ra));
	double invInert2 = dotVec2(normal, crossVec2(poly2->m_invInertia * crossVec2(rb, normal), rb));

	double invInert = invInert1 + invInert2;
	
	double invMassSum = invMass + invInert;

	double e = min(poly1->m_restitution, poly1->m_restitution);

	double j = -(1 + e) * contactVelocityMag;
	j /= invMassSum;

	vec2 impulse = normal * j;

	poly1->move(-normal * m.penetration / 2);
	poly2->move(normal * m.penetration / 2);

	poly1->m_velocity += -impulse * poly1->m_invMass;
	poly1->m_angularVelocity += -crossVec2(ra, impulse) * poly1->m_invInertia;
	poly2->m_velocity += impulse * poly2->m_invMass;
	poly2->m_angularVelocity += crossVec2(rb, impulse) * poly2->m_invInertia;
}

//void lge::ResolveCollision(Manifold m, Polygon* poly1, Polygon* poly2)
//{
//	if (poly1->m_invMass + poly2->m_invMass == 0) return;
//
//
//	std::vector<vec2> contactPoints = getContactPoints(poly1, poly2);
//
//	//if (contactPoints.empty()) ResolveCollisionWithoutRotation(m, poly1, poly2);
//
//	vec2 normal = m.normal[0];
//
//	std::cout << "Normal: " << normal << "\n";
//
//	for (auto i = 0; i < contactPoints.size(); i++)
//	{
//
//		std::cout << "Index: " << i << "\n";
//
//		vec2 ra = (contactPoints[i] - poly1->m_position);
//		vec2 rb = (contactPoints[i] - poly2->m_position);
//
//		vec2 velB = poly2->m_velocity + crossVec2Scalar(poly2->m_angularVelocity, rb); //rb * poly2->m_angularVelocity;
//		vec2 velA = poly1->m_velocity - crossVec2Scalar(poly1->m_angularVelocity, ra); //ra * poly1->m_angularVelocity;
//
//		std::cout << "velB: " << velB << "		velA: " << velA << "\n";
//
//		vec2 rv = velB - velA;
//
//		double contactVel = dotVec2(rv, normal);
//
//		if (contactVel > 0) return;
//
//		//double raCrossN = crossVec2(ra, normal);
//		//double rbCrossN = crossVec2(rb, normal);
//
//		double invMass = poly1->m_invMass + poly2->m_invMass;
//		//double invInert = dotVec2(normal, ((ra * normal) / poly1->m_inertia) * ra) + dotVec2(normal, ((rb * normal) / poly2->m_inertia) * rb); // kinda
//		//double invInert = crossVec2(normal * (crossVec2(ra,normal) / poly1->m_inertia), ra) + crossVec2(normal * (crossVec2(rb, normal) / poly2->m_inertia), rb);  //nope
//		//double invInert = crossVec2(dotVec2(normal,(crossVec2(ra,normal) * (1 / poly1->m_inertia))), ra) + crossVec2(dotVec2(normal, crossVec2(rb,normal) * (1 / poly2->m_inertia)), rb);  //works kinda
//		//double invInert = (raCrossN * raCrossN * (!poly1->m_isStatic / poly1->m_inertia)) + (rbCrossN * rbCrossN * (!poly2->m_isStatic / poly2->m_inertia));  //funky
//		//double invInert = dotVec2(normal, crossVec2((crossVec2(ra, normal) / poly1->m_inertia), ra)) + dotVec2(normal, crossVec2((crossVec2(rb, normal) / poly1->m_inertia), rb));
//
//		double invInert = crossVec2((normal * (crossVec2(ra, normal) *poly1->m_invInertia)), ra) + crossVec2((normal * (crossVec2(rb, normal) *poly2->m_invInertia)), rb);
//
//		//double invInert = (!poly1->m_isStatic * raCrossN * raCrossN * poly2->m_invInertia) + (!poly2->m_isStatic * rbCrossN * rbCrossN * poly2->m_invInertia);
//
//		double invMassSum = invMass + invInert;
//
//		double e = min(poly1->m_restitution, poly2->m_restitution);
//
//		//std::cout << "rv: " << rv << "		Contact Vel: " << contactVel << "		invMassSum: " << invMassSum << "\n";
//
//		double j = -(1 + e) * contactVel;
//		j /= invMassSum;
//		j /= (double) contactPoints.size();
//
//		vec2 impulse = normal * j;
//
//		//std::cout << "impulse: " << impulse << "	J: " << j << "\n\n";
//
//		ApplyImpulseImproved(poly1, -impulse, ra);
//		ApplyImpulseImproved(poly2, impulse, rb);
//
//	}
//}

lge::CollisionData lge::ResolveCollisionCollisionData(Manifold m, Polygon* poly1, Polygon* poly2)
{

	CollisionData info;

	if (poly1->m_invMass + poly2->m_invMass == 0) return info;

	std::vector<vec2> contactPoints = getContactPoints(poly1, poly2);

	vec2 normal = m.normal;

	vec2 contact = contactPoints[0];

	if (contactPoints.size() > 1)
	{
		contact = (contactPoints[0] + contactPoints[1]) / 2;
	}

	info.separating[0] = false;

	info.contactCount = 1;
	info.contactPoints[0] = contact;

	vec2 ra = contact - poly1->m_position;
	vec2 rb = contact - poly2->m_position;

	info.ra[0] = ra;
	info.rb[0] = rb;

	vec2 raPerp = vec2(-ra.y, ra.x);
	vec2 rbPerp = vec2(-rb.y, rb.x);

	vec2 angLineraVelA = raPerp * poly1->m_angularVelocity;
	vec2 angLinearVelB = rbPerp * poly2->m_angularVelocity;

	vec2 relativeVelocity = (poly2->m_velocity + angLinearVelB) - (poly1->m_velocity + angLineraVelA);

	info.relativeVel[0] = relativeVelocity;

	double contactVelocityMag = dotVec2(relativeVelocity, normal);

	info.contactVel[0] = contactVelocityMag;

	if (contactVelocityMag > 0)
	{
		info.separating[0] = true;
		return info;
	}

	double invMass = poly1->m_invMass + poly2->m_invMass;

	info.invMass = invMass;

	double invInert1 = dotVec2(normal, crossVec2(poly1->m_invInertia * crossVec2(ra, normal), ra));
	double invInert2 = dotVec2(normal, crossVec2(poly2->m_invInertia * crossVec2(rb, normal), rb));

	double invInert = invInert1 + invInert2;

	info.invInert[0] = invInert;

	double invMassSum = invMass + invInert;

	double e = min(poly1->m_restitution, poly1->m_restitution);

	double j = -(1 + e) * contactVelocityMag;
	info.jBefore[0] = j;
	j /= invMassSum;
	info.jAfter[0] = j;

	vec2 impulse = normal * j;
	info.impulse[0] = impulse;

	poly1->move(-normal * m.penetration / 2);
	poly2->move(normal * m.penetration / 2);

	poly1->m_velocity += -impulse * poly1->m_invMass;
	poly1->m_angularVelocity += -crossVec2(ra, impulse) * poly1->m_invInertia;
	poly2->m_velocity += impulse * poly2->m_invMass;
	poly2->m_angularVelocity += crossVec2(rb, impulse) * poly2->m_invInertia;

	return info;
}

void lge::ResolveCollisionImproved(Manifold m, Polygon* poly1, Polygon* poly2)
{
	if (poly1->m_invMass + poly2->m_invMass == 0) return;

	std::vector<vec2> contactPoints = getContactPoints(poly1, poly2);
	vec2 normal = m.normal;
	
	double e = min(poly1->m_restitution, poly2->m_restitution);

	int contactCount = contactPoints.size();

	vec2 impulseList[2];

	for (auto i = 0; i < contactCount; i++)
	{
		vec2 ra = contactPoints[i] - poly1->m_position;
		vec2 rb = contactPoints[i] - poly2->m_position;
		vec2 raPerp = vec2(-ra.y, ra.x);
		vec2 rbPerp = vec2(-rb.y, rb.x);

		vec2 angLineraVelA = raPerp * poly1->m_angularVelocity;
		vec2 angLinearVelB = rbPerp * poly2->m_angularVelocity;

		vec2 relativeVelocity = (poly2->m_velocity + angLinearVelB) - (poly1->m_velocity + angLineraVelA);

		double contactVelocityMag = dotVec2(relativeVelocity, normal);

		if (contactVelocityMag > 0)
		{
			continue;
		}

		double raPerpDotN = dotVec2(raPerp, normal);
		double rbPerpDotN = dotVec2(rbPerp, normal);

		double denom = poly1->m_invMass + poly2->m_invMass + 
			(raPerpDotN * raPerpDotN) * poly1->m_invInertia + 
			(rbPerpDotN * rbPerpDotN) * poly2->m_invInertia;

		double j = -(1.0 + e) * contactVelocityMag;
		j /= denom;
		j /= (double)contactCount;
		

		vec2 impulse = normal * j;
		impulseList[i] = impulse;
	}

	for (int i = 0; i < contactCount; i++)
	{
		vec2 impulse = impulseList[i];
		vec2 ra = contactPoints[i] - poly1->m_position;
		vec2 rb = contactPoints[i] - poly2->m_position;

		poly1->move(-normal * m.penetration / 2);
		poly2->move(normal * m.penetration / 2);

		poly1->m_velocity += -impulse * poly1->m_invMass;
		poly1->m_angularVelocity += -crossVec2(ra, impulse) * poly1->m_invInertia;
		poly2->m_velocity += impulse * poly2->m_invMass;
		poly2->m_angularVelocity += crossVec2(rb, impulse) * poly2->m_invInertia;
	}

}

lge::CollisionData lge::ResolveCollisionImprovedCollisionData(Manifold m, Polygon* poly1, Polygon* poly2)
{
	CollisionData info;

	if (poly1->m_invMass + poly2->m_invMass == 0) return info;

	std::vector<vec2> contactPoints = getContactPoints(poly1, poly2);
	vec2 normal = m.normal;

	double e = min(poly1->m_restitution, poly2->m_restitution);

	int contactCount = contactPoints.size();

	info.contactCount = contactCount;

	vec2 impulseList[2];

	for (auto i = 0; i < contactCount; i++)
	{
		info.separating[i] = false;
		vec2 ra = contactPoints[i] - poly1->m_position;
		vec2 rb = contactPoints[i] - poly2->m_position;
		info.contactPoints[i] = contactPoints[i];
		info.ra[i] = ra;
		info.rb[i] = rb;
		
		vec2 raPerp = vec2(-ra.y, ra.x);
		vec2 rbPerp = vec2(-rb.y, rb.x);

		vec2 angLineraVelA = raPerp * poly1->m_angularVelocity;
		vec2 angLinearVelB = rbPerp * poly2->m_angularVelocity;

		vec2 relativeVelocity = (poly2->m_velocity + angLinearVelB) - (poly1->m_velocity + angLineraVelA);
		info.relativeVel[i] = relativeVelocity;

		double contactVelocityMag = dotVec2(relativeVelocity, normal);
		info.contactVel[i] = contactVelocityMag;

		if (contactVelocityMag > 0)
		{
			info.separating[i] = true;
			continue;
		}

		double raPerpDotN = dotVec2(raPerp, normal);
		double rbPerpDotN = dotVec2(rbPerp, normal);

		double invMass = poly1->m_invMass + poly2->m_invMass;
		info.invMass = invMass;
		double invInert = (raPerpDotN * raPerpDotN) * poly1->m_invInertia +
			(rbPerpDotN * rbPerpDotN) * poly2->m_invInertia;
		info.invInert[i] = invInert;
		double denom = invMass + invInert;
			

		double j = -(1.0 + e) * contactVelocityMag;
		info.jBefore[i] = j;
		j /= denom;
		j /= (double)contactCount;
		info.jAfter[i] = j;

		vec2 impulse = normal * j;
		info.impulse[i] = impulse;
		impulseList[i] = impulse;
	}

	for (int i = 0; i < contactCount; i++)
	{
		vec2 impulse = impulseList[i];
		vec2 ra = contactPoints[i] - poly1->m_position;
		vec2 rb = contactPoints[i] - poly2->m_position;

		poly1->m_velocity += -impulse * poly1->m_invMass;
		poly1->m_angularVelocity += -crossVec2(ra, impulse) * poly1->m_invInertia;
		poly2->m_velocity += impulse * poly2->m_invMass;
		poly2->m_angularVelocity += crossVec2(rb, impulse) * poly2->m_invInertia;
	}

	poly1->move(-normal * m.penetration / 2);
	poly2->move(normal * m.penetration / 2);

	return info;
}

//TODO: Rework this
void lge::PositionalCorrection(Manifold m, Polygon* poly1, Polygon* poly2)
{
	const double percent = 0.2;
	vec2 normal = m.normal;

	vec2 correction = normal * percent * (max(-m.penetration, 0) / (poly1->m_invMass + poly2->m_invMass));
	//poly1->m_position -= correction * 1 / poly1->m_mass;
	//poly2->m_position += correction * 1 / poly2->m_mass;
	poly1->move(-correction * poly1->m_invMass);
	poly2->move(correction * poly2->m_invMass);

}

void lge::ApplyImpulse(Polygon* poly, vec2 impulse, std::vector<vec2> contactPoints)
{
	poly->m_velocity += (impulse * (1 / poly->m_mass));// + (impulse * (1 / poly2->m_mass) * poly2->m_isStatic);

	vec2 normal = impulse.normalize();
	for (unsigned int j = 0; j < contactPoints.size(); j++)
	{
		vec2 contactPoint = contactPoints[j];
		poly->m_angularVelocity += 1.0 / poly->m_inertia * crossVec2(contactPoint, normal) / contactPoints.size();
		//poly2->m_angularVelocity += 1.0 / poly2->m_inertia * crossVec2(contactPoint, normal) / contactPoints.size() * !poly2->m_isStatic;
	}
}

void lge::ApplyImpulseImproved(Polygon* poly, vec2 impulse, vec2 contactPoint)
{
	poly->m_velocity += impulse * poly->m_invMass;
	poly->m_angularVelocity += crossVec2(contactPoint, impulse) * poly->m_invInertia;
}
