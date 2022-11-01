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

lge::CollisionData<lge::vec2> lge::ResolveCollisionCollisionData(Manifold m, Polygon* poly1, Polygon* poly2)
{

	CollisionData<lge::vec2> info;

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

lge::CollisionData<lge::vec2> lge::ResolveCollisionImprovedCollisionData(Manifold m, Polygon* poly1, Polygon* poly2)
{
	CollisionData<vec2> info;

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

void lge::ResolveCollisionCombined(Manifold m, Polygon* poly1, Polygon* poly2)
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

	double raPerpDotN = dotVec2(raPerp, normal);
	double rbPerpDotN = dotVec2(rbPerp, normal);

	double denom = poly1->m_invMass + poly2->m_invMass +
		(raPerpDotN * raPerpDotN) * poly1->m_invInertia +
		(rbPerpDotN * rbPerpDotN) * poly2->m_invInertia;

	double e = min(poly1->m_restitution, poly2->m_restitution);

	double j = -(1.0 + e) * contactVelocityMag;
	j /= denom;

	vec2 impulse = normal * j;

	poly1->move(-normal * m.penetration / 2);
	poly2->move(normal * m.penetration / 2);

	poly1->m_velocity += -impulse * poly1->m_invMass;
	poly1->m_angularVelocity += -crossVec2(ra, impulse) * poly1->m_invInertia;
	poly2->m_velocity += impulse * poly2->m_invMass;
	poly2->m_angularVelocity += crossVec2(rb, impulse) * poly2->m_invInertia;
}

lge::CollisionData<lge::vec2> lge::ResolveCollisionCombinedCollisonData(Manifold m, Polygon* poly1, Polygon* poly2)
{

	CollisionData<vec2> data;

	if (poly1->m_invMass + poly2->m_invMass == 0) return data;

	data.separating[0] = false;

	std::vector<vec2> contactPoints = getContactPoints(poly1, poly2);

	vec2 normal = m.normal;

	vec2 contact = contactPoints[0];
	data.contactCount = 1;

	if (contactPoints.size() > 1)
	{
		contact = (contactPoints[0] + contactPoints[1]) / 2;
	}

	data.contactPoints[0] = contact;

	vec2 ra = contact - poly1->m_position;
	vec2 rb = contact - poly2->m_position;

	data.ra[0] = ra;
	data.rb[0] = rb;

	vec2 raPerp = vec2(-ra.y, ra.x);
	vec2 rbPerp = vec2(-rb.y, rb.x);

	vec2 angLineraVelA = raPerp * poly1->m_angularVelocity;
	vec2 angLinearVelB = rbPerp * poly2->m_angularVelocity;

	data.velA[0] = poly1->m_velocity + angLineraVelA;
	data.velB[0] = poly2->m_velocity + angLinearVelB;

	vec2 relativeVelocity = (poly2->m_velocity + angLinearVelB) - (poly1->m_velocity + angLineraVelA);

	data.relativeVel[0] = relativeVelocity;

	double contactVelocityMag = dotVec2(relativeVelocity, normal);

	data.contactVel[0] = contactVelocityMag;

	if (contactVelocityMag > 0)
	{
		data.separating[0] = true;
		return data;
	}

	double raPerpDotN = dotVec2(raPerp, normal);
	double rbPerpDotN = dotVec2(rbPerp, normal);

	double invMass = poly1->m_invMass + poly2->m_invMass;
	data.invMass = invMass;
	double invInert = (raPerpDotN * raPerpDotN) * poly1->m_invInertia +
		(rbPerpDotN * rbPerpDotN) * poly2->m_invInertia;
	data.invInert[0] = invInert;
	double denom = invMass + invInert;

	double e = min(poly1->m_restitution, poly2->m_restitution);

	double j = -(1.0 + e) * contactVelocityMag;
	data.jBefore[0] = j;

	j /= denom;

	data.jAfter[0] = j;

	vec2 impulse = normal * j;

	data.impulse[0] = impulse;

	data.eKin[0] = 0.5 * poly1->m_mass * poly1->m_velocity.lenSqr();
	data.eKin[1] = 0.5 * poly2->m_mass * poly2->m_velocity.lenSqr();

	data.eRot[0] = 0.5 * poly1->m_inertia * pow(poly1->m_angularVelocity,2);
	data.eRot[1] = 0.5 * poly2->m_inertia * pow(poly2->m_angularVelocity,2);

	poly1->move(-normal * m.penetration / 2);
	poly2->move(normal * m.penetration / 2);

	poly1->m_velocity += -impulse * poly1->m_invMass;
	poly1->m_angularVelocity += -crossVec2(ra, impulse) * poly1->m_invInertia;
	poly2->m_velocity += impulse * poly2->m_invMass;
	poly2->m_angularVelocity += crossVec2(rb, impulse) * poly2->m_invInertia;

	data.eKinAfter[0] = 0.5 * poly1->m_mass * poly1->m_velocity.lenSqr();
	data.eKinAfter[1] = 0.5 * poly2->m_mass * poly2->m_velocity.lenSqr();

	data.eRotAfter[0] = 0.5 * poly1->m_inertia * pow(poly1->m_angularVelocity,2);
	data.eRotAfter[1] = 0.5 * poly2->m_inertia * pow(poly2->m_angularVelocity,2);

	return data;
}

void lge::ResolveCollision3D(Manifold m, Polygon* poly1, Polygon* poly2)
{
	if (poly1->m_invMass + poly2->m_invMass == 0) return;

	std::vector<vec2> contactPoints = getContactPoints(poly1, poly2);
	vec3 normal = vec3(m.normal.x,m.normal.y,0);

	double e = min(poly1->m_restitution, poly2->m_restitution);

	vec3 contact = vec3(contactPoints[0].x,contactPoints[0].y,0);

	if (contactPoints.size() > 1)
	{
		vec2 avg = (contactPoints[0] + contactPoints[1]) / 2;
		contact = vec3(avg.x, avg.y, 0);
	}

	vec3 ra = contact - vec3(poly1->m_position.x,poly1->m_position.y,0);
	vec3 rb = contact - vec3(poly2->m_position.x,poly2->m_position.y,0);

	vec2 raPerp = vec2(-ra.y, ra.x);
	vec2 rbPerp = vec2(-rb.y, rb.x);

	vec3 angLineraVelA = dotVec3(ra, vec3(0,0,poly1->m_angularVelocity));
	vec3 angLinearVelB = dotVec3(rb, vec3(0,0,poly2->m_angularVelocity));

	vec3 relativeVelocity = (vec3(poly2->m_velocity.x,poly2->m_velocity.y,0) + angLinearVelB) - (vec3(poly1->m_velocity.x,poly1->m_velocity.y,0) + angLineraVelA);

	double contactVelocityMag = dotVec3(relativeVelocity, normal);

	if (contactVelocityMag > 0)
	{
		return;
	}

	double invMass = poly1->m_invMass + poly2->m_invMass;

	double invInert1 = dotVec3(normal, crossVec3(crossVec3(ra, normal) * poly1->m_invInertia, ra));
	double invInert2 = dotVec3(normal, crossVec3(crossVec3(rb, normal) * poly2->m_invInertia, rb));

	double invInert = invInert1 + invInert2;

	double invMassSum = invMass + invInert;

	double j = -(1 + e) * contactVelocityMag;
	j /= invMassSum;

	vec3 impulse = normal * j;

	poly1->move(-m.normal * m.penetration / 2);
	poly2->move(m.normal * m.penetration / 2);

	poly1->m_velocity += -vec2(impulse.x,impulse.y) * poly1->m_invMass;
	poly1->m_angularVelocity += -crossVec3(ra, impulse).z * poly1->m_invInertia;
	poly2->m_velocity += vec2(impulse.x,impulse.y) * poly2->m_invMass;
	poly2->m_angularVelocity += crossVec3(rb, impulse).z * poly2->m_invInertia;
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
