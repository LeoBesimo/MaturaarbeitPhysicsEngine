#include "ObjectSerializer.h"

void lge::ObjectSerializer::addObject(SerializableObject object)
{
	objects.push_back(object);
}

void lge::ObjectSerializer::serializeObjects(const char* fileName)
{
	std::ofstream file;
	file.open(fileName);

	for (SerializableObject o : objects)
	{
		file << "Frame: " << o.frame << "\n";

		file << "Polygon A:\n";
		file << "Position: " << o.a.m_position << "\n";
		file << "Rotation: " << o.a.m_angle << "\n";
		file << "Velocity: " << o.a.m_velocity << "\n";
		file << "Angular Velocity: " << o.a.m_angularVelocity << "\n";
		file << "Mass: " << o.a.m_mass << "\n";
		file << "Inertia: " << o.a.m_inertia << "\n";
		file << "Acceleration: " << o.a.m_acceleration << "\n";
		file << "Torque: " << o.a.m_torque << "\n";
		file << "Restitution: " << o.a.m_restitution << "\n";
		file << "is Static: " << o.a.m_isStatic << "\n";

		file << "Edge Points:\n";
		for (unsigned int i = 0; i < o.a.m_transformedPoints.size(); i++)
		{
			file << "	Index: " << i << "	" << o.a.m_transformedPoints[i] << "\n";
		}

		file << "\n";

		file << "Polygon B:\n";
		file << "Position: " << o.b.m_position << "\n";
		file << "Rotation: " << o.b.m_angle << "\n";
		file << "Velocity: " << o.b.m_velocity << "\n";
		file << "Angular Velocity: " << o.b.m_angularVelocity << "\n";
		file << "Mass: " << o.b.m_mass << "\n";
		file << "Inertia: " << o.b.m_inertia << "\n";
		file << "Acceleration: " << o.b.m_acceleration << "\n";
		file << "Torque: " << o.b.m_torque << "\n";
		file << "Restitution: " << o.b.m_restitution << "\n";
		file << "is Static: " << o.b.m_isStatic << "\n";

		file << "Edge Points:\n";
		for (unsigned int i = 0; i < o.b.m_transformedPoints.size(); i++)
		{
			file << "	Index: " << i << "	" << o.b.m_transformedPoints[i] << "\n";
		}

		file << "\n" << "Manifold:\n\n";

		file << "Collision: " << o.manifold.collided << "\n";
		file << "Normal 1: << " << o.manifold.normal[0] << "\n";
		file << "Normal 2: << " << o.manifold.normal[1] << "\n\n";

		for (unsigned int i = 0; i < o.collision.contactPoints.size(); i++)
		{
			file << "Contact Index: " << i << "\n";
			file << "Contact Point: " << o.collision.contactPoints[i] << "\n";
			if (!o.collision.separating && o.collision.impulse.size() == o.collision.contactPoints.size())
			{
				file << "Contact Velocity: " << o.collision.contactVel[i] << "\n";
				file << "Relative Velocity: " << o.collision.rv[i] << "\n";
				file << "Radius A: " << o.collision.ra[i] << "\n";
				file << "Radius B: " << o.collision.rb[i] << "\n";
				file << "Velocity A: " << o.collision.velA[i] << "\n";
				file << "Velocity B: " << o.collision.velB[i] << "\n";
				file << "Inverse Inertia: " << o.collision.invInert[i] << "\n";
				file << "Inverse Mass: " << o.collision.invMass << "\n";
				file << "Impulse Scalar Before Division: " << o.collision.jBefore[i] << "\n";
				file << "Impulse Scalar After Division: " << o.collision.jAfter[i] << "\n";
				file << "Impulse: " << o.collision.impulse[i] << "\n\n";
			}
			else
			{
				file << "Objects are Separating\n\n";
			}
		}

		file << "\n\n\n\n";

	}
	file.close();
}
