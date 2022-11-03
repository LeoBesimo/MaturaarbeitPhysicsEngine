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
		file << "Collision Algorithm: " << o.collisionAlgorithm << "\n";

		file << "Polygon A:\n";
		file << "Position: " << o.a.m_position << "\n";
		file << "Rotation: " << o.a.m_angle << "\n";
		file << "Velocity: " << o.a.m_velocity << "\n";
		file << "Angular Velocity: " << o.a.m_angularVelocity << "\n";
		file << "Mass: " << o.a.m_mass << "\n";
		file << "Inertia: " << o.a.m_inertia << "\n";
		file << "Force: " << o.a.m_force << "\n";
		file << "Torque: " << o.a.m_torque << "\n";
		file << "Restitution: " << o.a.m_restitution << "\n";

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
		file << "Force: " << o.b.m_force << "\n";
		file << "Torque: " << o.b.m_torque << "\n";
		file << "Restitution: " << o.b.m_restitution << "\n";

		file << "Edge Points:\n";
		for (unsigned int i = 0; i < o.b.m_transformedPoints.size(); i++)
		{
			file << "	Index: " << i << "	" << o.b.m_transformedPoints[i] << "\n";
		}

		file << "\n" << "Manifold:\n\n";

		file << "Collision: " << o.manifold.collided << "\n";
		file << "Normal: << " << o.manifold.normal << "\n\n";
		for (unsigned int i = 0; i < o.collision.contactCount; i++)
		{
			file << "Contact Index: " << i << "\n";
			file << "Contact Point: " << o.collision.contactPoints[i] << "\n";
			file << "Contact Velocity: " << o.collision.contactVel[i] << "\n";
			file << "Relative Velocity: " << o.collision.rv[i] << "\n";
			file << "Radius A: " << o.collision.ra[i] << " Mag: " << o.collision.ra[i].len() << "\n";
			file << "Radius B: " << o.collision.rb[i] << " Mag: " << o.collision.rb[i].len() << "\n";
			file << "Velocity A: " << o.collision.velA[i] << "\n";
			file << "Velocity B: " << o.collision.velB[i] << "\n";


			if (!o.collision.separating[i])
			{
				file << "Inverse Inertia: " << o.collision.invInert[i] << "\n";
				file << "Inverse Mass: " << o.collision.invMass << "\n";
				file << "Impulse Scalar Before Division: " << o.collision.jBefore[i] << "\n";
				file << "Impulse Scalar After Division: " << o.collision.jAfter[i] << "\n";
				file << "Impulse: " << o.collision.impulse[i] << "\n\n";

				double totalBefore = 0;
				double totalAfter = 0;

				for (unsigned int i = 0; i < 2; i++)
				{	
					file << "Poly" << i << ":\n";
					file << "E kin before: " << o.collision.eKin[i] << "\n";
					file << "E rot before: " << o.collision.eRot[i] << "\n";
					file << "E tot before: " << o.collision.eKin[i] + o.collision.eRot[i] << "\n";

					totalBefore += o.collision.eKin[i] + o.collision.eRot[i];

					file << "E kin After " << o.collision.eKinAfter[i] << "\n";
					file << "E rot After " << o.collision.eRotAfter[i] << "\n";
					file << "E tot After " << o.collision.eKinAfter[i] + o.collision.eRotAfter[i] << "\n";

					totalAfter += o.collision.eKinAfter[i] + o.collision.eRotAfter[i];
				}

				file << "Total energy Before: " << totalBefore << "\n";
				file << "Total energy After: " << totalAfter << "\n\n";

				vec3 linImpulseABefore = vec3(o.a.m_velocity.x, o.a.m_velocity.y, 0) * o.a.m_mass;
				vec3 linImpulseBBefore = vec3(o.b.m_velocity.x, o.b.m_velocity.y, 0) * o.b.m_mass;
				vec3 angImpulseABefore = vec3(0, 0, o.a.m_angularVelocity) * o.a.m_inertia;
				vec3 angImpulseBBefore = vec3(0, 0, o.b.m_angularVelocity) * o.b.m_inertia;

				vec3 totalImpulseABefore = linImpulseABefore + angImpulseABefore;
				vec3 totalImpulseBBefore = linImpulseBBefore + angImpulseBBefore;

				vec3 totalImpulseBefore = totalImpulseABefore + totalImpulseBBefore;

				vec3 linImpulseAAfter = vec3(o.collision.velAfterA.x, o.collision.velAfterA.y, 0) * o.a.m_mass;
				vec3 linImpulseBAfter = vec3(o.collision.velAfterB.x, o.collision.velAfterB.y, 0) * o.b.m_mass;
				vec3 angImpulseAAfter = vec3(0, 0, o.collision.angVelAfterA) * o.a.m_inertia;
				vec3 angImpulseBAfter = vec3(0, 0, o.collision.angVelAfterB) * o.b.m_inertia;

				vec3 totalImpulseAAfter = linImpulseAAfter + angImpulseAAfter;
				vec3 totalImpulseBAfter = linImpulseBAfter + angImpulseBAfter;

				vec3 totalImpulseAfter = totalImpulseAAfter + totalImpulseBAfter;

				file << "Linear Imulse A Before: " << linImpulseABefore << "\n";
				file << "Angular Imulse A Before: " << angImpulseABefore << "\n";
				file << "Linear Imulse B Before: " << linImpulseBBefore << "\n";
				file << "Angular Imulse B Before: " << angImpulseBBefore << "\n";

				file << "Total Impulse A Before: " << totalImpulseABefore << "\n";
				file << "Total Impulse B Before: " << totalImpulseBBefore << "\n";
				file << "Total Impulse Before: " << totalImpulseBefore << " Impulse Mag: " << totalImpulseBefore.len() << "\n\n";

				file << "Linear Imulse A After: " << linImpulseAAfter << "\n";
				file << "Angular Imulse A After: " << angImpulseAAfter << "\n";
				file << "Linear Imulse B After: " << linImpulseBAfter << "\n";
				file << "Angular Imulse B After: " << angImpulseBAfter << "\n";

				file << "Total Impulse A After: " << totalImpulseAAfter << "\n";
				file << "Total Impulse B After: " << totalImpulseBAfter << "\n";
				file << "Total Impulse After: " << totalImpulseAfter << " Impulse Mag: " << totalImpulseAfter.len() << "\n";
			}
			else
			{
				//file << "Contact Velocity: " << o.collision.contactVel[i] << "\n";
				//file << "Relative Velocity: " << o.collision.rv[i] << "\n";
				file << o.collision.separating[i] << "\n";
				file << "Separating at Contactpoint\n\n";
			}
		}

		file << "\n\n\n";
	}
	file.close();
}
