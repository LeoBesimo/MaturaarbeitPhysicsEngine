#include "PhysicsWorld.h"

lge::PhysicsWorld::PhysicsWorld(vec2 size)
{
	boxCorners.push_back(vec2(-1, -1));
	boxCorners.push_back(vec2(1, -1));
	boxCorners.push_back(vec2(1, 1));
	boxCorners.push_back(vec2(-1, 1));

	GRAVITY = vec2(0, 100);

	this->size = size;
	this->world = new Polygon(size / 2, 0, mat2(size.x, 0, 0, size.y), boxCorners);
}

lge::PhysicsWorld::~PhysicsWorld()
{
	for (int i = bodies.size() - 1; i >= 0; i--)
	{
		int index = bodies.size() - 1 - i;
		delete bodies[i];
		bodies[i] = nullptr;
		//bodies.erase(bodies.begin() + index);
	}
	bodies.clear();
}

lge::Polygon* lge::PhysicsWorld::addPolygon(vec2 position, mat2 scale, double angle, double sides, bool setStatic, double density, double restitiution, vec4 color)
{
	std::vector<vec2> corners;
	double increment = TWO_PI / sides;
	for (double i = 0; i < TWO_PI; i += increment)
	{
		corners.push_back(vec2(cos(i), sin(i)));
	}

	Polygon* body = new Polygon(position, angle, scale, corners);

	body->m_restitution = restitiution;
	body->m_color = color;

	if (setStatic)
	{
		body->setMass(0);
		body->setInertia(0);
	}
	else
	{
		body->calculateMass(density);
		body->calculateInertia();
	}

	bodies.push_back(body);

	return body;
}

lge::Polygon* lge::PhysicsWorld::addBox(vec2 position, vec2 dimension, double angle, bool setStatic, double density, double restitution, vec4 color)
{
	Polygon* body = new Polygon(position, angle, mat2(dimension.x / 2.0, 0, 0, dimension.y / 2.0), boxCorners);
	body->m_restitution = 1.0;
	body->m_color = color;

	if (setStatic)
	{
		body->setMass(0);
		body->setInertia(0);
	}
	else
	{
		body->calculateMass(density);
		body->calculateInertia();
		//body->setInertia(body->m_mass / 12 * (pow(dimension.x, 2) + pow(dimension.y, 2)));
	}

	bodies.push_back(body);

	return body;
}

void lge::PhysicsWorld::removeBody(int index)
{
	delete bodies[index];
	bodies[index] = nullptr;
	bodies.erase(bodies.begin() + index);
}

void lge::PhysicsWorld::update(double deltaTime)
{
	for (int i = bodies.size() - 1; i >= 0; i--)
	{
		if (!AABBCollision(bodies[i], world)) removeBody(i);
	}

	for (unsigned int i = 0; i < stepCount; i++)

	{
		
		for (Polygon* body : bodies)
		{
			body->m_force += GRAVITY * body->m_mass / stepCount;
			body->integrateForces(deltaTime);
		}

		for (Polygon* bodyA : bodies)
		{
			bodyA->update(deltaTime, stepCount);

			for (Polygon* bodyB : bodies)
			{
				if (bodyA == bodyB) continue;

				if (AABBCollision(bodyA, bodyB))
				{
					Manifold manifold = PolygonCollisionSatManifold(bodyA, bodyB);

					ResolveCollisionImproved(manifold, bodyA, bodyB);
				}
			}
		}
	}
}

void lge::PhysicsWorld::renderWorld(Renderer* renderer)
{
	for (Polygon* body : bodies)
	{
		renderer->stroke(body->m_color);
		renderer->renderVec2List(body->m_transformedPoints);
	}
}
