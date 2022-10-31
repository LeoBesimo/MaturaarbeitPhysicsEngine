#include "PhysicsWorld.h"

lge::PhysicsWorld::PhysicsWorld(Renderer* renderer, vec2 size)
{
	boxCorners.push_back(vec2(-1, -1));
	boxCorners.push_back(vec2(1, -1));
	boxCorners.push_back(vec2(1, 1));
	boxCorners.push_back(vec2(-1, 1));

	GRAVITY = vec2(0, 100);

	this->bodies.reserve(30);

	this->size = size;
	this->renderer = renderer;
	this->world = new Polygon(size / 2, 0, mat2(size.x/2, 0, 0, size.y/2), boxCorners);
}

lge::PhysicsWorld::~PhysicsWorld()
{
	for (int i = bodies.size() - 1; i >= 0; i--)
	{
		int index = bodies.size() - 1 - i;
		delete bodies[i];
		bodies[i] = nullptr;
	}
	bodies.clear();
}

lge::Polygon* lge::PhysicsWorld::addPolygon(vec2 position, mat2 scale, double angle, double sides, bool setStatic, double density, double restitiution, vec4 color)
{
	std::vector<vec2> corners;
	if (sides == 6) sides += 1;
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
	body->m_restitution =restitution;
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

void lge::PhysicsWorld::setResolutionIndex(int index)
{
	resolutionIndex = index;
}

int lge::PhysicsWorld::getResolutionIndex()
{
	return this->resolutionIndex;
}

void lge::PhysicsWorld::testSetup()
{
	addBox(vec2(700, 200), vec2(50, 30), 0, false, 1, 0.4, lge::Color::RED);
	addBox(vec2(300, 300), vec2(30, 60), 0, false, 1, 0.4, lge::Color::RED);
	addBox(vec2(500, 100), vec2(40, 40), 0, false, 1, 0.4, lge::Color::RED);

	addPolygon(vec2(200, 100), mat2(30, 30, 0, 30), 0, 5, false, 1, 0.4, lge::Color::PINK);
	addPolygon(vec2(800, 700), mat2(40, 0, 0, 40), 0, 7, false, 1, 0.4, lge::Color::PINK);
	Polygon* p = addPolygon(vec2(600, 250), mat2(30, 0, 0, 20), 0, 4, false, 1, 0.4, lge::Color::PINK);
}

void lge::PhysicsWorld::reset()
{
	for (int i = bodies.size() - 1; i >= 0; i--)
	{
		if (!bodies[i]->m_mass == 0) removeBody(i);
	}

	for (Polygon* body : bodies)
	{
		body->m_velocity = vec2();
		body->m_angularVelocity = 0;
	}
}

bool lge::PhysicsWorld::getData(const char* file)
{
	serializer.serializeObjects(file);
	return true;
}

void lge::PhysicsWorld::update(double deltaTime)
{
	for (int i = bodies.size() - 1; i >= 0; i--)
	{
		if (!AABBCollision(bodies[i], world)) removeBody(i);
	}

	//std::cout << bodies.size() << "\n";

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
					ObjectSerializer::SerializableObject data;
					Manifold manifold = PolygonCollisionSatManifold(bodyA, bodyB);
					switch (resolutionIndex)
					{
					case 1:
						ResolveCollision(manifold, bodyA, bodyB);
						break;
					case 2: 
						ResolveCollisionImproved(manifold, bodyA, bodyB);
						break;
					case 3:
						ResolveCollisionWithoutRotation(manifold, bodyA, bodyB);
						break;
					case 4:			
						data.collisionAlgorithm = resolutionIndex;
						data.manifold = manifold;
						data.a = *bodyA;
						data.b = *bodyB;
						data.frame = renderer->getFrameCount();
						data.collision = ResolveCollisionCollisionData(manifold, bodyA, bodyB);
						serializer.addObject(data);
						break;
					case 5:
						data.collisionAlgorithm = resolutionIndex;
						data.manifold = manifold;
						data.a = *bodyA;
						data.b = *bodyB;
						data.frame = renderer->getFrameCount();
						data.collision = ResolveCollisionImprovedCollisionData(manifold, bodyA, bodyB);
						serializer.addObject(data);
						break;
					case 6:
						ResolveCollisionCombined(manifold, bodyA, bodyB);
						break;
					case 7:
						data.collisionAlgorithm = resolutionIndex;
						data.manifold = manifold;
						data.a = *bodyA;
						data.b = *bodyB;
						data.frame = renderer->getFrameCount();
						data.collision = ResolveCollisionCombinedCollisonData(manifold, bodyA, bodyB);
						serializer.addObject(data);
						break;
					case 8:
						ResolveCollision3D(manifold, bodyA, bodyB);
					default:
						ResolveCollisionWithoutRotation(manifold, bodyA, bodyB);
						break;
					}
				}
			}
		}
	}
}

void lge::PhysicsWorld::renderWorld()
{

	std::stringstream stream;
	stream << "Bodies: " << bodies.size();

	renderer->stroke(lge::Color::CYAN);
	renderer->text(stream.str(), 10, 30, 16);

	for (Polygon* body : bodies)
	{
		renderer->stroke(body->m_color);
		renderer->renderVec2ListSolid(body->m_transformedPoints);
	}
}

std::vector<lge::Polygon*> lge::PhysicsWorld::getBodies()
{
	return bodies;
}
