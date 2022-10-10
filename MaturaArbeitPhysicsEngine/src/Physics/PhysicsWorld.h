#pragma once

#include "Shapes/Polygon.h"
#include "CollisionDetection.h"
#include "CollisionResolution.h"
#include "../Renderer/Renderer.h"

namespace lge
{

	class PhysicsWorld
	{
	private:

		Polygon* world;

		std::vector<Polygon*> bodies;
		vec2 size;

		vec2 GRAVITY;

		std::vector<vec2> boxCorners;

		unsigned int stepCount = 5;

	public:

		PhysicsWorld(vec2 size);
		~PhysicsWorld();

		Polygon* addPolygon(vec2 position, mat2 scale, double angle, double sides, bool setStatic, double density, double restitiution, vec4 color);
		Polygon* addBox(vec2 position, vec2 dimension, double angle, bool setStatic, double density, double restitution, vec4 color);

		void removeBody(int index);
		void update(double deltaTime);
		void renderWorld(Renderer* renderer);

	};

}
