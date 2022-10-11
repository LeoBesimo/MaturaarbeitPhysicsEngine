#pragma once
#include "components.h"
#include "Utilities.h"

#include "../Physics/Shapes/Polygon.h"
#include "../Physics/CollisionResolution.h"
#include "../Physics/Manifold.h"

#include <iostream>
#include <fstream>

namespace lge
{
	class ObjectSerializer
	{
	public:

		struct SerializableObject
		{
			int collisionAlgorithm = 1;
			long frame;
			Manifold manifold;
			Polygon a;
			Polygon b;
			CollisionData collision;
		} serializable;

	private:
		std::vector<SerializableObject> objects;


	public:

		ObjectSerializer() {};
		~ObjectSerializer() {};

		void addObject(SerializableObject object);

		void serializeObjects(const char* fileName);
	};
}