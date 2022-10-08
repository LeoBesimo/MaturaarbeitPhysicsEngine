#include <fstream>
#include <iostream>

#include <chrono>

#include <SFML/Graphics.hpp>

#include "Renderer/Renderer.h"
#include "Renderer/CustomShapes/EllipseShape.h"

#include "components.h"
#include "utilities.h"

#include "Physics/CollisionDetection.h"
#include "Physics/CollisionResolution.h"

#include "FunctionTimer.h"

#include "Debug Tools/ObjectSerializer.h"

/*
	fix normal generation for uneven side count
*/

int main()
{
	lge::ObjectSerializer serializer;
	//srand(time(NULL));
	srand(0);
	Renderer mainRenderer(1000, 1000, "Test");

	mainRenderer.getWindow()->setFramerateLimit(60);

	mainRenderer.circleMode(CENTER);
	mainRenderer.rectMode(CENTER);

	std::vector<lge::vec2> squareEdges;
	std::vector<lge::Polygon> walls;
	std::vector<lge::Polygon> polys;

	for (double i = 0; i < lge::TWO_PI; i += lge::TWO_PI / 4)
	{
		squareEdges.push_back(lge::vec2(cos(i), sin(i)));
	}

	double width = mainRenderer.getWindowSize().x;
	double height = mainRenderer.getWindowSize().y;

	std::vector<lge::vec2> vecs;
	vecs.push_back(lge::vec2(-width/2, -10));
	vecs.push_back(lge::vec2(width/2, -10));
	vecs.push_back(lge::vec2(width/2, 10));
	vecs.push_back(lge::vec2(-width/2, 10));

	walls.push_back(lge::Polygon(lge::vec2(width/2, 10), 0, lge::mat2(1,0,0,1), vecs));

	vecs.clear();

	vecs.push_back(lge::vec2(10, -height/2));
	vecs.push_back(lge::vec2(-10, -height/2));
	vecs.push_back(lge::vec2(-10, height/2));
	vecs.push_back(lge::vec2(10, height/2));

	walls.push_back(lge::Polygon(lge::vec2(width-10, height/2), 0, lge::mat2(1,0,0,1), vecs));

	vecs.clear();

	vecs.push_back(lge::vec2(-width / 2, -10));
	vecs.push_back(lge::vec2(width / 2, -10));
	vecs.push_back(lge::vec2(width / 2, height-10));
	vecs.push_back(lge::vec2(-width / 2, height-10));


	walls.push_back(lge::Polygon(lge::vec2(width/2, height-10), 0, lge::mat2(1,0,0,1), vecs));
	
	vecs.clear();

	vecs.push_back(lge::vec2(-10, -height/2));
	vecs.push_back(lge::vec2(10, -height/2));
	vecs.push_back(lge::vec2(10, height/2));
	vecs.push_back(lge::vec2(-10, height/2));

	walls.push_back(lge::Polygon(lge::vec2(10, height/2), 0, lge::mat2(1,0,0,1), vecs));

	std::vector<lge::vec2> vectors;

	vectors.push_back(lge::vec2(1, -1));
	vectors.push_back(lge::vec2(1, 1));
	vectors.push_back(lge::vec2(-1, 1));
	vectors.push_back(lge::vec2(-1, -1));

	lge::Polygon poly(lge::vec2(200, 400), lge::QUARTER_PI/2, lge::mat2(50, 0, 0, 50), vectors);
	poly.m_velocity = lge::vec2(60, 0);


	vectors.clear();
	for (double i = 0; i < lge::TWO_PI; i += lge::TWO_PI / 4)
	{
		vectors.push_back(lge::vec2(cos(i), sin(i)));
	}

	lge::Polygon poly2(lge::vec2(400, 400), 0, lge::mat2(50, 0, 0, 50), vectors);

	poly.setMass(10);
	poly2.setMass(10);
	poly.m_restitution = 1.0;
	poly2.m_restitution = 1.0;
	poly.setInertia(poly.m_mass / 12 * (pow(poly.m_scale.x.x, 2) + pow(poly.m_scale.y.y, 2)));
	poly2.setInertia(poly2.m_mass / 12 * (pow(poly2.m_scale.x.x, 2) + pow(poly2.m_scale.y.y, 2)));

	polys.push_back(poly);

	polys.push_back(poly2);

	for (lge::Polygon wall : walls)
	{	
		wall.setMass(0);
		wall.setInertia(0);
		wall.m_restitution = 0.75;
		polys.push_back(wall);
	}

	for (auto i = 0; i < 0; i++)
	{
		vectors.clear();
		double sides = lge::randomDouble(3, 9);
		for (double j = 0; j < lge::TWO_PI; j += lge::TWO_PI / sides)
		{
			vectors.push_back(lge::vec2(cos(j), sin(j)));
		}
		double scaleX = lge::randomDouble(10, 50);
		double scaleX2 = lge::randomDouble(10, 50);
		double scaleY = lge::randomDouble(10, 50);
		double scaleY2 = lge::randomDouble(10, 50);
		poly = lge::Polygon(lge::vec2(lge::randomDouble(100, 700), lge::randomDouble(100, 700)), lge::randomDouble(0, lge::TWO_PI), lge::mat2(scaleX, scaleY, scaleX2, scaleY2), vectors);
		polys.push_back(poly);
	}

	bool keyPressed = true;
	bool mousePressed = false;

	sf::Clock clock;
	float lastTime = 0;

	while (mainRenderer.isRunning())
	{
		std::vector<sf::Event> events = mainRenderer.getEvents();

		//keyPressed = false;
		lge::vec2 mouse = mainRenderer.getMousePosition();

		for (auto& event : events)
		{
			if (event.type == sf::Event::Closed)
			{
				mainRenderer.~Renderer();

				return 0;
			}

			if (event.type == sf::Event::KeyPressed)
			{
				if (event.key.code == sf::Keyboard::Escape)
				{
					serializer.serializeObjects("Test.txt");
					mainRenderer.~Renderer();
				}
				keyPressed = !keyPressed;
			}

			if (event.type == sf::Event::MouseButtonPressed)
			{
				if (!mousePressed)
				{
					vectors.clear();
					double sides = lge::randomDouble(3, 9);
					for (double j = 0; j < lge::TWO_PI; j += lge::TWO_PI / sides)
					{
						vectors.push_back(lge::vec2(cos(j), sin(j)));
					}
					double scaleX = lge::randomDouble(10, 50);
					double scaleX2 = lge::randomDouble(20, 50);
					double scaleY = lge::randomDouble(10, 50);
					double scaleY2 = lge::randomDouble(20, 50);
					poly = lge::Polygon(mouse, lge::randomDouble(0, lge::TWO_PI), lge::mat2(scaleX, 0, 0, scaleY), vectors);
					poly.m_velocity = lge::random2D() * lge::randomDouble(20, 60);
					polys.push_back(poly);
					//mousePressed = true;
				}
				else
				{
					mousePressed = false;
				}
			}
		}

		if (keyPressed) mainRenderer.clear(lge::vec4(0, 0, 0, 255));

		if (keyPressed)
		{

			for (auto i = 0; i < polys.size(); i++)
			{
				polys[i].m_force += lge::vec2(0, 98.1);
				polys[i].integrateForces(mainRenderer.getDeltaTime());
			}

			for (auto i = 0; i < polys.size(); i++)
			{
				//polys[i].m_velocity += lge::vec2(0, 0.8);
				polys[i].update(mainRenderer.getDeltaTime());

				lge::Manifold m;

				for (auto j = i + 1; j < polys.size(); j++)
				{
					if (i == j) continue;

					m = lge::PolygonCollisionSatManifold(&polys[i], &polys[j]);


					if (m.collided)//lge::AABBCollision(&polys[i],&polys[j]))
					{
						lge::ObjectSerializer::SerializableObject data;
						data.frame = mainRenderer.getFrameCount();
						//m = lge::PolygonCollisionSatManifold(&polys[i], &polys[j]);

						data.a = polys[i];
						data.b = polys[j];
						data.manifold = m;

						//if(i < 2) std::cout << "i: " << i << " " << polys[i].m_angularVelocity << "\n";


						std::vector<lge::vec2> contacts = lge::getContactPoints(&polys[i], &polys[j]);
						mainRenderer.fill(lge::vec4(255, 0, 255, 255));
						for (lge::vec2 c : contacts)
						{
							mainRenderer.circle(c.x, c.y, 8);
						}


						//polys[i].move(-m.normal[0] * m.penetration / 2);
						//polys[j].move(m.normal[0] * m.penetration / 2);

														
						lge::ResolveCollisionImproved(m, &polys[i], &polys[j]);
						//lge::ResolveCollisionImprovedNormalized(m, &polys[i], &polys[j]);
						//lge::CollisionData cData = lge::ResolveCollisionCollisionData(m, &polys[i], &polys[j]);
						//lge::ResolveCollisionWithoutRotation(m, &polys[i], &polys[j]);

						//data.collision = cData;

						//if(polys[i].m_invMass + polys[j].m_invMass != 0) serializer.addObject(data);

						//for (auto k = 0; k < contacts.size(); k++) mainRenderer.circle(contacts[k].x, contacts[k].y, 4);

						for (auto k = 0; k < 2; k++) {
							lge::vec2 n = m.normal[k];
							n *= 30;
							n += polys[i].m_position;
							mainRenderer.stroke(lge::vec4(255, 255, 255, 255));
							mainRenderer.line(polys[i].m_position.x, polys[i].m_position.y, n.x, n.y);
						}
					}
				}

				//for (auto j = 0; j < walls.size(); j++)
				//{
				//	continue;
				//	m = lge::PolygonCollisionSatManifold(&polys[i], &walls[j]);
				//	if (m.collided)
				//	{

				//		//lge::ResolveCollisionWithoutRotation(m, &polys[i], &walls[j]);

				//		lge::vec2 n = m.normal[0];
				//		m.normal[0] = m.normal[1];
				//		m.normal[1] = n;


				//		//lge::ResolveCollisionWithoutRotation(m, &walls[j], &polys[i]);
				//		//lge::ResolveCollisionImprovedNormalized(m, &polys[i], &walls[j]);
				//		std::vector<lge::vec2> contacts = lge::getContactPoints(&polys[i], &walls[j]);
				//		for (auto k = 0; k < contacts.size(); k++) mainRenderer.circle(contacts[k].x, contacts[k].y, 4);

				//		for (auto k = 0; k < 2; k++) {
				//			lge::vec2 n = m.normal[k];
				//			n *= 30;
				//			n += polys[i].m_position;
				//			mainRenderer.stroke(lge::vec4(255, 255, 255, 255));
				//			mainRenderer.line(polys[i].m_position.x, polys[i].m_position.y, n.x, n.y);
				//		}
				//	}
				//}
			}

			double minDist;
			lge::vec2 cp;

			for (auto k = 0; k < polys[1].m_transformedPoints.size(); k++)
			{
				if (k == 0)
				{
					minDist = FLT_MAX;
					cp = NULL;
				}

				lge::vec2 a = polys[1].m_transformedPoints[k];
				lge::vec2 b = polys[1].m_transformedPoints[(k + 1) % polys[1].m_transformedPoints.size()];
			}

			for (auto i = 0; i < polys.size(); i++)
				polys[i].update(mainRenderer.getDeltaTime());
		}

		for (auto i = 0; i < polys.size(); i++)
		{
			mainRenderer.stroke(lge::vec4(0, 255, 255, 255));
			mainRenderer.renderVec2List(polys[i].m_transformedPoints);
			std::vector<lge::vec2> normals = lge::getNormals(&polys[i]);
			normals = lge::addVec2ToVec2List(normals, polys[i].m_position);
			mainRenderer.stroke(lge::vec4(255, 0, 0, 255));
			for (auto j = 0; j < normals.size(); j++) mainRenderer.line(polys[i].m_position.x, polys[i].m_position.y, normals[j].x, normals[j].y);
		}

		mainRenderer.stroke(lge::vec4(0, 0, 255, 255));
		for (unsigned int i = 0; i < walls.size(); i++)
		{
			mainRenderer.renderVec2List(walls[i].m_transformedPoints);
		}

		if (keyPressed)
			mainRenderer.update();

		float fps = 1.f / clock.restart().asSeconds();

	}

	return 0;
}