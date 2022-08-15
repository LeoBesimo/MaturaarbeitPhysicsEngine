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

/*
	fix normal generation for uneven side count
*/

int main()
{

	srand(time(NULL));
	Renderer mainRenderer(800, 800, "Test");

	mainRenderer.circleMode(CENTER);
	mainRenderer.rectMode(CENTER);
	//mainRenderer.coordinateSystem(NORMALIZED);

	std::vector<lge::vec2> squareEdges;
	std::vector<lge::Polygon> walls;
	std::vector<lge::Polygon> polys;

	for (double i = 0; i < lge::TWO_PI; i += lge::TWO_PI / 4)
	{
		squareEdges.push_back(lge::vec2(cos(i), sin(i)));
	}

	//walls.push_back(lge::Polygon(lge::vec2(0, 400), lge::QUARTER_PI, lge::mat2(400, 380, 380, 400), squareEdges));
	//walls.push_back(lge::Polygon(lge::vec2(800, 400), lge::QUARTER_PI, lge::mat2(400, 380, 380, 400), squareEdges));
	//walls.push_back(lge::Polygon(lge::vec2(400, 0), lge::QUARTER_PI, lge::mat2(380, -400, -400, 380), squareEdges));
	//walls.push_back(lge::Polygon(lge::vec2(400, 800), lge::QUARTER_PI, lge::mat2(380, -400, -400, 380), squareEdges));

	std::vector<lge::vec2> vecs;
	vecs.push_back(lge::vec2(0, 0));
	vecs.push_back(lge::vec2(800, 0));
	vecs.push_back(lge::vec2(800, 20));
	vecs.push_back(lge::vec2(0, 20));

	walls.push_back(lge::Polygon(lge::vec2(0, 0), 0, lge::mat2(1,0,0,1), vecs));

	vecs.clear();

	vecs.push_back(lge::vec2(780, 0));
	vecs.push_back(lge::vec2(800, 0));
	vecs.push_back(lge::vec2(800, 800));
	vecs.push_back(lge::vec2(780, 800));

	walls.push_back(lge::Polygon(lge::vec2(0, 0), 0, lge::mat2(1,0,0,1), vecs));

	vecs.clear();

	vecs.push_back(lge::vec2(0, 780));
	vecs.push_back(lge::vec2(800, 780));
	vecs.push_back(lge::vec2(800, 800));
	vecs.push_back(lge::vec2(0, 800));


	walls.push_back(lge::Polygon(lge::vec2(0, 0), 0, lge::mat2(1,0,0,1), vecs));
	
	vecs.clear();

	vecs.push_back(lge::vec2(0, 0));
	vecs.push_back(lge::vec2(20, 0));
	vecs.push_back(lge::vec2(20, 800));
	vecs.push_back(lge::vec2(0, 800));

	walls.push_back(lge::Polygon(lge::vec2(0, 0), 0, lge::mat2(1,0,0,1), vecs));


	for (unsigned int i = 0; i < walls.size(); i++)
	{
		walls[i].m_isStatic = true;
		//walls[i].m_restitution = 1.0f;
		//walls[i].m_mass = FLT_MAX;
		//walls[i].update();
	}

	std::vector<lge::vec2> vectors;

	for (double i = 0; i < lge::TWO_PI; i += lge::TWO_PI / 4)
	{
		vectors.push_back(lge::vec2(cos(i), sin(i)));
	}

	//lge::Polygon poly(lge::vec2(-0.5, 0), -lge::QUARTER_PI / 2, lge::mat2(0.2, 0, 0, 0.2), vectors);
	lge::Polygon poly(lge::vec2(200, 400), -lge::QUARTER_PI/3, lge::mat2(51, 0, 0, 51), vectors);
	poly.m_velocity = lge::vec2(3, 0);
	//poly.m_velocity = lge::vec2(0.01, 0);
	//poly.m_mass = 100;
	//poly.m_angularVelocity = 0.05;

	vectors.clear();
	for (double i = 0; i < lge::TWO_PI; i += lge::TWO_PI / 5)
	{
		vectors.push_back(lge::vec2(cos(i), sin(i)));
	}

	//lge::Polygon poly2(lge::vec2(0, 0), 0, lge::mat2(0.2, 0, 0, 0.2), vectors);
	lge::Polygon poly2(lge::vec2(400, 400), -lge::QUARTER_PI, lge::mat2(50, 0, 0, 50), vectors);
	//poly2.m_velocity = lge::vec2(-0.01, 0);

	polys.push_back(poly);

	//poly2.m_isStatic = true;
	polys.push_back(poly2);

	for (lge::Polygon wall : walls)
	{
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

			//polys[0].m_position = mouse;
			//polys[0].m_velocity = lge::vec2();

			for (auto i = 0; i < polys.size(); i++)
				polys[i].update();

			for (auto i = 0; i < polys.size(); i++)
			{
				//polys[i].m_velocity += lge::vec2(0, 0.8);
				//polys[i].update();

				lge::Manifold m;

				for (auto j = 0; j < polys.size(); j++)
				{
					if (i == j) continue;

					m = lge::PolygonCollisionSatManifold(&polys[i], &polys[j]);



					if (m.collided)//lge::AABBCollision(&polys[i],&polys[j]))
					{

						//m = lge::PolygonCollisionSatManifold(&polys[i], &polys[j]);


						std::vector<lge::vec2> contacts = lge::getContactPoints(&polys[i], &polys[j]);
						mainRenderer.fill(lge::vec4(255, 0, 255, 255));
						for (lge::vec2 c : contacts)
						{
							mainRenderer.circle(c.x, c.y, 8);
						}

														
						//lge::ResolveCollisionImprovedNormalized(m, &polys[i], &polys[j]);
						//lge::ResolveCollisionImprovedNormalized(m, &polys[i], &polys[j]);
						lge::ResolveCollision(m, &polys[i], &polys[j]);
						//lge::ResolveCollisionWithoutRotation(m, &polys[i], &polys[j]);

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


				for (auto j = 0; j < walls.size(); j++)
				{
					continue;
					m = lge::PolygonCollisionSatManifold(&polys[i], &walls[j]);
					if (m.collided)
					{

						//lge::ResolveCollisionWithoutRotation(m, &polys[i], &walls[j]);

						lge::vec2 n = m.normal[0];
						m.normal[0] = m.normal[1];
						m.normal[1] = n;

						//lge::ResolveCollisionWithoutRotation(m, &walls[j], &polys[i]);
						//lge::ResolveCollisionImprovedNormalized(m, &polys[i], &walls[j]);
						std::vector<lge::vec2> contacts = lge::getContactPoints(&polys[i], &walls[j]);
						for (auto k = 0; k < contacts.size(); k++) mainRenderer.circle(contacts[k].x, contacts[k].y, 4);

						for (auto k = 0; k < 2; k++) {
							lge::vec2 n = m.normal[k];
							n *= 30;
							n += polys[i].m_position;
							mainRenderer.stroke(lge::vec4(255, 255, 255, 255));
							mainRenderer.line(polys[i].m_position.x, polys[i].m_position.y, n.x, n.y);
						}
					}
				}
			}
		}

		for (auto i = 0; i < polys.size(); i++)
		{
			mainRenderer.stroke(lge::vec4(0, 255, 255, 255));
			mainRenderer.renderVec2List(polys[i].m_transformedPoints);
			std::vector<lge::vec2> normals = lge::getNormals(&polys[i]);
			//normals = lge::multVec2ToVec2List(normals, lge::vec2(20, 20));
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

		//float currentTime = clock.restart().asMilliseconds();
		float fps = 1.f / clock.restart().asSeconds();
		//std::cout << "Framerate: " << fps << "\n";
		//lastTime = currentTime;

	}

	return 0;
}