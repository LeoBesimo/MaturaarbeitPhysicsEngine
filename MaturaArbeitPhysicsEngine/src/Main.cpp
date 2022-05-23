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

int main()
{
	Renderer mainRenderer(800, 800, "Test");


	std::vector<lge::vec2> vectors;

	for (double i = 0; i < lge::TWO_PI; i += lge::TWO_PI / 4)
	{
		//vectors.push_back(lge::vec2(cos(i), sin(i)));
	}

	vectors.push_back(lge::vec2(1, -1));
	vectors.push_back(lge::vec2(1, 1));
	vectors.push_back(lge::vec2(-1, 1));
	vectors.push_back(lge::vec2(-1, -1));
	vectors.push_back(lge::vec2(0, -1.5));
	
	lge::Polygon poly(lge::vec2(200,400), lge::QUARTER_PI, lge::vec2(50, 50), vectors);
	poly.m_velocity = lge::vec2(5, 0);
	poly.m_mass = 100;

	vectors.clear();

	for (double i = 0; i < lge::TWO_PI; i += lge::TWO_PI / 9)
	{
		vectors.push_back(lge::vec2(cos(i), sin(i)));
	}

	lge::Polygon poly2(lge::vec2(400,400), lge::QUARTER_PI/2, lge::vec2(100, 100), vectors);
	//poly2.m_velocity = lge::vec2(-5, 0);

	mainRenderer.circleMode(CENTER);
	//mainRenderer.coordinateSystem(NORMALIZED);
	mainRenderer.rectMode(CENTER);

	int i = 0;


	lge::vec2 testVec;

	while (mainRenderer.isRunning() && i < 1000)
	{
		std::vector<sf::Event> events = mainRenderer.getEvents();

		for (auto& event : events)
		{
			if (event.type == sf::Event::Closed)
			{
				mainRenderer.~Renderer();

				return 0;
			}
		}

		lge::vec2 mouse = mainRenderer.getMousePosition();

		mainRenderer.stroke(lge::vec4(0, 255, 255, 255));
		mainRenderer.fill(lge::vec4(255, 0, 255, 255));

		poly.m_position = mouse;
		//poly.m_angle = lge::map(mouse.x, -1, 1, 0, lge::TWO_PI);

		poly.update();
		//poly.updateNormals();
		poly2.update();
		//poly2.updateNormals();

		//lge::PolygonCollisonSat(&poly, &poly2);
		lge::Manifold manifold = lge::PolygonCollisionSatManifold(&poly, &poly2);
		//std::cout << manifold.collided << "\n";
		//if (manifold.collided) lge::ResolveCollision(manifold ,&poly, &poly2);
		//if (manifold.collided) lge::ResolveCollision(manifold ,&poly2, &poly);

		//std::cout << poly2.m_velocity << "\n";

		mainRenderer.clear(lge::vec4(0, 0, 0, 255));

		mainRenderer.renderVec2List(poly.m_transformedPoints);
		mainRenderer.renderVec2List(poly2.m_transformedPoints);

		if (manifold.collided)
		{
			mainRenderer.stroke(lge::vec4(255, 0, 0, 255));

			lge::vec2 point1 = poly.m_transformedPoints[manifold.indexP1[0]];
			lge::vec2 point2 = poly.m_transformedPoints[manifold.indexP1[1]];
			lge::vec2 point3 = poly2.m_transformedPoints[manifold.indexP2[0]];
			lge::vec2 point4 = poly2.m_transformedPoints[manifold.indexP2[1]];

			mainRenderer.circle(point1.x, point1.y, 4);
			mainRenderer.circle(point3.x, point3.y, 4);

			mainRenderer.fill(lge::vec4(0, 255, 0, 255));
			mainRenderer.circle(point2.x, point2.y, 4);

			mainRenderer.circle(point4.x, point4.y, 4);

			int index1 = manifold.normalIndex[0];
			int index2 = manifold.normalIndex[1];

			lge::vec2 p1 = poly.m_transformedPoints[(index1) % poly.m_transformedPoints.size()];
			lge::vec2 p2 = poly.m_transformedPoints[(index1 + 1) % poly.m_transformedPoints.size()];
			lge::vec2 p3 = poly2.m_transformedPoints[(index2) % poly2.m_transformedPoints.size()];
			lge::vec2 p4 = poly2.m_transformedPoints[(index2 + 1) % poly2.m_transformedPoints.size()];

			mainRenderer.stroke(lge::vec4(255, 0, 0, 255));
			mainRenderer.line(p1.x, p1.y, p2.x, p2.y);
			mainRenderer.line(p3.x, p3.y, p4.x, p4.y);

		}



		//if (manifold.collided) mainRenderer.line(poly.m_position.x, poly.m_position.y, manifold.normal[0].x,manifold.normal[0].y);
		//if (manifold.collided) mainRenderer.line(poly2.m_position.x, poly2.m_position.y, manifold.normal[1].x, manifold.normal[1].y);

		mainRenderer.stroke(lge::vec4(0, 255, 0, 255));

		//mainRenderer.renderVec2List(poly.m_transformedEdge);
		//mainRenderer.renderVec2List(poly2.m_transformedEdge);

		mainRenderer.update();
	}

	return 0;
}