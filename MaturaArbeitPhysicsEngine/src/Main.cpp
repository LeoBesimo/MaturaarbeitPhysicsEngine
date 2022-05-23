#include <fstream>
#include <iostream>

#include <chrono>

#include <SFML/Graphics.hpp>

#include "Renderer/Renderer.h"
#include "Renderer/CustomShapes/EllipseShape.h"

#include "components.h"
#include "utilities.h"

#include "Physics/CollisionDetection.h"

#include "FunctionTimer.h"

int main()
{
	Renderer mainRenderer(800, 800, "Test");


	std::vector<lge::vec2> vectors;

	for (double i = 0; i < lge::TWO_PI; i += lge::TWO_PI / 5)
	{
		vectors.push_back(lge::vec2(cos(i), sin(i)));
	}
	
	lge::Polygon poly(lge::vec2(0,-0.47), 0.0, lge::vec2(0.25, 0.25), vectors);

	vectors.clear();

	for (double i = 0; i < lge::TWO_PI; i += lge::TWO_PI / 16)
	{
		vectors.push_back(lge::vec2(cos(i), sin(i)));
	}

	lge::Polygon poly2(lge::vec2(0,0), 0.0, lge::vec2(0.25, 0.25), vectors);


	mainRenderer.circleMode(CENTER);
	mainRenderer.coordinateSystem(NORMALIZED);
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

		//poly.m_position = mouse;
		poly.m_angle = lge::map(mouse.x, -1, 1, 0, lge::TWO_PI);

		poly.update();
		poly.updateNormals();
		poly2.update();
		poly2.updateNormals();

		lge::PolygonCollisonSat(&poly, &poly2);

		mainRenderer.clear(lge::vec4(0, 0, 0, 255));

		mainRenderer.renderVec2List(poly.m_transformedPoints);
		mainRenderer.renderVec2List(poly2.m_transformedPoints);


		mainRenderer.stroke(lge::vec4(255, 0, 0, 255));



		mainRenderer.stroke(lge::vec4(0, 255, 0, 255));

		//mainRenderer.renderVec2List(poly.m_transformedEdge);
		//mainRenderer.renderVec2List(poly2.m_transformedEdge);

		mainRenderer.update();
	}

	return 0;
}