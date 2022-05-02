#include <fstream>
#include <iostream>

#include <chrono>

#include <SFML/Graphics.hpp>

#include "Renderer/Renderer.h"
#include "Renderer/CustomShapes/EllipseShape.h"

#include "Utilities.h"

int main()
{
	Renderer mainRenderer(400, 400, "Test");

	//EllipseShape ellipse(sf::Vector2f(100, 50));

	mainRenderer.circleMode(CENTER);
	mainRenderer.coordinateSystem(NORMALIZED);
	mainRenderer.rectMode(CENTER);

	//sf::RenderTarget* target = mainRenderer.getRenderTarget();

	//std::vector<double> timeMeasurement;
	int i = 0;

	lge::FunctionTimer timer;

	//lge::vec2 testVec;

	while (mainRenderer.isRunning() && i < 100)
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

		//auto start = std::chrono::high_resolution_clock::now();

		//timer.start();

		mainRenderer.clear(lge::vec4(255, 255, 255, 255));
		mainRenderer.fill(lge::vec4(0, 128, 128, 255));

		mainRenderer.ellipse(0, 0, 0.75, 0.5);

		mainRenderer.update();
		//timer.end();
		//timer.save("Rendertime Ellipse Normalized 3");
		//auto end = std::chrono::high_resolution_clock::now();

		//std::chrono::duration<double, std::milli> duration = end - start;
		//std::cout << duration.count() << " ms\n";

		//timeMeasurement.push_back(duration.count());
		
		i++;
	}
	/*
	std::ofstream outdata;
	outdata.open("Rendertime Ellipse Normalized2.txt");

	outdata << "Parameters Passed into function: 0, 0, 0.5, 0.5\n";

	if (!outdata) {
		std::cerr << "File could not be opened\n";
		exit(1);
	}
	double avg = 0;
	double min = INFINITY;
	double max = -INFINITY;
	for (int j = 0; j < timeMeasurement.size(); j++)
	{
		double val = timeMeasurement[j];
		avg += val;
		if (val < min) min = val;
		if (val > max) max = val;

		outdata << "Frame: " << j << " Time: " << val << "ms\n";
	}

	avg /= timeMeasurement.size();

	outdata << "Avg: " << avg << "ms Min: " << min << "ms Max: " << max << "ms\n";

	outdata << "\n";
	
	for (int j = 0; j < timeMeasurement.size(); j++)
	{
		outdata << timeMeasurement[j] << "\n";
	}

	outdata.close();
	*/
	return 0;
}

//mainRenderer.fill(lge::vec4(128, 0, 128, 255));
		//mainRenderer.ellipse(200, 200, 100, 100);
		//mainRenderer.rect(100, 100, 50, 50);
		//mainRenderer.rect(200, 200, 200, 200);
		//mainRenderer.circle(0, 0, 0.5);
		//mainRenderer.ellipse(0, 0, 0.5, 1);
		//mainRenderer.rect(0, 0, 1, 1);
		//mainRenderer.ellipse(0, 0, 0.5, 0.5);