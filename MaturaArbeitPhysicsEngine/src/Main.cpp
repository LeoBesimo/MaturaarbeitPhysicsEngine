#include <fstream>
#include <iostream>

#include <chrono>
#include <string.h>

#include "Physics/PhysicsWorld.h"

int main()
{
	srand(time(NULL));
	//srand(0);
	Renderer mainRenderer(1000, 1000, "Maturaarbeit Physiksimulation Leo Besimo");

	mainRenderer.getWindow()->setFramerateLimit(60);

	lge::vec2 windowSize = mainRenderer.getWindowSize();

	mainRenderer.circleMode(CENTER);
	mainRenderer.rectMode(CENTER);

	lge::PhysicsWorld world(&mainRenderer ,mainRenderer.getWindowSize());

	//lge::Polygon* poly = world.addBox(lge::vec2(200, 200), lge::vec2(50, 50), 0.25, false, 1, 1.0, lge::vec4(255, 0, 255, 255));

	//poly->m_velocity = lge::vec2(60, 0);

	//world.addBox(lge::vec2(400, 200), lge::vec2(50, 80), 0, false, 1, 1.0, lge::vec4(255, 0, 0, 255));

	//lge::Polygon* n5 = world.addPolygon(lge::vec2(600, 400), lge::mat2(30, 0, 0, 60), 0, 5, false, 1, 1.0, lge::vec4(255, 0, 255, 255));
	//n5->m_velocity += lge::vec2(80, 0);


	int borderWidth = 50;
	//world.addBox(lge::vec2(0, windowSize.y / 2), lge::vec2(borderWidth, windowSize.y), 0, true, 0, 1.0, lge::vec4(100, 100, 0, 255));
	//world.addBox(lge::vec2(windowSize.x, windowSize.y / 2), lge::vec2(borderWidth, windowSize.y), 0, true, 0, 1.0, lge::vec4(100, 200, 0, 255));
	//world.addBox(lge::vec2(windowSize.x / 2, 0), lge::vec2(windowSize.x, borderWidth), 0, true, 0, 1.0, lge::vec4(100, 0, 100, 255));
	lge::Polygon* p = world.addBox(lge::vec2(windowSize.x / 2, windowSize.y), lge::vec2(windowSize.x-200, borderWidth), 0, true, 0, 1.0, lge::Color::INDIGO);
	//p->m_density = 1;
	//p->calculateInertia();


	world.addBox(lge::vec2(300, 600), lge::vec2(300, borderWidth), lge::QUARTER_PI/3, true, 1, 1.0, lge::Color::WHITE);
	world.addBox(lge::vec2(700, 400), lge::vec2(300, borderWidth), -lge::QUARTER_PI / 3, true, 1, 1.0, lge::Color::WHITE);

	/*lge::Polygon* turntable = world.addBox(lge::vec2(500, 300), lge::vec2(200, 30), lge::HALF_PI*1.205, true, 1, 1.0, lge::Color::BROWN);
	turntable->m_density = 0.5;
	turntable->calculateInertia();*/


	double globalRestitution = 0.4;

	bool keyPressed = true;
	bool mousePressed = false;

	sf::Clock clock;
	float lastTime = 0;

#ifdef NDEBUG
	std::stringstream instructions;
	instructions << "Number keys: Set Bouncieness for new Bodies (1 = 0.1, 5 = 0.5, 0 = 1) Default = 0.4\n";
	instructions << "Key R: Delete all nonstatic Bodies\n";
	instructions << "Key E: Spawn Yellow Rectangle at (500,200) with Bounciness 1\n";
	instructions << "Key T: Spawn 6 Preprogrammed Bodies\n";
	instructions << "Left Mousebutton: Spawn Rectangle at Mouseposition with selected Bouncieness\n";
	instructions << "Right Mousebutton: Spawn Polygon at Mouseposition with selected Bouncieness\n";

	std::cout << instructions.str();
#endif

	while (mainRenderer.isRunning())
	{
		std::vector<sf::Event> events = mainRenderer.getEvents();

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
#ifdef _DEBUG
				if (event.key.code == sf::Keyboard::Escape)
				{
					if (world.getData("ugugubu.txt"))
					{
						world.~PhysicsWorld();
						mainRenderer.~Renderer();
						return 0;
					}
				}


				if (event.key.code == sf::Keyboard::Num1) world.setResolutionIndex(1);
				if (event.key.code == sf::Keyboard::Num2) world.setResolutionIndex(2);
				if (event.key.code == sf::Keyboard::Num3) world.setResolutionIndex(3);
				if (event.key.code == sf::Keyboard::Num4) world.setResolutionIndex(4);
				if (event.key.code == sf::Keyboard::Num5) world.setResolutionIndex(5);
				if (event.key.code == sf::Keyboard::Num6) world.setResolutionIndex(6);
#endif //

#ifdef NDEBUG
				if (event.key.code == sf::Keyboard::Num1) globalRestitution = 0.1;
				if (event.key.code == sf::Keyboard::Num2) globalRestitution = 0.2;
				if (event.key.code == sf::Keyboard::Num3) globalRestitution = 0.3;
				if (event.key.code == sf::Keyboard::Num4) globalRestitution = 0.4;
				if (event.key.code == sf::Keyboard::Num5) globalRestitution = 0.5;
				if (event.key.code == sf::Keyboard::Num6) globalRestitution = 0.6;
				if (event.key.code == sf::Keyboard::Num7) globalRestitution = 0.7;
				if (event.key.code == sf::Keyboard::Num8) globalRestitution = 0.8;
				if (event.key.code == sf::Keyboard::Num9) globalRestitution = 0.9;
				if (event.key.code == sf::Keyboard::Num0) globalRestitution = 1.0;
#endif // NDEBUG

				if (event.key.code == sf::Keyboard::T) world.testSetup();
				if (event.key.code == sf::Keyboard::R) world.reset();
				if (event.key.code == sf::Keyboard::E) world.addBox(lge::vec2(500, 200), lge::vec2(30, 50), 0.25, false, 1, 1, lge::Color::YELLOW);
				//if (event.key.code == sf::Keyboard::Z) world.addPolygon(mouse, lge::mat2(15,0,0, 15), 0, 25, false, 1, 0.4, lge::Color::LIGHTGRAY);

				keyPressed = !keyPressed;
			}

			if (event.type == sf::Event::MouseButtonPressed)
			{
				if(event.mouseButton.button == sf::Mouse::Button::Left)
					world.addBox(mouse, lge::vec2(lge::randomDouble(30, 50), lge::randomDouble(30, 50)), 0, false, 1, globalRestitution, lge::Color::GREEN);
				if (event.mouseButton.button == sf::Mouse::Button::Right)
					world.addPolygon(mouse, lge::mat2(lge::randomDouble(30, 50), 0, 0, lge::randomDouble(30, 50)), 0, 3 + rand() % 5, false, 0.5, globalRestitution, lge::Color::CYAN);
				if (!mousePressed)
				{

				}
				else
				{
					mousePressed = false;
				}
			}
		}

		mainRenderer.clear(lge::vec4(0, 0, 0, 255));

		world.update(mainRenderer.getDeltaTime());

		world.renderWorld();

		mainRenderer.displayFramerate();
		mainRenderer.stroke(lge::Color::CYAN);
		mainRenderer.text(std::to_string(globalRestitution),10,50,16);
		mainRenderer.update();

		float fps = 1.f / clock.restart().asSeconds();
	}

	return 0;
}