#include <SFML/Graphics.hpp>
#include "Renderer/Renderer.h"

int main()
{

    Renderer mainRenderer(200,200,"Test");
    //sf::RenderWindow window(sf::VideoMode(200, 200), "SFML works!");
    sf::CircleShape shape(100.f);
    shape.setFillColor(sf::Color::Green);

    sf::RenderTarget* target = mainRenderer.getRenderTarget();

    while (mainRenderer.isRunning())
    {
        std::vector<sf::Event> events = mainRenderer.getEvents();

        std::cout << events.size() << "\n";

        for (sf::Event &event : events)
        {
            if (event.type == sf::Event::Closed)
            {
                mainRenderer.~Renderer();
                return 0;
            }
        }

        

        mainRenderer.clear(lge::vec4(255, 0, 0, 255));
        target->draw(shape);
        mainRenderer.update();
    }

    return 0;
}