#include <SFML/Graphics.hpp>
#include "Renderer/Renderer.h"
#include "Renderer/CustomShapes/EllipseShape.h"

int main()
{
    Renderer mainRenderer(200,200,"Test");
    sf::CircleShape shape(100.f);
    shape.setFillColor(sf::Color::Green);
    //EllipseShape ellipse(sf::Vector2f(100, 50));
    EllipseShape ellipse(sf::Vector2f(100, 50));
    ellipse.setFillColor(sf::Color::Blue);
    ellipse.setPosition(0, 50);

    sf::RenderTarget* target = mainRenderer.getRenderTarget();

    while (mainRenderer.isRunning())
    {
        std::vector<sf::Event> events = mainRenderer.getEvents();

        for (sf::Event &event : events)
        {
            if (event.type == sf::Event::Closed)
            {
                mainRenderer.~Renderer();

                return 0;
            }
        }

        mainRenderer.clear(lge::vec4(255, 0, 0, 255));
        target->draw(ellipse);
        mainRenderer.update();
    }
    return 0;
}