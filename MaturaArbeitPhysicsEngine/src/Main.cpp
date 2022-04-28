#include <SFML/Graphics.hpp>
#include "Renderer/Renderer.h"
#include "Renderer/CustomShapes/EllipseShape.h"

int main()
{
    Renderer mainRenderer(400,200,"Test");
    sf::CircleShape shape(100.f);
    shape.setFillColor(sf::Color::Green);
    //EllipseShape ellipse(sf::Vector2f(100, 50));
    EllipseShape ellipse(sf::Vector2f(100, 50));
    ellipse.setFillColor(sf::Color::Blue);
    ellipse.setPosition(0, 50);

    mainRenderer.circleMode(CENTER);
    mainRenderer.coordinateSystem(NORMALIZED);
    mainRenderer.rectMode(CENTER);

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
        mainRenderer.fill(lge::vec4(0, 128, 128, 255));
        //target->draw(ellipse);
        //mainRenderer.fill(lge::vec4(128, 0, 128, 255));
        //mainRenderer.ellipse(200, 200, 100, 100);
        //mainRenderer.rect(100, 100, 50, 50);
        mainRenderer.rect(0, 0, 1, 1);
        //mainRenderer.ellipse(0, 0, 0.5, 0.5);
        
        mainRenderer.update();
    }
    return 0;
}