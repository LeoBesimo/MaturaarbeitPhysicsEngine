#pragma once
#include <vector>
#include <SFML/Graphics.hpp>

#include "components.h"
#include "CustomShapes/EllipseShape.h"

/*TODO:
* Test Performance of renderer
* Make shapes used by the renderer Member variables for less memory consumption and better performance
* Modyify renderVec2List function to make Loop Closing Toggleable
*/

enum Options
{
	CORNERS = 0,
	CENTER = 1
};

enum CoordinateSystem
{
	PIXELS = 0,
	NORMALIZED = 1
};

class Renderer
{
private:
	sf::RenderWindow* m_window;

	unsigned int m_windowWidth = 0, m_windowHeight = 0;

	sf::Color m_fillColor = sf::Color::Transparent;
	sf::Color m_strokeColor = sf::Color::Black;

	Options m_circleMode = CORNERS;
	Options m_rectMode = CORNERS;

	CoordinateSystem m_coordinateSystem = PIXELS;

	EllipseShape m_ellipseShape;
	sf::CircleShape m_circleShape;
	sf::RectangleShape m_rectShape;

	//float m_strokeWeight = 1;

public:

	Renderer(unsigned int width, unsigned int height, std::string name);
	~Renderer();
	
	bool isRunning();
	sf::RenderWindow* getWindow();
	sf::RenderTarget* getRenderTarget();

	std::vector<sf::Event> getEvents();

	lge::vec2 getMousePosition();

	void clear(lge::vec4 color);
	void update();

	void fill(lge::vec4 color);
	void stroke(lge::vec4 color);

	void rectMode(Options rectMode);
	void circleMode(Options circleMode);
	void coordinateSystem(CoordinateSystem cs);

	void renderVec2List(std::vector<lge::vec2> &vectors);

	void line(float x1, float y1, float x2, float y2);
	void ellipse(float x, float y, float r1, float r2);
	void circle(float x, float y, float r);
	void rect(float x, float y, float w, float h);
};

