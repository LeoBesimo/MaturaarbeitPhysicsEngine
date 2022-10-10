#pragma once
#include <vector>
#include <sstream>
#include <SFML/Graphics.hpp>

#include "components.h"
#include "Utilities.h"
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

	long m_frameCount = 0;

	sf::Color m_fillColor = sf::Color::Transparent;
	sf::Color m_strokeColor = sf::Color::Black;

	Options m_circleMode = CORNERS;
	Options m_rectMode = CORNERS;

	CoordinateSystem m_coordinateSystem = PIXELS;

	EllipseShape m_ellipseShape;
	sf::CircleShape m_circleShape;
	sf::RectangleShape m_rectShape;
	sf::Text m_text;
	sf::Font m_font;

	sf::Clock m_deltaClock;
	double m_deltaTime = 0;
	sf::Clock m_framaRateClock;

	double m_frameRate;

public:

	Renderer(unsigned int width, unsigned int height, std::string name);
	~Renderer();
	
	bool isRunning();
	sf::RenderWindow* getWindow();
	sf::RenderTarget* getRenderTarget();

	std::vector<sf::Event> getEvents();

	lge::vec2 getMousePosition();
	lge::vec2 getWindowSize();

	double getDeltaTime();

	long getFrameCount();

	void clear(lge::vec4 color);
	void update();

	void fill(lge::vec4 color);
	void stroke(lge::vec4 color);

	void rectMode(Options rectMode);
	void circleMode(Options circleMode);
	void coordinateSystem(CoordinateSystem cs);

	void renderVec2List(std::vector<lge::vec2> &vectors);
	void renderVec2ListSolid(std::vector<lge::vec2> &vectors);

	void text(std::string text, float x, float y, int size);
	void displayFramerate();

	void line(float x1, float y1, float x2, float y2);
	void ellipse(float x, float y, float r1, float r2);
	void circle(float x, float y, float r);
	void rect(float x, float y, float w, float h);
};

