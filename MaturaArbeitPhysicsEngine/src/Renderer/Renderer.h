#pragma once
#include <vector>
#include <SFML/Graphics.hpp>

#include "components.h"

/*TODO:
* Add Line and Circle Render Functions
* Modyify renderVec2List function to make Loop Closing Toggleable
*/

class Renderer
{
private:
	sf::RenderWindow* m_window;

	unsigned int m_windowWidth = 0, m_windowHeight = 0;


public:

	Renderer(unsigned int width, unsigned int height, std::string name);
	~Renderer();
	
	bool isRunning();
	sf::RenderWindow* getWindow();
	sf::RenderTarget* getRenderTarget();

	std::vector<sf::Event> getEvents();

	void clear(lge::vec4 color);
	void update();

	void renderVec2List(std::vector<lge::vec2> &vectors);
};

