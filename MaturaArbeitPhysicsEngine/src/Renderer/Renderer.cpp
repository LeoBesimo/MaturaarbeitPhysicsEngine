#include "Renderer.h"

Renderer::Renderer(unsigned int width, unsigned int height, std::string name)
{
	m_window = new sf::RenderWindow(sf::VideoMode(width, height), name);
	m_windowWidth = width;
	m_windowHeight = height;
}

Renderer::~Renderer()
{
	m_window->close();
	delete m_window;
	m_window = nullptr;
	exit(EXIT_SUCCESS);
}

bool Renderer::isRunning()
{
	return m_window->isOpen();
}

sf::RenderWindow* Renderer::getWindow()
{
	return m_window;
}

sf::RenderTarget* Renderer::getRenderTarget()
{
	return m_window;
}

std::vector<sf::Event> Renderer::getEvents()
{
	std::vector<sf::Event> events;
	sf::Event pollEvent;
	while (m_window->pollEvent(pollEvent))
	{
		events.push_back(pollEvent);
	}

	return events;
}

void Renderer::clear(lge::vec4 color = lge::vec4())
{
	m_window->clear(sf::Color(color.w, color.x, color.y, color.z));
}

void Renderer::update()
{
	m_window->display();
}

void Renderer::renderVec2List(std::vector<lge::vec2> &vectors)
{
	{
		sf::VertexArray lines(sf::Lines, vectors.size() * 2);
		unsigned int j = 0;
		for (unsigned int i = 0; i < vectors.size() * 2; i += 2)
		{
			lines[i].position = sf::Vector2f(vectors[j % vectors.size()].x, vectors[j % vectors.size()].y);
			lines[i + 1] = sf::Vector2f(vectors[(j + 1) % vectors.size()].x, vectors[(j + 1) % vectors.size()].y);
			j++;
		}

		for (unsigned int i = 0; i < lines.getVertexCount(); i++) lines[i].color = sf::Color::Black;

		//std::cout << lines.getVertexCount() << "\n";

		m_window->draw(lines);
	}
}
