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

void Renderer::fill(lge::vec4 color)
{
	m_fillColor = sf::Color(color.w, color.x, color.y, color.z);
}

void Renderer::stroke(lge::vec4 color)
{
	m_strokeColor = sf::Color(color.w, color.x, color.y, color.z);
}

void Renderer::rectMode(Options rectMode)
{
	m_rectMode = rectMode;
}

void Renderer::circleMode(Options circleMode)
{
	m_circleMode = circleMode;
}

void Renderer::coordinateSystem(CoordinateSystem cs)
{
	m_coordinateSystem = cs;
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

void Renderer::line(float x1, float y1, float x2, float y2)
{
	if (m_coordinateSystem)
	{
		sf::Vertex line[] =
		{
			sf::Vertex(sf::Vector2f((x1 + 1) / 2 * m_windowWidth, (y1 + 1) / 2 * m_windowHeight),m_strokeColor),
			sf::Vertex(sf::Vector2f((x2 + 1) / 2 * m_windowWidth, (y2 + 1) / 2 * m_windowHeight), m_strokeColor)
		};
		m_window->draw(line, 2, sf::Lines);
		return;
	}

	sf::Vertex line[] =
	{
		sf::Vertex(sf::Vector2f(x1,y1),m_strokeColor),
		sf::Vertex(sf::Vector2f(x2,y2), m_strokeColor)
	};
	m_window->draw(line, 2, sf::Lines);
}

void Renderer::ellipse(float x, float y, float r1, float r2)
{
	if (m_coordinateSystem)
	{
		float scaledR1 = (r1) / 2 * m_windowWidth;
		float scaledR2 = (r2) / 2 * m_windowHeight;
		float scaledX = (x + 1) / 2 * m_windowWidth;
		float scaledY = (y + 1) / 2 * m_windowHeight;
		EllipseShape ellipse(sf::Vector2f(scaledR1, scaledR2));
		ellipse.setPosition(scaledX - scaledR1 * m_circleMode, scaledY - scaledR2 * m_circleMode);
		ellipse.setFillColor(m_fillColor);
		ellipse.setOutlineColor(m_strokeColor);

		m_window->draw(ellipse);
		return;
	}

	EllipseShape ellipse(sf::Vector2f(r1, r2));
	ellipse.setPosition(x - r1 * m_circleMode, y - r2 * m_circleMode);
	ellipse.setFillColor(m_fillColor);
	ellipse.setOutlineColor(m_strokeColor);

	m_window->draw(ellipse);
}

void Renderer::circle(float x, float y, float r)
{

	if (m_coordinateSystem)
	{
		float scaledR1 = (r) / 2 * m_windowWidth;
		float scaledR2 = (r) / 2 * m_windowHeight;
		float scaledX = (x + 1) / 2 * m_windowWidth;
		float scaledY = (y + 1) / 2 * m_windowHeight;
		EllipseShape ellipse(sf::Vector2f(scaledR1, scaledR2));
		ellipse.setPosition(scaledX - scaledR1 * m_circleMode, scaledY - scaledR2 * m_circleMode);
		ellipse.setFillColor(m_fillColor);
		ellipse.setOutlineColor(m_strokeColor);

		m_window->draw(ellipse);
		return;
	}

	sf::CircleShape circle(r);
	circle.setPosition(x - r * m_circleMode, y - r * m_circleMode);
	circle.setFillColor(m_fillColor);
	circle.setOutlineColor(m_strokeColor);

	m_window->draw(circle);
}

void Renderer::rect(float x, float y, float w, float h)
{
	if (m_coordinateSystem)
	{
		float scaledW = (w) / 2 * m_windowWidth;
		float scaledH = (h) / 2 * m_windowHeight;
		float scaledX = (x + 1) / 2 * m_windowWidth;
		float scaledY = (y + 1) / 2 * m_windowHeight;

		sf::RectangleShape rect(sf::Vector2f(scaledW, scaledH));

		rect.setPosition(scaledX - scaledW / 2 * m_rectMode, scaledY - scaledH / 2 * m_rectMode);
		rect.setFillColor(m_fillColor);
		rect.setOutlineColor(m_strokeColor);

		m_window->draw(rect);
		return;
	}

	sf::RectangleShape rect(sf::Vector2f(w, h));

	rect.setPosition(x - w / 2 * m_rectMode, y - h / 2 * m_rectMode);
	rect.setFillColor(m_fillColor);
	rect.setOutlineColor(m_strokeColor);

	m_window->draw(rect);
}
