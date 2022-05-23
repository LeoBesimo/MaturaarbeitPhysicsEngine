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

lge::vec2 Renderer::getMousePosition()
{
	sf::Vector2i mousePos = sf::Mouse::getPosition(*m_window);

	if (m_coordinateSystem)
	{
		double x = double(mousePos.x) / m_windowWidth * 2 - 1;
		double y = double(mousePos.y) / m_windowHeight * 2 - 1;
		return lge::vec2(x, y);
	}

	return lge::vec2(mousePos.x,mousePos.y);
}

lge::vec2 Renderer::getWindowSize()
{
	return lge::vec2(m_windowWidth,m_windowHeight);
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
		for (unsigned int i = 0; i < lines.getVertexCount(); i++) lines[i].color = m_strokeColor;

		unsigned int j = 0;
		if (m_coordinateSystem) 
		{
			for (unsigned int i = 0; i < vectors.size(); i++)
			{
				int index = (i + 1) % vectors.size();

				float x = (vectors[i].x + 1) / 2 * m_windowWidth;
				float y = (vectors[i].y + 1) / 2 * m_windowHeight;
				float nextX = (vectors[index].x + 1) / 2 * m_windowWidth;
				float nextY = (vectors[index].y + 1) / 2 * m_windowHeight;
				lines[j].position = sf::Vector2f(x,y);
				lines[j + 1].position = sf::Vector2f(nextX,nextY);
				j += 2;
			}

			m_window->draw(lines);
			return;
		}

		for (unsigned int i = 0; i < vectors.size(); i++)
		{
			int index = (i + 1) % vectors.size();
			lines[j].position = sf::Vector2f(vectors[i].x, vectors[i].y);
			lines[j + 1].position = sf::Vector2f(vectors[index].x, vectors[index].y);
			j += 2;
		}

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
		//EllipseShape ellipse(sf::Vector2f(scaledR1, scaledR2));
		sf::Vector2f radius(scaledR1, scaledR2);
		m_ellipseShape.setRadius(radius);
		m_ellipseShape.setPosition(scaledX - scaledR1 * m_circleMode, scaledY - scaledR2 * m_circleMode);
		m_ellipseShape.setFillColor(m_fillColor);
		m_ellipseShape.setOutlineColor(m_strokeColor);

		m_window->draw(m_ellipseShape);
		return;
	}

	//EllipseShape ellipse(sf::Vector2f(r1, r2));
	sf::Vector2f radius(r1, r2);

	m_ellipseShape.setRadius(radius);
	m_ellipseShape.setPosition(x - r1 * m_circleMode, y - r2 * m_circleMode);
	m_ellipseShape.setFillColor(m_fillColor);
	m_ellipseShape.setOutlineColor(m_strokeColor);

	m_window->draw(m_ellipseShape);
}

void Renderer::circle(float x, float y, float r)
{

	if (m_coordinateSystem)
	{
		float scaledR1 = (r) / 2 * m_windowWidth;
		float scaledR2 = (r) / 2 * m_windowHeight;
		float scaledX = (x + 1) / 2 * m_windowWidth;
		float scaledY = (y + 1) / 2 * m_windowHeight;
		//EllipseShape ellipse(sf::Vector2f(scaledR1, scaledR2));
		sf::Vector2f radius(scaledR1, scaledR2);
		m_ellipseShape.setRadius(radius);
		m_ellipseShape.setPosition(scaledX - scaledR1 * m_circleMode, scaledY - scaledR2 * m_circleMode);
		m_ellipseShape.setFillColor(m_fillColor);
		m_ellipseShape.setOutlineColor(m_strokeColor);

		m_window->draw(m_ellipseShape);
		return;
	}

	//sf::CircleShape circle(r);
	m_circleShape.setRadius(r);
	m_circleShape.setPosition(x - r * m_circleMode, y - r * m_circleMode);
	m_circleShape.setFillColor(m_fillColor);
	m_circleShape.setOutlineColor(m_strokeColor);

	m_window->draw(m_circleShape);
}

void Renderer::rect(float x, float y, float w, float h)
{
	if (m_coordinateSystem)
	{
		float scaledW = (w) / 2 * m_windowWidth;
		float scaledH = (h) / 2 * m_windowHeight;
		float scaledX = (x + 1) / 2 * m_windowWidth;
		float scaledY = (y + 1) / 2 * m_windowHeight;

		//sf::RectangleShape rect(sf::Vector2f(scaledW, scaledH));
		sf::Vector2f size(scaledW, scaledH);
		m_rectShape.setSize(size);

		m_rectShape.setPosition(scaledX - scaledW / 2 * m_rectMode, scaledY - scaledH / 2 * m_rectMode);
		m_rectShape.setFillColor(m_fillColor);
		m_rectShape.setOutlineColor(m_strokeColor);

		m_window->draw(m_rectShape);
		return;
	}

	//sf::RectangleShape rect(sf::Vector2f(w, h));

	sf::Vector2f size(w, h);
	m_rectShape.setSize(size);

	m_rectShape.setPosition(x - w / 2 * m_rectMode, y - h / 2 * m_rectMode);
	m_rectShape.setFillColor(m_fillColor);
	m_rectShape.setOutlineColor(m_strokeColor);

	m_window->draw(m_rectShape);
}
