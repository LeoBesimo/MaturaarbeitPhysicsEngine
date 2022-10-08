#pragma once

#include <vector>

#include "utilities.h"
#include "components.h"

namespace lge
{
	struct Polygon
	{
		std::vector<vec2> m_originalPoints;
		std::vector<vec2> m_transformedPoints;
		/*
		std::vector<vec2> m_originalNormals;
		std::vector<vec2> m_transformedNormals;

		std::vector<vec2> m_edgeMiddle;
		std::vector<vec2> m_edge;
		std::vector<vec2> m_transformedEdge;*/

		vec2 m_position;
		lge::mat2 m_scale;

		double m_angle;

		double m_angularVelocity = 0;
		double m_torque = 0;

		vec2 m_force;
		vec2 m_velocity = vec2();

		double m_restitution = 1;
		double m_mass;
		double m_inertia;

		double m_area;

		double m_invMass;
		double m_invInertia;

		Polygon() {}

		Polygon(vec2 position, double angle, lge::mat2 scale, std::vector<vec2> originalPoints)
		{
			m_position = position;
			m_angle = angle;
			m_scale = scale;
			m_originalPoints = originalPoints;
			m_area = 0;
			calculateSides();
			//calculateInertia();
			//calculateInertiaIrregular();
		}

		float pureDistPointToLine(vec2 point, vec2 a, vec2 b) 
		{
			vec2 ab = b - a;
			vec2 ap = point - a;

			vec2 closest;

			double proj = dotVec2(ap, ab);
			double abLenSqr = ab.lenSqr();
			double d = proj / abLenSqr;

			if (d <= 0.0)
			{
				closest = a;
			}
			else if (d >= 1.0)
			{
				closest = b;
			}
			else
			{
				closest = a + (ab * d);
			}

			double dx = point.x - closest.x;
			double dy = point.y - closest.y;

			return dx * dx + dy * dy;

			//data.distSqr = lge::distSqr(ap, data.closest);
		}

		void setMass(double mass)
		{
			if (mass == 0)
			{
				m_mass = 0;
				m_invMass = 0;
			}
			else
			{
				m_mass = mass;
				m_invMass = 1 / mass;
			}

			//calculateMass(10);
			//setInertia(m_mass / 6 * (m_scale.x.x * m_scale.y.y));
		}

		void calculateMass(double density)
		{

			const unsigned int n = m_transformedPoints.size();

			m_mass = 0;
			double inertia = 0;
			double area = 0;

			for (unsigned int i = 0; i < n; i++)
			{
				vec2 p1 = m_transformedPoints[i];
				vec2 p2 = m_transformedPoints[(i + 1) % n];

				vec2 center = (m_position + p1 + p2) / 3;

				double d = dist(m_position,center);
				double height = sqrt(pureDistPointToLine(m_position, p1, p2));
				double triangleArea = 0.5 * (p2 - p1).len() * height;

				area += triangleArea;
			}

			m_mass = density * area;
			m_inertia = inertia;

			//m_mass = m_invMass == 0 ? 0 : 1 / m_mass;
			//this->m_invInertia = inertia == 0 ? 0 : 1 / inertia;

			std::cout << " " << m_mass << " " << m_inertia << "\n";

		}

		void calculateInertia()
		{
			double n = (double) m_originalPoints.size();

			double avgDist = 0;
			for (unsigned int i = 0; i < n; i++)
			{
				vec2 p1 = m_transformedPoints[i];
				vec2 p2 = m_transformedPoints[(i + 1) % (int) n];

				avgDist += dist(p1, p2);
			}

			avgDist /= n;

			double alpha = TWO_PI / n;

			double factor = n / 96.0;

			double aPow4 = avgDist * avgDist * avgDist * avgDist;

			double num = 2 + cos(alpha);
			double den = (1 - cos(alpha)) * (1 - cos(alpha));

			double factor2 = num / den;

			std::cout << alpha << " " << factor << " " << aPow4 << " " << factor2 << "\n";

			m_inertia = factor * aPow4 * factor2 * sin(alpha);
			m_invInertia = 1 / m_inertia;
			std::cout << m_inertia << " " << m_invInertia << "\n";
		}

		void calculateInertiaIrregular()
		{

			// https://de.wikipedia.org/wiki/Fl%C3%A4chentr%C3%A4gheitsmoment 
			double minDist = FLT_MAX;
			double maxDist = FLT_MIN;

			for (unsigned int i = 0; i < m_transformedPoints.size(); i++) 
			{
				auto j = (i + 1) % m_transformedPoints.size();

				vec2 p1 = m_transformedPoints[i];
				vec2 p2 = m_transformedPoints[j];

				double d = pureDistPointToLine(m_position, p1, p2);
				if (d < minDist) minDist = d;
				if (d > maxDist) maxDist = d;
			}

			m_inertia = (PI / 4) * (maxDist * maxDist * maxDist * minDist);
			m_invInertia = 1 / m_inertia;
			std::cout << m_inertia << " " << m_invInertia << "\n";
		}

		void move(vec2 distance)
		{
			if (m_mass == 0) return;
			m_position += distance;
			addVec2ToVec2List(m_transformedPoints, distance);
		}

		void rotate(double angle)
		{
			if (m_mass == 0) return;
			m_angle += angle;
			calculateSides();
		}

		void setInertia(double inertia)
		{
			if (inertia == 0)
			{
				m_inertia = 0;
				m_invInertia = 0;
			}
			else
			{
				m_inertia = inertia;
				m_invInertia = 1 / m_inertia;
			}
		}

		void calculateSides()
		{
			mat2 rotation(m_angle);
			mat2 scaling = m_scale;
			mat2 transform = rotation * scaling;
			m_transformedPoints = m_originalPoints;
			m_transformedPoints = applyMat2ToVec2List(m_transformedPoints, transform);
			addVec2ToVec2List(m_transformedPoints, m_position);
		}

		void integrateForces(double dt)
		{
			m_velocity += (m_force) * m_invMass * (dt);
			m_force *= 0;

			m_angularVelocity += m_torque * m_invInertia * (dt);
			m_torque = 0;
		
		}

		void update(double dt)
		{	
			move(m_velocity * dt);

			//m_position += m_velocity * dt;
			//m_force *= 0;
			
			rotate(m_angularVelocity * dt);

			//m_angle += m_angularVelocity * dt;
			//m_torque *= 0;

			//updateSides();
			/*
			m_transformedEdge = m_edge;
			m_transformedEdge = applyMat2ToVec2List(m_transformedEdge, transform);
			addVec2ToVec2List(m_transformedEdge, m_position);
			^*/
		}
	};
}