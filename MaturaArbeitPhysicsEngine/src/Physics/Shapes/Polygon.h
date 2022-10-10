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

		vec2 m_position;
		lge::mat2 m_scale;

		double m_angle;

		double m_angularVelocity = 0;
		double m_torque = 0;

		vec4 m_color = vec4(255,255,255,255);

		vec2 m_force;
		vec2 m_velocity = vec2();

		double m_restitution = 1;
		double m_mass;
		double m_inertia;

		double m_density;

		double m_invMass;
		double m_invInertia;

		Polygon() {}

		Polygon(vec2 position, double angle, lge::mat2 scale, std::vector<vec2> originalPoints)
		{
			m_position = position;
			m_angle = angle;
			m_scale = scale;
			m_originalPoints = originalPoints;
			calculateSides();
		}

		void setMass(double mass)
		{
			if (mass == 0)
			{
				m_density = 0;
				m_mass = 0;
				m_invMass = 0;
			}
			else
			{
				m_mass = mass;
				m_invMass = 1 / mass;
			}
		}

		void calculateMass(double density)
		{

			const unsigned int n = m_transformedPoints.size();

			m_mass = 0;
			m_density = density;
			double inertia = 0;
			double area = 0;

			for (unsigned int i = 0; i < n; i++)
			{
				vec2 p1 = m_transformedPoints[i];
				vec2 p2 = m_transformedPoints[(i + 1) % n];

				vec2 center = (m_position + p1 + p2) / 3;

				double d = dist(m_position,center);
				double height = sqrt(distPointToLine(m_position, p1, p2).distSqr);
				double triangleArea = 0.5 * (p2 - p1).len() * height;

				area += triangleArea;
			}

			m_mass = density * area;
			m_inertia = inertia;

			m_invMass = 1 / m_mass;
		}

		void calculateInertia()
		{
			int n = m_transformedPoints.size();

			double totalInertia = 0;

			for (unsigned int i = 0; i < n; i++)
			{
				vec2 p1 = m_transformedPoints[i];
				vec2 p2 = m_transformedPoints[(i + 1) % n];
				vec2 center = m_position;
				
				PLData data = distPointToLine(center, p1, p2);

				vec2 triangleCenter1 = (center + p1 + data.closest) / 3;
				vec2 triangleCenter2 = (center + p2 + data.closest) / 3;


				double b1 = distSqr(p1, data.closest);
				double b2 = distSqr(p2, data.closest);

				double area1 = sqrt(data.distSqr + b1) / 2;
				double area2 = sqrt(data.distSqr + b2) / 2;

				double mass1 = area1 * m_density;
				double mass2 = area2 * m_density;

				double j1 = (mass1 / 18) * (b1 + data.distSqr);
				double j2 = (mass2 / 18) * (b2 + data.distSqr);

				totalInertia += j1 * distSqr(center, triangleCenter1);
				totalInertia += j2 * distSqr(center, triangleCenter2);
			}

			m_inertia = totalInertia;
			m_invInertia = 1 / totalInertia;
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
			rotate(m_angularVelocity * dt);
		}

		void update(double dt, int stepCount)
		{
			move(m_velocity * dt / stepCount);
			rotate(m_angularVelocity * dt / stepCount);
		}
	};
}