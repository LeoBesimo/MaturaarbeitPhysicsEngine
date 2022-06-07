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
		double torque = 0;

		vec2 m_acceleration;
		vec2 m_velocity = vec2();

		double m_restitution = 1;
		double m_mass = 10;
		double m_inertia = 10;

		bool m_isStatic = false;

		Polygon() {}

		Polygon(vec2 position, double angle, lge::mat2 scale, std::vector<vec2> originalPoints)
		{
			m_position = position;
			m_angle = angle;
			m_scale = scale;
			m_originalPoints = originalPoints;
			/*
			for (unsigned int i = 0; i < originalPoints.size(); i++)
			{                          
				vec2 middle = (originalPoints[(i + 1) % originalPoints.size()] + originalPoints[i]) / 2;
				vec2 edge = originalPoints[(i + 1) % originalPoints.size()] - originalPoints[i];
				m_edge.push_back(edge);
				m_edgeMiddle.push_back(middle);
				m_originalNormals.push_back(vec2(-edge.y, edge.x));
			}
			*/
			updateSides();
		}
		/*
		void updateNormals()
		{
			for (unsigned int i = 0; i < m_transformedEdge.size(); i++);
		}*/

		void updateSides()
		{
			mat2 rotation(m_angle);
			mat2 scaling = m_scale;
			mat2 transform = rotation * scaling;
			m_transformedPoints = m_originalPoints;
			m_transformedPoints = applyMat2ToVec2List(m_transformedPoints, transform);
			addVec2ToVec2List(m_transformedPoints, m_position);
		}

		void update()
		{
			m_velocity += m_acceleration;
			m_acceleration *= !m_isStatic;
			m_position += m_velocity;
			m_acceleration *= 0;

			m_angularVelocity += torque;
			m_angularVelocity *= !m_isStatic;
			m_angle += m_angularVelocity;
			torque *= 0;

			updateSides();
			/*
			m_transformedEdge = m_edge;
			m_transformedEdge = applyMat2ToVec2List(m_transformedEdge, transform);
			addVec2ToVec2List(m_transformedEdge, m_position);
			^*/
		}
	};
}