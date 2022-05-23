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

		std::vector<vec2> m_originalNormals;
		std::vector<vec2> m_transformedNormals;

		std::vector<vec2> m_edgeMiddle;
		std::vector<vec2> m_edge;
		std::vector<vec2> m_transformedEdge;

		vec2 m_position;
		vec2 m_scale;

		double m_angle;

		vec2 m_acceleration;
		vec2 m_velocity;

		Polygon() {}

		Polygon(vec2 position, double angle, vec2 scale, std::vector<vec2> originalPoints)
		{
			m_position = position;
			m_angle = angle;
			m_scale = scale;
			m_originalPoints = originalPoints;
			
			for (unsigned int i = 0; i < originalPoints.size(); i++)
			{                          
				vec2 middle = (originalPoints[(i + 1) % originalPoints.size()] + originalPoints[i]) / 2;
				vec2 edge = originalPoints[(i + 1) % originalPoints.size()] - originalPoints[i];
				m_edge.push_back(edge);
				m_edgeMiddle.push_back(middle);
				m_originalNormals.push_back(vec2(-edge.y, edge.x));
			}
		
		}

		void updateNormals()
		{
			for (unsigned int i = 0; i < m_transformedEdge.size(); i++);
		}

		void update()
		{
			mat2 rotation(m_angle);
			mat2 scaling(m_scale.x, 0, 0, m_scale.y);
			mat2 transform = rotation * scaling;
			m_transformedPoints = m_originalPoints;
			m_transformedPoints = applyMat2ToVec2List(m_transformedPoints,transform);
			addVec2ToVec2List(m_transformedPoints,m_position);

			m_transformedEdge = m_edge;
			m_transformedEdge = applyMat2ToVec2List(m_transformedEdge, transform);
			addVec2ToVec2List(m_transformedEdge, m_position);
		}
	};
}