#pragma once

#include <random>
#include <sstream>

#include "Components.h"

namespace lge
{
	typedef std::string uuid;


	static std::random_device              rd;
	static std::mt19937                    gen(rd());
	static std::uniform_int_distribution<> dis(0, 15);
	static std::uniform_int_distribution<> dis2(8, 11);

	
	int hash(int state);
	double randomDouble(double min, double man);
	vec2 random2D();
	vec2 signRandom2D();
	vec3 random3D();
	vec4 random4D();
	double min(double a, double b);
	double max(double a, double b);
	double clamp(double a, double low, double hi);
	vec2 minVec2(vec2 a, vec2 b);
	vec2 maxVec2(vec2 a, vec2 b);
	vec2 clampVec2(vec2 a, vec2 min, vec2 max);
	vec2 absVec2(vec2 a);
	double map(double val, double oldMin, double oldMax, double newMin, double newMax);
	double constrain(double val, double min, double max);
	vec2 constrainVec2(vec2 a, vec2 min, vec2 max);
	vec2 mapVec2(vec2 a, vec2 oldMin, vec2 oldMax, vec2 newMin, vec2 newMax);

	uuid generateUUIDv4();

	std::vector<vec2> applyMat2ToVec2List(std::vector<vec2> &vectors, mat2 matrix);
	std::vector<vec2> addVec2ToVec2List(std::vector<vec2>& vectors, vec2 toAdd);
	std::vector<vec2> multVec2ToVec2List(std::vector<vec2>& vectors, vec2 toMult);
	std::vector<vec2> subVec2ToVec2List(std::vector<vec2>& vectors, vec2 toSub);
	std::vector<vec2> divVec2ToVec2List(std::vector<vec2>& vectors, vec2 toDiv);
	std::vector<vec2> mapVec2List(std::vector<vec2>& vectors, vec2 oldMin, vec2 oldMax, vec2 newMin, vec2 newMax);
	vec2 avgVec2List(std::vector<vec2>& vectors);
}

