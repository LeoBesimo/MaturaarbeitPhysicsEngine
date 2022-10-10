#pragma once
#include <random>
#include <sstream>
#include <math.h>

#include "lgeMath.h"
#include "components.h"

namespace lge
{
	typedef std::string uuid;

	static std::random_device              rd;
	static std::mt19937                    gen(rd());
	static std::uniform_int_distribution<> dis(0, 15);
	static std::uniform_int_distribution<> dis2(8, 11);

	//useful helper functions
	inline int hash(int state);
	inline double randomDouble(double min, double man);
	inline vec2 random2D();
	inline vec2 signRandom2D();
	inline vec3 random3D();
	inline vec4 random4D();
	inline double min(double a, double b);
	inline double max(double a, double b);
	inline double clamp(double a, double low, double hi);  //redundant with constrain
	inline vec2 minVec2(vec2 a, vec2 b);
	inline vec2 maxVec2(vec2 a, vec2 b);
	inline vec2 clampVec2(vec2 a, vec2 min, vec2 max);  //redundant with constrainVec2
	inline vec2 absVec2(vec2 a);
	inline double map(double val, double oldMin, double oldMax, double newMin, double newMax);
	inline double constrain(double val, double min, double max);
	inline vec2 constrainVec2(vec2 a, vec2 min, vec2 max);
	inline vec2 mapVec2(vec2 a, vec2 oldMin, vec2 oldMax, vec2 newMin, vec2 newMax);
	inline double dist(double x, double y, double x2, double y2);
	inline double dist(vec2 a, vec2 b);
	inline double distSqr(vec2 a, vec2 b);
	inline bool nearlyEqual(double a, double b);
	inline bool nearlyEqual(vec2 a, vec2 b);
	inline PLData distPointToLine(vec2 point, vec2 a, vec2 b);

	inline uuid generateUUIDv4();

	//applies specific things to a list of 2d vectors
	inline std::vector<vec2> applyMat2ToVec2List(std::vector<vec2>& vectors, mat2 matrix);
	inline std::vector<vec2> addVec2ToVec2List(std::vector<vec2>& vectors, vec2 toAdd);
	inline std::vector<vec2> multVec2ToVec2List(std::vector<vec2>& vectors, vec2 toMult);
	inline std::vector<vec2> subVec2ToVec2List(std::vector<vec2>& vectors, vec2 toSub);
	inline std::vector<vec2> divVec2ToVec2List(std::vector<vec2>& vectors, vec2 toDiv);
	inline std::vector<vec2> mapVec2List(std::vector<vec2>& vectors, vec2 oldMin, vec2 oldMax, vec2 newMin, vec2 newMax);
	inline vec2 avgVec2List(std::vector<vec2>& vectors);
}


namespace lge
{
	int hash(int state)
	{
		state ^= 2747636419u;
		state *= 2654435769u;
		state ^= state >> 16;
		state *= 2654435769u;
		state ^= state >> 16;
		state *= 2654435769u;
		return state;
	}

	double randomDouble(double min, double max)
	{
		return (min)+((double)rand() / RAND_MAX) * (max - min);  // can change random2D().lenSqr() with rand() / RAND_MAX
	}

	vec2 random2D()
	{
		return vec2(randomDouble(-1, 1), randomDouble(-1, 1)).normalize();//vec2(hash(rand()), hash(rand())).normalize();
	}

	vec2 signRandom2D()
	{
		vec2 r = random2D();
		r.x = r.x < 0 ? r.x * -1 : r.x;
		r.y = r.y < 0 ? r.y * -1 : r.y;
		r.normalize();
		return r;
	}

	vec3 random3D()
	{
		return vec3(hash(rand()), hash(rand()), hash(rand())).normalize();
	}

	vec4 random4D()
	{
		return vec4(hash(rand()), hash(rand()), hash(rand()), hash(rand())).normalize();
	}

	double min(double a, double b)
	{
		return a < b ? a : b;
	}

	double max(double a, double b)
	{
		return a > b ? a : b;
	}

	double clamp(double a, double low, double hi)
	{
		return max(low, min(a, hi));
	}

	vec2 minVec2(vec2 a, vec2 b)
	{
		return vec2(min(a.x, b.x), min(a.y, b.y));
	}

	vec2 maxVec2(vec2 a, vec2 b)
	{
		return vec2(max(a.x, b.x), max(a.y, b.y));
	}

	vec2 clampVec2(vec2 a, vec2 min, vec2 max)
	{
		return maxVec2(min, minVec2(a, max));
	}

	vec2 absVec2(vec2 a)
	{
		return vec2(a.x < 0 ? -a.x : a.x, a.y < 0 ? -a.y : a.y);
	}

	double map(double val, double oldMin, double oldMax, double newMin, double newMax)
	{
		return newMin + (newMax - newMin) * ((val - oldMin) / (oldMax - oldMin));
	}

	double constrain(double val, double min, double max)
	{
		return val < min ? min : val > max ? max : val;
	}

	vec2 constrainVec2(vec2 vec, vec2 min, vec2 max)
	{
		vec.x = constrain(vec.x, min.x, max.x);
		vec.y = constrain(vec.y, min.y, max.y);
		return vec;
	}

	vec2 mapVec2(vec2 a, vec2 oldMin, vec2 oldMax, vec2 newMin, vec2 newMax)
	{
		double x = map(a.x, oldMin.x, oldMax.x, newMin.x, newMax.x);
		double y = map(a.y, oldMin.y, oldMax.y, newMin.y, newMax.y);
		return vec2(x, y);
	}

	double dist(double x, double y, double x2, double y2)
	{
		return sqrt((x2 - x) * (x2 - x) + (y2 - y) * (y2 - y));
	}

	double dist(vec2 a, vec2 b)
	{
		return dist(a.x,a.y,b.x,b.y);
	}

	double distSqr(vec2 a, vec2 b)
	{
		double dx = b.x - a.x;
		double dy = b.y - a.y;


		return (dx * dx) + (dy * dy);
	}

	bool nearlyEqual(double a, double b)
	{
		return abs(a - b) < THRESHOLD;
	}

	bool nearlyEqual(vec2 a, vec2 b)
	{
		return nearlyEqual(a.x,b.x) && nearlyEqual(a.y,b.y);
	}

	std::string generateUUIDv4()
	{
		std::stringstream ss;
		int i;
		ss << std::hex;
		for (i = 0; i < 8; i++) {
			ss << dis(gen);
		}
		ss << "-";
		for (i = 0; i < 4; i++) {
			ss << dis(gen);
		}
		ss << "-4";
		for (i = 0; i < 3; i++) {
			ss << dis(gen);
		}
		ss << "-";
		ss << dis2(gen);
		for (i = 0; i < 3; i++) {
			ss << dis(gen);
		}
		ss << "-";
		for (i = 0; i < 12; i++) {
			ss << dis(gen);
		};
		return ss.str();
	}
	std::vector<vec2> applyMat2ToVec2List(std::vector<vec2>& vectors, mat2 matrix)
	{
		for (unsigned int i = 0; i < vectors.size(); i++)
		{
			vectors[i] = (matrix * vectors[i]);
		}
		return vectors;
	}
	std::vector<vec2> addVec2ToVec2List(std::vector<vec2>& vectors, vec2 toAdd)
	{
		for (unsigned int i = 0; i < vectors.size(); i++)
		{
			vectors[i] += toAdd;
		}
		return vectors;
	}
	std::vector<vec2> multVec2ToVec2List(std::vector<vec2>& vectors, vec2 toMult)
	{
		for (unsigned int i = 0; i < vectors.size(); i++)
		{
			vectors[i] *= toMult;
		}
		return vectors;
	}
	std::vector<vec2> subVec2ToVec2List(std::vector<vec2>& vectors, vec2 toSub)
	{
		for (unsigned int i = 0; i < vectors.size(); i++)
		{
			vectors[i] -= toSub;
		}
		return vectors;
	}
	std::vector<vec2> divVec2ToVec2List(std::vector<vec2>& vectors, vec2 toDiv)
	{
		for (unsigned int i = 0; i < vectors.size(); i++)
		{
			vectors[i] /= toDiv;
		}
		return vectors;
	}
	std::vector<vec2> mapVec2List(std::vector<vec2>& vectors, vec2 oldMin, vec2 oldMax, vec2 newMin, vec2 newMax)
	{
		for (unsigned int i = 0; i < vectors.size(); i++)
		{
			vectors[i] = mapVec2(vectors[i], oldMin, oldMax, newMin, newMax);
		}
		return vectors;
	}
	vec2 avgVec2List(std::vector<vec2>& vectors)
	{
		vec2 sum;
		for (unsigned int i = 0; i < vectors.size(); i++)
			sum += vectors[i];
		return sum / vectors.size();
	}

	lge::PLData lge::distPointToLine(vec2 point, vec2 a, vec2 b)
	{
		PLData data;

		vec2 ab = b - a;
		vec2 ap = point - a;

		double proj = dotVec2(ap, ab);
		double abLenSqr = ab.lenSqr();
		double d = proj / abLenSqr;

		if (d <= 0.0)
		{
			data.closest = a;
		}
		else if (d >= 1.0)
		{
			data.closest = b;
		}
		else
		{
			data.closest = a + (ab * d);
		}

		double dx = point.x - data.closest.x;
		double dy = point.y - data.closest.y;

		data.distSqr = dx * dx + dy * dy;

		//data.distSqr = lge::distSqr(ap, data.closest);

		return data;
	}
}

