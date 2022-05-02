#pragma once
#include <chrono>
#include <fstream>
#include <vector>
#include <iostream>

namespace lge
{
	class FunctionTimer
	{
	private:

		std::vector<double> m_measurements;

		std::chrono::high_resolution_clock m_timer;
		
		std::chrono::high_resolution_clock::time_point m_start;
		std::chrono::high_resolution_clock::time_point m_end;

	public:

		FunctionTimer();

		void reset();

		void start();
		void end();

		void evaluate();
		bool save(std::string filePath);

	};
}


