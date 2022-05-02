#include "FunctionTimer.h"

lge::FunctionTimer::FunctionTimer()
{
	std::cout << "test";
}

void lge::FunctionTimer::reset()
{
	m_measurements.clear();
}

void lge::FunctionTimer::start()
{
	m_start = m_timer.now();
}

void lge::FunctionTimer::end()
{
	m_end = m_timer.now();
	std::chrono::duration<double, std::milli> duration = m_end - m_start;
	m_measurements.push_back(duration.count());
}

void lge::FunctionTimer::evaluate()
{
	double avg = 0;
	double min = INFINITY;
	double max = -INFINITY;

	for (int j = 0; j < m_measurements.size(); j++)
	{
		double val = m_measurements[j];
		avg += val;
		if (val < min) min = val;
		if (val > max) max = val;

		std::cout << "Measurement: " << j << " Time: " << val << "ms\n";
	}
	avg /= m_measurements.size();

	std::cout << "Measurements: " << m_measurements.size() << " Avg: " << avg << "ms Min: " << min << "ms Max: " << max << "ms\n";
}

bool lge::FunctionTimer::save(std::string filePath)
{
	std::ofstream outdata;
	outdata.open(filePath);

	if (!outdata) {
		std::cerr << "File could not be opened\n";
		return false;
	}

	double avg = 0;
	double min = INFINITY;
	double max = -INFINITY;
	
	for (int j = 0; j < m_measurements.size(); j++)
	{
		double val = m_measurements[j];
		avg += val;
		if (val < min) min = val;
		if (val > max) max = val;

		outdata << "Measurement: " << j << " Time: " << val << "ms\n";
	}

	avg /= m_measurements.size();

	outdata << "Measurements: " << m_measurements.size() << " Avg: " << avg << "ms Min: " << min << "ms Max: " << max << "ms\n";

	outdata << "Raw Measurements in ms:\n";

	for (int j = 0; j < m_measurements.size(); j++)
	{
		outdata << m_measurements[j] << "\n";
	}

	outdata.close();

	return true;
}
