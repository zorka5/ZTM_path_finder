#pragma once

#include <utility>

class Localization
{
	public:
	double x = 0;
	double y = 0;

public:
	Localization() {};
	Localization(double x_, double y_) : x(x_), y(y_) {};
	void set_localization(double x_, double y_) { x = x_; y = y_; };
};

