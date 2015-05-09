#pragma once

#include "Particle.h"
#include "Force.h"

class LineWireConstraint : public Force
{
public:
	LineWireConstraint(Particle *p, float height);

	void draw();
	void apply();

private:

	Particle * const m_p;
	float m_height;
};
