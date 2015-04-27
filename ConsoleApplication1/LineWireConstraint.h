#pragma once

#include "Particle.h"

class LineWireConstraint {
public:
	LineWireConstraint(Particle *p, float height);

	void draw();
	void apply();

private:

	Particle * const m_p;
	float m_height;
};
