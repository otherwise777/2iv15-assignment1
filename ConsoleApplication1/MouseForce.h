#pragma once

#include "Particle.h"

class MouseForce {
public:
	MouseForce(Particle *p, Vec2f & Mouse, double ks, double kd);

	void getMouse(const Vec2f & Mouse);
	void draw();
	void apply();

private:

	Particle * const m_p;   // particle 1
	Vec2f m_mouse;
	double const m_ks, m_kd; // spring strength constants
};
