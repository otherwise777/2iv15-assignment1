#pragma once

#include "Particle.h"
#include "Constraint.h"

class PointConstraint : public Constraint
{
public:
	PointConstraint(Particle *p, Vec2f height);

	void draw();
	float getC();
	float getCDot();
	vector<Vec2f> getJacobian();
	vector<Vec2f> getJacobianDot();
	vector<Particle*> getParticles();

private:

	Particle * const m_p;
	Vec2f m_Point;
};
