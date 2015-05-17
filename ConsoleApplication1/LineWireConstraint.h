#pragma once

#include "Particle.h"
#include "Constraint.h"

class LineWireConstraint : public Constraint
{
public:
	LineWireConstraint(Particle *p, float height);

	void draw();
	float getC();
	float getCDot();
	vector<Vec2f> getJacobian();
	vector<Vec2f> getJacobianDot();
	vector<Particle*> getParticles();

private:

	Particle * const m_p;
	float m_height;
};
