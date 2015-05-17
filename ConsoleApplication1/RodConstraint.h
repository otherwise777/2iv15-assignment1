#pragma once

#include "Particle.h"
#include "Constraint.h"

class RodConstraint : public Constraint
{
 public:
  RodConstraint(Particle *p1, Particle * p2, double dist);

  void draw();
  float getC();
  float getCDot();
  vector<Vec2f> getJacobian();
  vector<Vec2f> getJacobianDot();
  vector<Particle*> getParticles();

 private:

  Particle * const m_p1;
  Particle * const m_p2;
  double const m_dist;
};
