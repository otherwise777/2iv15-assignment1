#pragma once

#include "Particle.h"
#include "Constraint.h"
#include <vector>
using namespace std;

class CircularWireConstraint : public Constraint
{
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const double radius);

  void draw();
  float getC();
  float getCDot();
  vector<Vec2f> getJ();
  vector<Vec2f> getJdot();
  std::vector<Particle> getParticles();
 private:

  Particle * const m_p;
  Vec2f const m_center;
  double const m_radius;
};
