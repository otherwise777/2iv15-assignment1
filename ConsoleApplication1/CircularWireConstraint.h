#pragma once

#include "Particle.h"
#include "Force.h"
#include <vector>

class CircularWireConstraint : public Force
{
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const double radius);

  void draw();
  void apply();
  float getC();
  float getCDot();
 private:

  Particle * const m_p;
  Vec2f const m_center;
  double const m_radius;
};
