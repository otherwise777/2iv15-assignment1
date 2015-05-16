#pragma once

#include "Particle.h"
#include "Force.h"
#include <vector>
using namespace std;

class CircularWireConstraint : public Force
{
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const double radius);

  void draw();
  void apply();
  float getC();
  float getCDot();
  vector<Vec2f> getJacobian();
  vector<Vec2f> getJacobianDot();
 private:

  Particle * const m_p;
  Vec2f const m_center;
  double const m_radius;
};
