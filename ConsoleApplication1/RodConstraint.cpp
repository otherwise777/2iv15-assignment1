#include "RodConstraint.h"
#include <glut.h>
#include <iostream>
using namespace std;

RodConstraint::RodConstraint(Particle *p1, Particle * p2, double dist) :
  m_p1(p1), m_p2(p2), m_dist(dist) {}

void RodConstraint::draw()
{
  glBegin( GL_LINES );
  glColor3f(0, 1, 0);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0, 1, 0);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();

}

void RodConstraint::apply()
{
	float ks = 0.8;
	float kd = 1;

	Vec2f posdif = (m_p1->m_Position - m_p2->m_Position);
	Vec2f speeddif = (m_p1->m_Velocity - m_p2->m_Velocity);
	float posLength = (sqrt(posdif[0] * posdif[0] + posdif[1] * posdif[1]));

	float C = (posdif[0] * posdif[0] + posdif[1] * posdif[1] - m_dist*m_dist);
	float CDot = (speeddif[0] * speeddif[0] + speeddif[1] * speeddif[1]);

	Vec2f force_p1 = (posdif / posLength)*(ks * C);
	Vec2f force_p2 = -force_p1;

	m_p1->m_Force -= force_p1;
	m_p2->m_Force -= force_p2;
}