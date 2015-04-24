#include "SpringForce.h"
#include <glut.h>
#include <iostream>
using namespace std;

SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd) :
  m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

void SpringForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
}

void SpringForce::apply()
{
	float distance = 0;
	float distanceX = 0;
	float distanceY = 0;

	distanceX = (m_p1->m_Position[0] - m_p2->m_Position[0]);
	distanceY = (m_p1->m_Position[1] - m_p2->m_Position[1]);
	distance = sqrt(distanceX*distanceX + distanceY*distanceY);
}