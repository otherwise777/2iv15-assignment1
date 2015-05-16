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

	float C = getC();
	float CDot = getCDot();

	Vec2f force_p1 = (posdif / posLength)*(ks * C);
	Vec2f force_p2 = -force_p1;

	m_p1->m_Force -= force_p1;
	m_p2->m_Force -= force_p2;
}

// return the C => C(x1, y1, x2, y2) = (x1 - x2)^2 + (y1 - y2)^2 - r^2
float RodConstraint::getC(){
	//Vec2f posdif = (m_p1->m_Position - m_p2->m_Position);
	//return (posdif[0] * posdif[0] + posdif[1] * posdif[1] - m_dist*m_dist);

	const float& x1 = m_p1->m_Position[0];
	const float& x2 = m_p2->m_Position[0];
	const float& y1 = m_p1->m_Position[1];
	const float& y2 = m_p2->m_Position[1];
	const float& vx1 = m_p1->m_Velocity[0];
	const float& vx2 = m_p2->m_Velocity[0];
	const float& vy1 = m_p1->m_Velocity[1];
	const float& vy2 = m_p2->m_Velocity[1];
	return ((x1 - x2)*(vx1)+(y1 - y2)*(vy1)) - ((x1 - x2)*(vx2)+(y1 - y2)*(vy2));
}

// return the CDot => CDot(x, y , vx, vy) = (x-y)*vx - ((x-y)*vy)
float RodConstraint::getCDot(){
	Vec2f posdif = (m_p1->m_Position - m_p2->m_Position);
	Vec2f veldif = (m_p1->m_Velocity - m_p2->m_Velocity);
	Vec2f DotProduct = posdif[0] * veldif[0] + posdif[1] * veldif[1];
	const float& vx1 = m_p1->m_Velocity[0];
	const float& vx2 = m_p2->m_Velocity[0];
	const float& vy1 = m_p1->m_Velocity[1];
	const float& vy2 = m_p2->m_Velocity[1];
	return ((posdif[0])*(vx1)+(posdif[1])*(vy1)) - ((posdif[0])*(vx2)+(posdif[1])*(vy2));
}
