#include "CircularWireConstraint.h"
#include <glut.h>
#include <iostream>
using namespace std;

#define PI 3.1415926535897932384626433832795

static void draw_circle(const Vec2f & vect, float radius)
{
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0,1.0,0.0); 
	for (int i=0; i<360; i=i+18)
	{
		float degInRad = i*PI/180;
		glVertex2f(vect[0]+cos(degInRad)*radius,vect[1]+sin(degInRad)*radius);
	}
	glEnd();
}

CircularWireConstraint::CircularWireConstraint(Particle *p, const Vec2f & center, const double radius) :
	m_p(p), m_center(center), m_radius(radius) {}

void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);
}

void CircularWireConstraint::apply()
{
	float ks = 2;
	float kd = 1;

	Vec2f posdif = (m_p->m_Position - m_center);
	Vec2f speeddif = (m_p->m_Velocity);
	float posLength = sqrt(posdif[0] * posdif[0] + posdif[1] * posdif[1]);

	float C = (posdif[0] * posdif[0] + posdif[1] * posdif[1] - m_radius * m_radius);
	float CDot = (speeddif[0] * speeddif[0] + speeddif[1] * speeddif[1]);

	Vec2f force_p1 = (posdif / posLength)*(ks * C);

	m_p->m_Velocity -= force_p1;
}
