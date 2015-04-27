#include "LineWireConstraint.h"
#include <glut.h>
#include <iostream>
using namespace std;

#define PI 3.1415926535897932384626433832795


LineWireConstraint::LineWireConstraint(Particle *p, float height) :
m_height(height), m_p(p) {}

void LineWireConstraint::draw()
{
	glBegin(GL_LINES);
	glColor3f(0, 1, 0);
	glVertex2f(-1, m_height);
	glColor3f(0, 1, 0);
	glVertex2f(1, m_height);
	glEnd();
}

void LineWireConstraint::apply()
{
	float m_ks = 2;
	float m_kd = 1;

	float heightdif = (m_p->m_Position[1] - m_height);
	Vec2f posdif = Vec2f(0.0, heightdif);
	//Vec2f posdif = (m_p->m_Position - m_center);
	Vec2f speeddif = (m_p->m_Velocity);
	float posLength = (sqrt(posdif[0] * posdif[0] + posdif[1] * posdif[1]));
	float dotProduct = (speeddif[0] * posdif[0] + speeddif[1] * posdif[1]);

	Vec2f force_p1 = (posdif / posLength)*((m_ks * (posLength)) + (m_kd * (dotProduct / posLength)));

	m_p->m_Velocity -= force_p1;
}
