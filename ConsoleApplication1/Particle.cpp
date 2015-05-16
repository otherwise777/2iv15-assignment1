#include "Particle.h"
#include <glut.h>

Particle::Particle(const Vec2f & ConstructPos, float mass, int ID) :
m_ConstructPos(ConstructPos), m_Position(Vec2f(0.0, 0.0)), m_ID(ID), m_Velocity(Vec2f(0.0, 0.0)), m_mass(mass)
{
}

Particle::~Particle(void)
{
}

int Particle::getParticleID()
{
	return m_ID;
}

void Particle::reset()
{
	m_Position = m_ConstructPos;
	m_Velocity = Vec2f(0.0, 0.0);
}

void Particle::draw()
{
	const double h = (m_mass / 33);
	glColor3f(1.f, 1.f, 1.f); 
	glBegin(GL_QUADS);
	glVertex2f(m_Position[0]-h/2.0, m_Position[1]-h/2.0);
	glVertex2f(m_Position[0]+h/2.0, m_Position[1]-h/2.0);
	glVertex2f(m_Position[0]+h/2.0, m_Position[1]+h/2.0);
	glVertex2f(m_Position[0]-h/2.0, m_Position[1]+h/2.0);
	glEnd();
}