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

// return the c: C(x, y) = (y - yl)
float LineWireConstraint::getC(){
	float C = (m_p->m_Position[1] - m_height);
	return C;
}

//from the slides C = 1/2(x.x-1) =  (1/2)x.(1/2)x-(1/2)
//from the slides CDot = (1/2)xDot.(1/2)x + (1/2)x.(1/2)xDot
//our C = (y - yc)
//return the C	Dot: (dx/dy)C(x, y) = (y - yc)dx
//CDot = vy
float LineWireConstraint::getCDot(){
	Vec2f veldif = (m_p->m_Velocity);
	float CDot = (veldif[1]);
	return CDot;
}

vector<Vec2f> LineWireConstraint::getJacobian(){
	//a vector, only 1 now, might implement multiple particles
	vector<Vec2f> J;
	J.push_back(Vec2f(0, 1));
	return J;
}

//return JDot, if there are more use same order as particle
vector<Vec2f> LineWireConstraint::getJacobianDot(){
	vector<Vec2f> JDot;
	JDot.push_back(Vec2f(0, 0));
	return JDot;
}

vector<Particle*> LineWireConstraint::getParticles(){
	//only 1 here, but might implement ability for more
	vector<Particle*> result;
	result.push_back(m_p);
	return result;
}