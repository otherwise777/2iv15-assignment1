#include "CircularWireConstraint.h"
#include <glut.h>
#include <iostream>
using namespace std;
#include "linearSolver.h"

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
	float m_ks = 2;
	float m_kd = 1;

	Vec2f posdif = (m_p->m_Position - m_center);
	Vec2f speeddif = (m_p->m_Velocity);
	float posLength = (sqrt(posdif[0] * posdif[0] + posdif[1] * posdif[1]));
	float dotProduct = (speeddif[0] * posdif[0] + speeddif[1] * posdif[1]);

	float C = getC();
	float CDot = getCDot();
	//matrix W is the matrix with the masses of all the particles on the Identity it's size is 2n*2n
	//vector<vector<int>> matrix = vector<vector<int>>(n, vector<int>(n, 0));

	Vec2f force_p1 = (posdif / posLength)*((m_ks * (posLength - m_radius)) + (m_kd * (dotProduct / posLength)));

	m_p->m_Velocity -= force_p1;
}

// return the c: C(x, y) = (x - xc)^2 + (y - yc)^2 - r^2
float CircularWireConstraint::getC(){
	Vec2f posdif = (m_p->m_Position - m_center);
	return (posdif[0]*posdif[0] + posdif[1]*posdif[1] - m_radius*m_radius);
}

//from the slides C = 1/2(x.x-1) =  (1/2)x.(1/2)x-(1/2)
//from the slides CDot = (1/2)xDot.(1/2)x + (1/2)x.(1/2)xDot
//our C = (x - xc)^2 + (y - yc)^2 - r^2
//return the cDot: (dx/dy)C(x, y) = ((x - xc)^2 + (y - yc)^2 - r^2)dx
//CDot = 2*(x-xc)*vx + 2*(y-yc)*vy - 0
//CDot = 2*((x-xc)*vx + (y-yc)*vy)
float CircularWireConstraint::getCDot(){
	Vec2f xVector = (m_p->m_Position - m_center);
	Vec2f veldif = (m_p->m_Velocity);
	float dotProduct = vecDotNew(2*xVector, 2*veldif);
	return dotProduct;
}

//we only have an x and y for the jacobian, so the jacobian will be a vector array with a single vector
//J = (dC/dx), (dC/dy)
vector<Vec2f> CircularWireConstraint::getJacobian(){
	vector<Vec2f> result;
	Vec2f xVector = (m_p->m_Position - m_center);
	Vec2f veldif = (m_p->m_Velocity);
	result.push_back({ 2 * (xVector[0]*veldif[0]), 2 * (xVector[1]*veldif[1]) });
	return result;
}

vector<Vec2f> CircularWireConstraint::getJacobianDot(){
	vector<Vec2f> result;
	Vec2f veldif = (m_p->m_Velocity);
	result.push_back({ 2 * veldif[0], 2 * veldif[1] });
	return result;
}