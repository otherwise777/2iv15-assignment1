#include "Particle.h"
#include <vector>
#include <algorithm>
#include "Gravity.h"
#include <iostream>
#include <list>
#include "linearSolver.h"
using namespace std;

void CalculateForces(std::vector<Force*> forces, std::vector<Constraint*> constraints, std::vector<Particle*> pVector);

void Euler(std::vector<Force*> forces, std::vector<Constraint*> constraints, std::vector<Particle*> pVector, float dt)
{
	//euler
	CalculateForces(forces, constraints, pVector);
	for (int i = 0; i < pVector.size(); i++)
	{
		pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
		pVector[i]->m_Position += pVector[i]->m_Velocity * dt;
		//pVector[i]->m_Velocity = DAMP*pVector[i]->m_Velocity + Vec2f(RAND, RAND) * 0.005;
	}
}