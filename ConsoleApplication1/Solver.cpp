#include "Particle.h"
#include <vector>
#include <algorithm>
#include "Gravity.h"

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)
void simulation_step( std::vector<Particle*> pVector, std::vector<Force*> forces, float dt )
{
	int i, size = pVector.size();
	
	for (int i = 0; i < forces.size(); i++) {
		forces[i]->apply();
	}

	//F= m*a
	//a = F/m
	//V= a*t
	//V= (F*t)/m
	//S= V*t
	for (int i = 0; i < pVector.size(); i++) {
		pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt)/pVector[i]->m_mass);
	}

	for(i=0; i<size; i++)
	{
		pVector[i]->m_Position += pVector[i]->m_Velocity * dt;
		pVector[i]->m_Velocity = DAMP*pVector[i]->m_Velocity + Vec2f(RAND,RAND) * 0.005;
	}

}

