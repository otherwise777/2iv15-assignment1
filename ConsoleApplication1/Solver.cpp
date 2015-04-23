#include "Particle.h"
#include <vector>

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)
void simulation_step( std::vector<Particle*> pVector, float dt )
{
	int i, size = pVector.size();
	
	for(i=0; i<size; i++)
	{
		//gravity
		//pVector[i]->m_Position += Vec2f(0.0, -0.000981);
		pVector[i]->m_Position += dt*pVector[i]->m_Velocity;
		pVector[i]->m_Velocity = DAMP*pVector[i]->m_Velocity + Vec2f(RAND,RAND) * 0.005;
	}

}
