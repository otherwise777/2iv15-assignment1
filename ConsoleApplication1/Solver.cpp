#include "Particle.h"
#include <vector>
#include <algorithm>
#include "Gravity.h"
#include <iostream>
#include <list>
using namespace std;

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)
void simulation_step( std::vector<Particle*> pVector, std::vector<Force*> forces, float dt )
{
	int solver = 2;

	int i, size = pVector.size();

	//calculate forces

	if (solver == 1)
	{
		for (int i = 0; i < forces.size(); i++) {
			forces[i]->apply();
		}
		//F= m*a
		//a = F/m
		//V= a*t
		//V= (F*t)/m
		//S= V*t
		for (i = 0; i < size; i++)
		{
			pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
			pVector[i]->m_Position += pVector[i]->m_Velocity * dt;
			//pVector[i]->m_Velocity = DAMP*pVector[i]->m_Velocity + Vec2f(RAND, RAND) * 0.005;
		}
	}
	else if (solver == 2)
	{
		static std::vector<Particle*> pVectorCopy1;
		std::vector<Vec2f> posTemp;
		pVectorCopy1 = pVector;
		for (i = 0; i < size; i++)
		{
			posTemp.push_back(pVector[i]->m_Position);
		}

		for (int i = 0; i < forces.size(); i++) {
			forces[i]->apply();
		}


		for (i = 0; i < size; i++)
		{
			pVectorCopy1[i]->m_Velocity += ((pVectorCopy1[i]->m_Force * dt) / pVectorCopy1[i]->m_mass);
			pVectorCopy1[i]->m_Position += pVectorCopy1[i]->m_Velocity * (dt / 2);
		}
		for (int i = 0; i < forces.size(); i++) {
			forces[i]->apply();
		}
		for (i = 0; i < size; i++)
		{
			pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
			pVector[i]->m_Position = posTemp[i] + pVector[i]->m_Velocity*dt;
		}
	}
	else if (solver == 3)
	{
		static std::vector<Particle*> pVectorCopy1;
		static std::vector<Particle*> pVectorCopy2;
		static std::vector<Particle*> pVectorCopy3;
		static std::vector<Particle*> pVectorCopy4;
		pVectorCopy1 = pVector;
		pVectorCopy2 = pVector;
		pVectorCopy3 = pVector;
		pVectorCopy4 = pVector;

		for (i = 0; i < size; i++)
		{
			pVectorCopy2[i]->m_Velocity += ((pVectorCopy2[i]->m_Force * dt) / pVectorCopy2[i]->m_mass);
			pVectorCopy2[i]->m_Position += pVectorCopy2[i]->m_Velocity * (dt / 2);
		}

		pVector = pVectorCopy1;
	}
}

