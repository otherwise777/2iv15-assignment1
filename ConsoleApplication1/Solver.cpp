#include "Particle.h"
#include <vector>
#include <algorithm>
#include "Gravity.h"
#include <iostream>
#include <list>
#include "linearSolver.h"
using namespace std;

void DoConstraint(std::vector<Particle*> pVector, std::vector<Constraint*> constraints);

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)
void simulation_step(std::vector<Particle*> pVector, std::vector<Force*> forces, std::vector<Constraint*> constraints, float dt, int solver)
{
	for (int i = 0; i < pVector.size(); i++) {
		pVector[i]->m_Force[0] = 0;
		pVector[i]->m_Force[1] = 0;
	}
	DoConstraint(pVector, constraints);

	int i, size = pVector.size();
	if (solver == 1)
	{
		//calculate forces
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
		std::vector<Vec2f> posTemp;

		//save original position
		for (i = 0; i < size; i++)
		{
			posTemp.push_back(pVector[i]->m_Position);
		}
		//apply forces for midpoint calculation
		for (int i = 0; i < forces.size(); i++) {
			forces[i]->apply();
		}
		//calculate midpoint position
		for (i = 0; i < size; i++)
		{
			pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
			pVector[i]->m_Position += pVector[i]->m_Velocity * (dt / 2);
		}
		//recalculate the force for the midpoint
		for (int i = 0; i < forces.size(); i++) {
			forces[i]->apply();
		}
		//use midpoint forces on the original position
		for (i = 0; i < size; i++)
		{
			pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
			pVector[i]->m_Position = posTemp[i] + pVector[i]->m_Velocity*dt;
		}
	}
	else if (solver == 3)
	{
		std::vector<Vec2f> posTemp;
		std::vector<Vec2f> k1;
		std::vector<Vec2f> k2;
		std::vector<Vec2f> k3;
		std::vector<Vec2f> k4;

		//save original position and initialize the 4 k's
		for (i = 0; i < size; i++)
		{
			posTemp.push_back(pVector[i]->m_Position);
			k1.push_back(Vec2f(0.0, 0.0));
			k2.push_back(Vec2f(0.0, 0.0));
			k3.push_back(Vec2f(0.0, 0.0));
			k4.push_back(Vec2f(0.0, 0.0));
		}
		//calculate forces
		for (int i = 0; i < forces.size(); i++) {
			forces[i]->apply();
		}
		//calculate midpoint position and save k1
		for (i = 0; i < size; i++)
		{
			pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
			k1[i] = pVector[i]->m_Velocity;
			pVector[i]->m_Position += pVector[i]->m_Velocity * (dt / 2);
		}
		//recalculate the force for the midpoint k2
		for (int i = 0; i < forces.size(); i++) {
			forces[i]->apply();
		}
		//use midpoint forces to calculate k3 and save k2
		for (i = 0; i < size; i++)
		{
			pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
			k2[i] = pVector[i]->m_Velocity;
			pVector[i]->m_Position = posTemp[i] + pVector[i]->m_Velocity * (dt / 2);
		}
		//recalculate the force for the midpoint
		for (int i = 0; i < forces.size(); i++) {
			forces[i]->apply();
		}
		//save k3 and set position for k4
		for (i = 0; i < size; i++)
		{
			pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
			k3[i] = pVector[i]->m_Velocity;
			pVector[i]->m_Position = posTemp[i] + pVector[i]->m_Velocity * dt;
		}
		//recalculate the force for the midpoint
		for (int i = 0; i < forces.size(); i++) {
			forces[i]->apply();
		}
		//save k4 and use it to calculate the actual position
		for (i = 0; i < size; i++)
		{
			pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
			k4[i] = pVector[i]->m_Velocity;
			pVector[i]->m_Position = posTemp[i] + (dt / 6)*(k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
		}
	}
}

static void DoConstraint(std::vector<Particle*> pVector, std::vector<Constraint*> constraints)
{


	int i, size = pVector.size();
	float ks = 1.0;
	float kd = 1.0;
	//the matrices M, which has the size 2n*2n, where n is the number of particles.
	//on the identity is the mass of the particle, so [m1, m1, m2, m2, .... , mn, mn]
	//W is the matrix also with size 2n*2n, where the inverse of the mass is the identity.
	vector<vector<float>> W = vector<vector<float>>(2 * size, vector<float>(2 * size, 0));
	vector<vector<float>> M = vector<vector<float>>(2 * size, vector<float>(2 * size, 0));
	for (int i = 0; i < size; i++)
	{
		M[i * 2 + 0][i * 2 + 0] = pVector[i]->m_mass;
		M[i * 2 + 1][i * 2 + 1] = pVector[i]->m_mass;
		W[i * 2 + 0][i * 2 + 0] = 1 / pVector[i]->m_mass;
		W[i * 2 + 1][i * 2 + 1] = 1 / pVector[i]->m_mass;
	}

	vector<float> C = vector<float>(constraints.size());
	vector<float> CDot = vector<float>(constraints.size());
	for (int i = 0; i < constraints.size(); i++)
	{
		C[i] = constraints[i]->getC();
		CDot[i] = constraints[i]->getCDot();
	}

	//initialize the Jacobian Matrix
	vector<vector<float>> J = vector<vector<float>>(constraints.size(), vector<float>(2 * size));
	for (int i = 0; i < constraints.size(); i++) {
		for (int j = 0; j < 2*size; j++) {
			J[i][j] = 0;
		}
	}
	
	//create the Jacobian Matrix
	for (int i = 0; i < constraints.size(); i++) {
		vector<Vec2f> jac = constraints[i]->getJacobian();
		vector<Particle*> particle = constraints[i]->getParticles();
		for (int j = 0; j < particle.size(); j++) {
			for (int k = 0; k < 2; k++) {
				J[i][particle[j] -> getParticleID() *2 + k] = jac[j][k];
			}
		}
	}

	//Make JDot
	vector<vector<float>> JDot(constraints.size(), vector<float>(2*size));
	for (int i = 0; i < constraints.size(); i++) {
		for (int j = 0; j < 2*size; j++) {
			JDot[i][j] = 0;
		}
	}

	for (int i = 0; i < constraints.size(); i++) {
		vector<Vec2f> jacDot = constraints[i]->getJacobianDot();
		vector<Particle*> particle = constraints[i]->getParticles();
		for (int j = 0; j < particle.size(); j++) {
			for (int k = 0; k < 2; k++) {
				JDot[i][particle[j]->getParticleID() * 2 + k] = jacDot[j][k];
			}
		}
	}

	//Make JT
	vector< vector<float>> JT = vector< vector<float>>((2*size), vector<float>(constraints.size()));
	for (int i = 0; i < constraints.size(); i++) {
		for (int j = 0; j < constraints.size(); j++){
			JT[j][i] = J[i][j];
		}
	}

	vector<vector<float>> JW = vector<vector<float>>(J.size(), vector<float>(W[0].size()));
	JW = VectorMultiplication(J, W);
	vector<vector<float>> JWJT = vector<vector<float>>(JW.size(), vector<float>(JT[0].size()));
	JWJT = VectorMultiplication(JW, JT);

	//Make q
	vector<float>q = vector<float>(2 * size);
	for (unsigned i = 0; i < size; i++) {
		for (int j = 0; j < 2; j++) {
			q[2 * i + j] = pVector[i]->m_Position[j];
		}
	}

	//Make qDot
	vector<float>qDot = vector<float>(2*size);
	for (unsigned i = 0; i < size; i++) {
		for (int j = 0; j < 2; j++) {
			qDot[2*i + j] = pVector[i]->m_Velocity[j];
		}
	}

	//Make Q
	vector<float>Q = vector<float>(2 * size);
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < 2; j++) {
			Q[2*i + j] = pVector[i]->m_Force[j];
		}
	}

	vector<float>  JDotq = vector<float>(qDot.size());
	JDotq = VectorMultiplication(JDot, qDot);
	JDotq = VectorScalarMultiplication(JDotq, -1);

	vector<float> JWQ = vector<float>(Q.size());
	JWQ = VectorMultiplication(JW, Q);

	vector<float> CStrength = vector<float>(constraints.size());
	vector<float> CDotDamping = vector<float>(constraints.size());

	CStrength = VectorScalarMultiplication(C, ks);
	CDotDamping = VectorScalarMultiplication(CDot, kd);

	vector<float> JWJTLambda = vector<float>(constraints.size());
	JWJTLambda = VectorSubtraction(JDotq, JWQ);
	JWJTLambda = VectorSubtraction(JWJTLambda, CStrength);
	JWJTLambda = VectorSubtraction(JWJTLambda, CDotDamping);

	double* JWJTLambdaDouble = new double[JWJTLambda.size()];
	for (int i = 0; i < JWJTLambda.size(); i++)
	{
		JWJTLambdaDouble[i] = JWJTLambda[i];
	}

	//JWJTLambda = -JqDot-JWQ-ksC-kdCDot.
	//we will find Lambda with the ConjGrad function provided.
	//JWJT will be our implicitMatrix, and the results will be in lambda
	implicitMatrix *Mat = new implicitMatrix(&JWJT);
	double* lambda = new double[constraints.size()];
	double* r = new double[constraints.size()];
	int steps = 100;
	//the moment of calculations
	ConjGrad(constraints.size(), Mat, lambda, JWJTLambdaDouble, 1e-30f, &steps );
	
	//Make lambda into a vector float, for multiplication with JT
	vector<float>lambdaVF = vector<float>(constraints.size());
	for (int i = 0; i < constraints.size(); i++){
		lambdaVF[i] = (float)lambda[i];
	}
	//Make Q
	vector<float>QHat = vector<float>(2 * size);
	QHat = VectorMultiplication(JT, lambdaVF);

	//Assign forces
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < 2; j++) {
			pVector[i]->m_Force[j] += QHat[2 * i + j];
		}
	}
}
