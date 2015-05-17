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
	for (int i = 0; i < pVector.size(); i++) 
	{
		pVector[i]->m_Force[0] = 0;
		pVector[i]->m_Force[1] = 0;
	}

	int i, size = pVector.size();
	if (solver == 1)
	{
		//calculate forces
		for (int i = 0; i < forces.size(); i++) {
			forces[i]->apply();
		}
		DoConstraint(pVector, constraints);
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
		DoConstraint(pVector, constraints);
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
		DoConstraint(pVector, constraints);
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
	if (constraints.size() == 0 || pVector.size() == 0){
		return;
	}

	int n = 2;
	float ks = 30.0f;
	float kd = 2.0f;

	vector<vector<float>> W = vector<vector<float>>(2 * pVector.size(), vector<float>(2 * pVector.size(), 0));
	vector<vector<float>> M = vector<vector<float>>(2 * pVector.size(), vector<float>(2 * pVector.size(), 0));
	vector<float>q = vector<float>(pVector.size()*n);
	vector<float>qDot = vector<float>(pVector.size()*n);
	vector<float>Q = vector<float>(pVector.size()*n);
	for (int i = 0; i < pVector.size(); i++)
	{
		M[i * 2 + 0][i * 2 + 0] = pVector[i]->m_mass;
		M[i * 2 + 1][i * 2 + 1] = pVector[i]->m_mass;
		W[i * 2 + 0][i * 2 + 0] = 1 / pVector[i]->m_mass;
		W[i * 2 + 1][i * 2 + 1] = 1 / pVector[i]->m_mass;
		q[i * 2 + 0] = pVector[i]->m_Position[0];
		q[i * 2 + 1] = pVector[i]->m_Position[1];
		qDot[i * 2 + 0] = pVector[i]->m_Velocity[0];
		qDot[i * 2 + 1] = pVector[i]->m_Velocity[1];
		Q[i * 2 + 0] = pVector[i]->m_Force[0];
		Q[i * 2 + 1] = pVector[i]->m_Force[1];
	}

	//Make C
	vector<float>C = vector<float>(constraints.size());
	vector<float>CDot = vector<float>(constraints.size());
	for (int i = 0; i < constraints.size(); i++) {
		C[i] = constraints[i]->getC();
		CDot[i] = constraints[i]->getCDot();
	}

	vector<vector<float>> J = vector<vector<float>>(constraints.size(), vector<float>(pVector.size()*n));
	vector<vector<float>> JDot = vector<vector<float>>(constraints.size(), vector<float>(pVector.size()*n));
	vector<vector<float>> JT = vector<vector<float>>((pVector.size()*n), vector<float>(constraints.size()));
	for (int i = 0; i < constraints.size(); i++) {
		vector<Vec2f> jac = constraints[i]->getJacobian();
		vector<Vec2f> jacDot = constraints[i]->getJacobianDot();
		vector<Particle*> particles = constraints[i]->getParticles();
		for (int j = 0; j < particles.size(); j++) {
			for (int k = 0; k < n; k++) {
				J[i][particles[j]->getParticleID()*n + k] = jac[j][k];
				JT[particles[j]->getParticleID()*n + k][i] = jac[j][k];
				JDot[i][particles[j]->getParticleID()*n + k] = jacDot[j][k];
			}
		}
	}

	vector<vector<float>>JW = vector<vector<float>>(constraints.size(), vector<float>(pVector.size()*n));
	vector<vector<float>>JWJT = vector<vector<float>>(constraints.size(), vector<float>(pVector.size()*n));
	JW = VectorMultiplication(J, W);
	JWJT = VectorMultiplication(JW, JT);

	vector<float>  JDotqDot = vector<float>(qDot.size());
	vector<float> JWQ = vector<float>(Q.size());
	JDotqDot = VectorMultiplication(JDot, qDot);
	JDotqDot = VectorScalarMultiplication(JDotqDot, -1);
	JWQ = VectorMultiplication(JW, Q);

	vector<float> CStrength = vector<float>(constraints.size());
	vector<float> CDotDamping = vector<float>(constraints.size());
	CStrength = VectorScalarMultiplication(C, ks);
	CDotDamping = VectorScalarMultiplication(CDot, kd);

	vector<float> JWJTLambda = vector<float>(constraints.size());
	JWJTLambda = VectorSubtraction(JDotqDot, JWQ);
	JWJTLambda = VectorSubtraction(JWJTLambda, CStrength);
	JWJTLambda = VectorSubtraction(JWJTLambda, CDotDamping);

	double* JWJTLambdaDouble = new double[JWJTLambda.size()];
	for (int i = 0; i < JWJTLambda.size(); i++)
	{
		JWJTLambdaDouble[i] = JWJTLambda[i];
	}

	implicitMatrix *ImJWJT = new implicitMatrix(&JWJT);
	double* lambda = new double[constraints.size()];

	int d = 100;
	ConjGrad(constraints.size(), ImJWJT, lambda, JWJTLambdaDouble, 1e-30f, &d);

	vector<float>lambdaFloat = vector<float>(constraints.size());
	for (unsigned i = 0; i < constraints.size(); i++){
		lambdaFloat[i] = lambda[i];
	}

	vector<float>Qhat = vector<float>(constraints.size());
	Qhat = VectorMultiplication(JT, lambdaFloat);

	for (int i = 0; i < pVector.size(); i++) {
		for (int j = 0; j < n; j++) {
			pVector[i]->m_Force[j] += Qhat[2 * i + j];
		}
	}
}
