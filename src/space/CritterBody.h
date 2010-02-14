/*
 * CritterBody.h
 *
 *  Created on: Jan 26, 2010
 *      Author: seh
 */

#ifndef CRITTERBODY_H_
#define CRITTERBODY_H_

#include <math.h>

#include "DefaultSpace.h"
#include "AbstractBody.h"
#include "../neural/Brain.h"



#define GPS_HARMONICS 4
#define SPACE_FREQ  0.1 //lower requires brains to be more sensitive but gives more dynamic range in the space

class SnakeBody : public AbstractBody {
	Brain* brain;

	float m_fMuscleStrength;

	unsigned parts;
	btVector3 positionOffset;

public:
	SnakeBody(const btVector3& _positionOffset,	unsigned _parts) {
		positionOffset = _positionOffset;
		parts = _parts;
	}

	virtual void init() {

		btVector3 vUp(0, 1, 0);
		m_fMuscleStrength = 0.5f;

		shapes.reserve(parts);
		bodies.reserve(parts);
		joints.reserve(parts-1);

		//
		// Setup geometry
		//
		float fBodySize = 0.15f;
		float fPartLength = 0.45f;
		unsigned i;
		for (i = 0; i < parts; i++) {
			shapes.push_back(new btCapsuleShape(btScalar(fBodySize), btScalar(fPartLength)));
		}

		int numNeurons = 1024;
		int minSynapses = 1;
		int maxSynapses = 16;

		brain = new Brain(GPS_HARMONICS * 3 * parts, parts - 1, numNeurons,
				minSynapses, maxSynapses, 0.5);

		brain->printSummary();

		//
		// Setup rigid bodies
		//
		float fHeight = 0.5;
		btTransform offset;
		offset.setIdentity();
		offset.setOrigin(positionOffset);

		// root
		btVector3 vRoot = btVector3(btScalar(0.), btScalar(fHeight), btScalar(0.));
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(vRoot);
		//m_bodies[0] = localCreateRigidBody(btScalar(1.), offset * transform,m_shapes[0]);

		for (i = 0; i < parts; i++) {
			float fAngle = 2 * M_PI * i / parts;
			float fSin = sin(fAngle);
			float fCos = cos(fAngle);

			transform.setIdentity();
			transform.setOrigin(btVector3(0,i,0));
			//			btVector3 vBoneOrigin = btVector3(btScalar(fCos * (fBodySize + 0.5
			//					* fPartLength)), btScalar(fHeight), btScalar(fSin
			//					* (fBodySize + 0.5 * fPartLength)));
			//			transform.setOrigin(vBoneOrigin);
			//
			//			transform.setIdentity();
			//			transform.setOrigin(btVector3(btScalar(fCos * (fBodySize
			//					+ fPartLength)), btScalar(fHeight - 0.5 * fPartLength),
			//					btScalar(fSin * (fBodySize + fPartLength))));
			bodies[i] = localCreateRigidBody(btScalar(1.),	offset * transform, shapes[i]);

			//			for (unsigned j = 0; j < legParts; j++) {
			//				if (j == 0) {
			//					// thigh
			//					btVector3 vToBone = (vBoneOrigin - vRoot).normalize();
			//					btVector3 vAxis = vToBone.cross(vUp);
			//					transform.setRotation(btQuaternion(vAxis, M_PI_2));
			//					m_bodies[1 + j + legParts * i] = localCreateRigidBody(
			//							btScalar(1.), offset * transform, m_shapes[1 + 2
			//									* i]);
			//				} else if (j == 1) {
			//					// shin
			//					transform.setIdentity();
			//					transform.setOrigin(btVector3(btScalar(fCos * (fBodySize
			//							+ fLegLength)), btScalar(fHeight - 0.5
			//							* fForeLegLength), btScalar(fSin * (fBodySize
			//							+ fLegLength))));
			//					m_bodies[1 + j + legParts * i] = localCreateRigidBody(
			//							btScalar(1.), offset * transform, m_shapes[2 + 2
			//									* i]);
			//				}
			//			}

		}

		// Setup some damping on the m_bodies
		for (i = 0; i < parts; ++i) {
			bodies[i]->setDamping(0.05, 0.85);
			bodies[i]->setDeactivationTime(0.8);
			//m_bodies[i]->setSleepingThresholds(1.6, 2.5);
			bodies[i]->setSleepingThresholds(0.5f, 0.5f);
		}

		//
		// Setup the constraints
		//
		//btGeneric6DoFConstraint* hingeC;
		//btConeTwistConstraint* coneC;

		btTransform localA, localB, localC;

		unsigned joint = 0;
		for (i = 0; i < parts - 1; i++) {
			float fAngle = 2 * M_PI * i / parts;
			float fSin = sin(fAngle);
			float fCos = cos(fAngle);

			//connect legPart[j-1] to legPart[j]

			//						localA.setIdentity(); localB.setIdentity();
			//						localA.getBasis().setEulerZYX(0,-fAngle,0);
			//						localA.setOrigin(btVector3(btScalar(fCos*fBodySize), btScalar(0.), btScalar(fSin*fBodySize)));
			//						localB = m_bodies[i]->getWorldTransform().inverse() * m_bodies[i+1]->getWorldTransform() * localA;
			//						hingeC->setLimit(btScalar(0), btScalar(0.15));

			double headroomFactor = 1.5;

			localA.setIdentity();
			localA.setOrigin(btVector3(0, (fPartLength / 2) * headroomFactor,
					0));
			localB.setIdentity();
			localB.setOrigin(
					btVector3(0, -(fPartLength / 2) * headroomFactor, 0));

			btGeneric6DofConstraint* c = new btGeneric6DofConstraint(
					*bodies[i], *bodies[i + 1], localA, localB, false);

			c->setAngularLowerLimit(btVector3(-M_PI_4, -M_PI_4, -M_PI_4));
			c->setAngularUpperLimit(btVector3(M_PI_4, M_PI_4, M_PI_4));

			joints[joint++] = c;
			dyn->addConstraint(c);

			//			// hip joints
			//			localA.setIdentity(); localB.setIdentity();
			//			localA.getBasis().setEulerZYX(0,-fAngle,0);	localA.setOrigin(btVector3(btScalar(fCos*fBodySize), btScalar(0.), btScalar(fSin*fBodySize)));
			//			localB = m_bodies[1+2*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
			//			hingeC = new btGeneric6DoFConstraint(*m_bodies[0], *m_bodies[1+2*i], localA, localB);
			//			hingeC->setLimit(btScalar(-0.75 * M_PI_4), btScalar(M_PI_8));
			//			//hingeC->setLimit(btScalar(0), btScalar(M_PI));
			//			//hingeC->setLimit(btScalar(-0.1), btScalar(0.1));
			//			m_joints[2*i] = hingeC;
			//			m_ownerWorld->addConstraint(m_joints[2*i], true);
			//
			//			// knee joints
			//			localA.setIdentity(); localB.setIdentity(); localC.setIdentity();
			//			localA.getBasis().setEulerZYX(0,-fAngle,0);	localA.setOrigin(btVector3(btScalar(fCos*(fBodySize+fLegLength)), btScalar(0.), btScalar(fSin*(fBodySize+fLegLength))));
			//			localB = m_bodies[1+2*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
			//			localC = m_bodies[2+2*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
			//			hingeC = new btGeneric6DoFConstraint(*m_bodies[1+2*i], *m_bodies[2+2*i], localB, localC);
			//			hingeC->setLimit(btScalar(0), btScalar(0.15));
			//			//hingeC->setLimit(btScalar(0), btScalar(M_PI));
			//			m_joints[1+2*i] = hingeC;
			//			m_ownerWorld->addConstraint(m_joints[1+2*i], true);
		}

	}

	//sinusoidal gps filter
	float sinGPS(int harmonic, float x) {
		return sin(x * SPACE_FREQ * pow(2, harmonic)) / ((float) harmonic);
	}

	virtual btVector3 getColor(btCollisionShape* shape) {
		float i = (float)(indexOfShape(shape)%2);
		return btVector3(0.3+i*0.5, 0.8+ i*0.2, 0.4);
	}

	virtual void process(btScalar dt) {
		//apply inputs
		unsigned i = 0;
		for (unsigned h = 0; h < GPS_HARMONICS; h++) {
			for (unsigned s = 0; s < parts; s++) {
				btRigidBody* m = bodies[s];
				brain->ins[i++]->setInput(sinGPS(h,
						m->getCenterOfMassPosition().getX()));
				brain->ins[i++]->setInput(sinGPS(h,
						m->getCenterOfMassPosition().getY()));
				brain->ins[i++]->setInput(sinGPS(h,
						m->getCenterOfMassPosition().getZ()));
			}
		}

		//process brain
		brain->forward(1.0);

		btVector3 v;

		//read outputs
		for (unsigned i = 0; i < parts - 1; i++) {
			btGeneric6DofConstraint* c = static_cast<btGeneric6DofConstraint*> (joints[i]);

			double scale = 0.005;
			float xmax = (getLegTargetAngle(i))*scale;
			float xmin = xmax * 0.5;

			//cout << xmin << " " << xmax << "\n";

			c->getTranslationalLimitMotor()->m_lowerLimit.setX(xmin);
			c->getTranslationalLimitMotor()->m_upperLimit.setX(xmax);

			//
			//			btVector3 v(xmin, 0, 0);
			//			c->setLinearLowerLimit(v);
			//
			//			btVector3 w(xmax, 0, 0);
			//			c->setLinearUpperLimit(w);

			//			btScalar fCurAngle = hingeC->getHingeAngle();
			//
			//			btScalar fTargetAngle = (1.0 + getLegTargetAngle(i)) * 2.0; //0.5 * (1 + sin(2 * M_PI * fTargetPercent));
			//
			//			btScalar fTargetLimitAngle = hingeC->getLowerLimit() + fTargetAngle
			//					* (hingeC->getUpperLimit() - hingeC->getLowerLimit());
			//			btScalar fAngleError = fTargetLimitAngle - fCurAngle;
			//			btScalar fDesiredAngularVel = 1000000.f * fAngleError / dt
			//					* 1000000.;
			//			hingeC->enableAngularMotor(true, fDesiredAngularVel,
			//					m_fMuscleStrength);
		}

	}

	btScalar getLegTargetAngle(int part) {
		return brain->outs[part]->getOutput();
	}

	virtual ~SnakeBody() {
		int i;

		// Remove all constraints
		for (i = 0; i < parts - 1; ++i) {
			dyn->removeConstraint(joints[i]);
			delete joints[i];
			joints[i] = 0;
		}

		// Remove all bodies and shapes
		for (i = 0; i < parts; ++i) {
			dyn->removeRigidBody(bodies[i]);

			delete bodies[i]->getMotionState();

			delete bodies[i];
			bodies[i] = 0;
			delete shapes[i];
			shapes[i] = 0;
		}
	}

//	btTypedConstraint** GetJoints() {
//		return &m_joints[0];
//	}

};


class SpiderBody : public AbstractBody {
	Brain* brain;

	float m_fMuscleStrength;

	int NUM_LEGS, PART_COUNT, JOINT_COUNT;

	btVector3 positionOffset;

public:
	SpiderBody (int numLegs, const btVector3& _positionOffset)	{
		NUM_LEGS = numLegs;

		PART_COUNT = 2 * NUM_LEGS + 1;
		JOINT_COUNT = PART_COUNT - 1;
		positionOffset = _positionOffset;
	}

	virtual void init() {
		shapes.reserve(PART_COUNT);
		bodies.reserve(PART_COUNT);
		joints.reserve(JOINT_COUNT);

		btVector3 vUp(0, 1, 0);
		m_fMuscleStrength = 0.9f;

		//
		// Setup geometry
		//
		float fBodySize  = 0.25f;
		float fLegLength = 0.45f;
		float fForeLegLength = 0.85f;
		shapes.push_back( new btCapsuleShape(btScalar(fBodySize), btScalar(0.10)) );
		int i;
		for ( i=0; i<NUM_LEGS; i++)
		{
			shapes.push_back( new btCapsuleShape(btScalar(0.10), btScalar(fLegLength)) );
			shapes.push_back(  new btCapsuleShape(btScalar(0.08), btScalar(fForeLegLength)) );
		}

		int numNeurons = 1024;
		int minSynapses = 1;
		int maxSynapses = 6;


		brain = new Brain(GPS_HARMONICS * 3*(2*NUM_LEGS+1), NUM_LEGS*2, numNeurons, minSynapses, maxSynapses, 0.5);
		brain->printSummary();

		//
		// Setup rigid bodies
		//
		float fHeight = 0.5;
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);

		// root
		btVector3 vRoot = btVector3(btScalar(0.), btScalar(fHeight), btScalar(0.));
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(vRoot);

		bodies.push_back( localCreateRigidBody(btScalar(1.), offset*transform, shapes[0]) );

		// legs
		for ( i=0; i<NUM_LEGS; i++)
		{
			float fAngle = 2 * M_PI * i / NUM_LEGS;
			float fSin = sin(fAngle);
			float fCos = cos(fAngle);

			transform.setIdentity();
			btVector3 vBoneOrigin = btVector3(btScalar(fCos*(fBodySize+0.5*fLegLength)), btScalar(fHeight), btScalar(fSin*(fBodySize+0.5*fLegLength)));
			transform.setOrigin(vBoneOrigin);

			// thigh
			btVector3 vToBone = (vBoneOrigin - vRoot).normalize();
			btVector3 vAxis = vToBone.cross(vUp);
			transform.setRotation(btQuaternion(vAxis, M_PI_2));
			bodies[1+2*i] = localCreateRigidBody(btScalar(1.), offset*transform, shapes[1+2*i]);

			// shin
			transform.setIdentity();
			transform.setOrigin(btVector3(btScalar(fCos*(fBodySize+fLegLength)), btScalar(fHeight-0.5*fForeLegLength), btScalar(fSin*(fBodySize+fLegLength))));
			bodies[2+2*i] = localCreateRigidBody(btScalar(1.), offset*transform, shapes[2+2*i]);
		}

		// Setup some damping on the m_bodies
		for (i = 0; i < PART_COUNT; ++i)
		{
			bodies[i]->setDamping(0.05, 0.85);
			bodies[i]->setDeactivationTime(0.8);
			bodies[i]->setSleepingThresholds(0.5f, 0.5f);
			//m_bodies[i]->setSleepingThresholds(1.6, 2.5);
		}


		//
		// Setup the constraints
		//
		btHingeConstraint* hingeC;
		//btConeTwistConstraint* coneC;

		btTransform localA, localB, localC;

		for ( i=0; i<NUM_LEGS; i++)
		{
			float fAngle = 2 * M_PI * i / NUM_LEGS;
			float fSin = sin(fAngle);
			float fCos = cos(fAngle);

			// hip joints
			localA.setIdentity(); localB.setIdentity();
			localA.getBasis().setEulerZYX(0,-fAngle,0);	localA.setOrigin(btVector3(btScalar(fCos*fBodySize), btScalar(0.), btScalar(fSin*fBodySize)));
			localB = bodies[1+2*i]->getWorldTransform().inverse() * bodies[0]->getWorldTransform() * localA;
			hingeC = new btHingeConstraint(*bodies[0], *bodies[1+2*i], localA, localB);
			hingeC->setLimit(btScalar(-0.75 * M_PI_4), btScalar(M_PI_8));
			//hingeC->setLimit(btScalar(-0.1), btScalar(0.1));
			joints[2*i] = hingeC;
			dyn->addConstraint(joints[2*i], true);

			// knee joints
			localA.setIdentity(); localB.setIdentity(); localC.setIdentity();
			localA.getBasis().setEulerZYX(0,-fAngle,0);	localA.setOrigin(btVector3(btScalar(fCos*(fBodySize+fLegLength)), btScalar(0.), btScalar(fSin*(fBodySize+fLegLength))));
			localB = bodies[1+2*i]->getWorldTransform().inverse() * bodies[0]->getWorldTransform() * localA;
			localC = bodies[2+2*i]->getWorldTransform().inverse() * bodies[0]->getWorldTransform() * localA;
			hingeC = new btHingeConstraint(*bodies[1+2*i], *bodies[2+2*i], localB, localC);
			//hingeC->setLimit(btScalar(-0.01), btScalar(0.01));
			hingeC->setLimit(btScalar(-M_PI_8), btScalar(0.2));
			joints[1+2*i] = hingeC;
			dyn->addConstraint(joints[1+2*i], true);
		}

	}

	virtual btVector3 getColor(btCollisionShape* shape) {
		float i = (float)(indexOfShape(shape)%2);
		return btVector3(0.8+i*0.5, 0.2+ i*0.2, 0.8);
	}

	//sinusoidal gps filter
	float sinGPS(int harmonic, float x) {
		return sin(x*SPACE_FREQ*pow(2, harmonic))/((float)harmonic);
	}

	virtual void process(btScalar dt) {
		//apply inputs
		unsigned i = 0;
		for (unsigned h = 0; h < GPS_HARMONICS; h++) {
			for (unsigned s = 0; s < PART_COUNT; s++) {
				btRigidBody* m = bodies[s];
				brain->ins[i++]->setInput(sinGPS(h, m->getCenterOfMassPosition().getX()));
				brain->ins[i++]->setInput(sinGPS(h, m->getCenterOfMassPosition().getY()));
				brain->ins[i++]->setInput(sinGPS(h, m->getCenterOfMassPosition().getZ()));
			}
		}


		//process brain
		brain->forward(1.0);

		//read outputs
		for (int i=0; i<2*NUM_LEGS; i++) {
			btHingeConstraint* hingeC = static_cast<btHingeConstraint*>(joints[i]);
			btScalar fCurAngle      = hingeC->getHingeAngle();

			//btScalar fTargetPercent = getLegTargetPercent(i, r); //(int(m_Time / 1000) % int(m_fCyclePeriod)) / m_fCyclePeriod;
			btScalar fTargetAngle   = -1.0 + (getLegTargetAngle(i/2, i%2))*M_PI*4.0; //0.5 * (1 + sin(2 * M_PI * fTargetPercent));

			btScalar fTargetLimitAngle = hingeC->getLowerLimit() + fTargetAngle * (hingeC->getUpperLimit() - hingeC->getLowerLimit());
			btScalar fAngleError  = fTargetLimitAngle - fCurAngle;
			btScalar fDesiredAngularVel = 1000000.f * fAngleError/dt*1000000.;
			hingeC->enableAngularMotor(true, fDesiredAngularVel, m_fMuscleStrength);
		}

	}

	btScalar getLegTargetAngle(int leg, bool hipOrKnee) {
		return brain->outs[leg * 2 + hipOrKnee]->getOutput();
	}

	virtual	~SpiderBody()
	{
		int i;

		// Remove all constraints
		for ( i = 0; i < JOINT_COUNT; ++i)
		{
			dyn->removeConstraint(joints[i]);
			delete joints[i]; joints[i] = 0;
		}

		// Remove all bodies and shapes
		for ( i = 0; i < PART_COUNT; ++i)
		{
			dyn->removeRigidBody(bodies[i]);

			delete bodies[i]->getMotionState();

			delete bodies[i]; bodies[i] = 0;
			delete shapes[i]; shapes[i] = 0;
		}
	}

};


#endif /* CRITTERBODY_H_ */
