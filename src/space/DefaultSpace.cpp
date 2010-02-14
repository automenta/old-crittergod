/*
 Bullet Continuous Collision Detection and Physics Library Copyright (c) 2007 Erwin Coumans
 Motor Demo

 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:

 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

#include <iostream>

#include <FTGL/ftgl.h>
#include "../video/font/FontDemo1.h"

#include "../neural/Brain.h"

#include "../physics/btBulletDynamicsCommon.h"
#include "../physics/OpenGL/GlutStuff.h"
#include "../physics/OpenGL/GL_ShapeDrawer.h"

#include "../physics/LinearMath/btIDebugDraw.h"

#include "../physics/OpenGL/GLDebugDrawer.h"
#include "DefaultSpace.h"
#include <math.h>

#include "AbstractBody.h"

// LOCAL FUNCTIONS

void vertex(btVector3 &v) {
	glVertex3d(v.getX(), v.getY(), v.getZ());
}

void drawFrame(btTransform &tr) {
	const float fSize = 1.f;

	glBegin(GL_LINES);

	// x
	glColor3f(255.f, 0, 0);
	btVector3 vX = tr * btVector3(fSize, 0, 0);
	vertex(tr.getOrigin());
	vertex(vX);

	// y
	glColor3f(0, 255.f, 0);
	btVector3 vY = tr * btVector3(0, fSize, 0);
	vertex(tr.getOrigin());
	vertex(vY);

	// z
	glColor3f(0, 0, 255.f);
	btVector3 vZ = tr * btVector3(0, 0, fSize);
	vertex(tr.getOrigin());
	vertex(vZ);

	glEnd();
}

// /LOCAL FUNCTIONS


//#define NUM_LEGS 6
//#define BODYPART_COUNT 2 * NUM_LEGS + 1
//#define JOINT_COUNT BODYPART_COUNT - 1


//class SpiderBody {
//	Brain* brain;
//
//	unsigned legs, legParts;
//	unsigned numParts;
//	unsigned numJoints;
//
//	btDynamicsWorld*	m_ownerWorld;
//	btCollisionShape**	m_shapes; //[BODYPART_COUNT];
//	btRigidBody**		m_bodies; //[BODYPART_COUNT];
//	btTypedConstraint**	m_joints; //[JOINT_COUNT];
//	float m_fMuscleStrength;
//
//	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)	{
//		bool isDynamic = (mass != 0.f);
//
//		btVector3 localInertia(0,0,0);
//		if (isDynamic)
//			shape->calculateLocalInertia(mass,localInertia);
//
//		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
//		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
//		btRigidBody* body = new btRigidBody(rbInfo);
//
//		m_ownerWorld->addRigidBody(body);
//
//		return body;
//	}
//
//
//public:
//	SpiderBody (btDynamicsWorld* ownerWorld, const btVector3& positionOffset, unsigned _legs, unsigned _legParts) : m_ownerWorld (ownerWorld)	{
//		legs = _legs;
//		legParts = _legParts;
//
//		numParts = 1 + legParts * legs;
//		numJoints = numParts - 1;
//
//		m_shapes = new btCollisionShape*[numParts];
//		m_bodies = new btRigidBody*[numParts];
//		m_joints = new btTypedConstraint*[numJoints];
//
//
//		btVector3 vUp(0, 1, 0);
//		m_fMuscleStrength = 1.9f;
//
//		//
//		// Setup geometry
//		//
//		float fBodySize  = 0.25f;
//		float fLegLength = 0.45f;
//		float fForeLegLength = 0.85f;
//		m_shapes[0] = new btCapsuleShape(btScalar(fBodySize), btScalar(0.10));
//		unsigned i;
//		for ( i=0; i< legs; i++)
//		{
//			m_shapes[1 + 2*i] = new btCapsuleShape(btScalar(0.10), btScalar(fLegLength));
//			m_shapes[2 + 2*i] = new btCapsuleShape(btScalar(0.08), btScalar(fForeLegLength));
//		}
//
//		int numNeurons = 1024;
//		int minSynapses = 1;
//		int maxSynapses = 4;
//
//
//		brain = new Brain(GPS_HARMONICS * 3*(legParts*legs+1), legParts*legs, numNeurons, minSynapses, maxSynapses);
//		brain->printSummary();
//
//		//
//		// Setup rigid bodies
//		//
//		float fHeight = 0.5;
//		btTransform offset; offset.setIdentity();
//		offset.setOrigin(positionOffset);
//
//		// root
//		btVector3 vRoot = btVector3(btScalar(0.), btScalar(fHeight), btScalar(0.));
//		btTransform transform;
//		transform.setIdentity();
//		transform.setOrigin(vRoot);
//		m_bodies[0] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[0]);
//
//		// legs
//		for ( i=0; i<legs; i++)		{
//			float fAngle = 2 * M_PI * i / legs;
//			float fSin = sin(fAngle);
//			float fCos = cos(fAngle);
//
//			transform.setIdentity();
//			btVector3 vBoneOrigin = btVector3(btScalar(fCos*(fBodySize+0.5*fLegLength)), btScalar(fHeight), btScalar(fSin*(fBodySize+0.5*fLegLength)));
//			transform.setOrigin(vBoneOrigin);
//
//			for (unsigned j = 0; j < legParts; j++) {
//				if (j == 0) {
//					// thigh
//					btVector3 vToBone = (vBoneOrigin - vRoot).normalize();
//					btVector3 vAxis = vToBone.cross(vUp);
//					transform.setRotation(btQuaternion(vAxis, M_PI_2));
//					m_bodies[1 + j+legParts*i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[1+2*i]);
//				}
//				else if (j == 1) {
//					// shin
//					transform.setIdentity();
//					transform.setOrigin(btVector3(btScalar(fCos*(fBodySize+fLegLength)), btScalar(fHeight-0.5*fForeLegLength), btScalar(fSin*(fBodySize+fLegLength))));
//					m_bodies[1 + j+legParts*i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[2+2*i]);
//				}
//			}
//
//		}
//
//		// Setup some damping on the m_bodies
//		for (i = 0; i < numParts; ++i)		{
//			m_bodies[i]->setDamping(0.05, 0.85);
//			m_bodies[i]->setDeactivationTime(0.8);
//			//m_bodies[i]->setSleepingThresholds(1.6, 2.5);
//			m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);
//		}
//
//
//		//
//		// Setup the constraints
//		//
//		btGeneric6DoFConstraint* hingeC;
//		//btConeTwistConstraint* coneC;
//
//		btTransform localA, localB, localC;
//
//		unsigned joint = 0;
//		for ( i=0; i< legs; i++)		{
//			float fAngle = 2 * M_PI * i / legs;
//			float fSin = sin(fAngle);
//			float fCos = cos(fAngle);
//
//			if (legParts > 1) {
//				for (unsigned j = 1; j < legParts; j++) {
//					//connect legPart[j-1] to legPart[j]
//
//					btRigidBody* a = (j == 0) ? *m_bodies[0] :  *m_bodies[1+j+legParts*i];
//					btRigidBody* b = *m_bodies[2+j+legParts*i];
//					hingeC = new btGeneric6DoFConstraint(a, *m_bodies[1+2*i], localA, localB);
//
//					m_joints[joint] = hingeC;
//					m_ownerWorld->addConstraint(hingeC);
//				}
//			}
//
////			// hip joints
////			localA.setIdentity(); localB.setIdentity();
////			localA.getBasis().setEulerZYX(0,-fAngle,0);	localA.setOrigin(btVector3(btScalar(fCos*fBodySize), btScalar(0.), btScalar(fSin*fBodySize)));
////			localB = m_bodies[1+2*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
////			hingeC = new btGeneric6DoFConstraint(*m_bodies[0], *m_bodies[1+2*i], localA, localB);
////			hingeC->setLimit(btScalar(-0.75 * M_PI_4), btScalar(M_PI_8));
////			//hingeC->setLimit(btScalar(0), btScalar(M_PI));
////			//hingeC->setLimit(btScalar(-0.1), btScalar(0.1));
////			m_joints[2*i] = hingeC;
////			m_ownerWorld->addConstraint(m_joints[2*i], true);
////
////			// knee joints
////			localA.setIdentity(); localB.setIdentity(); localC.setIdentity();
////			localA.getBasis().setEulerZYX(0,-fAngle,0);	localA.setOrigin(btVector3(btScalar(fCos*(fBodySize+fLegLength)), btScalar(0.), btScalar(fSin*(fBodySize+fLegLength))));
////			localB = m_bodies[1+2*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
////			localC = m_bodies[2+2*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
////			hingeC = new btGeneric6DoFConstraint(*m_bodies[1+2*i], *m_bodies[2+2*i], localB, localC);
////			hingeC->setLimit(btScalar(0), btScalar(0.15));
////			//hingeC->setLimit(btScalar(0), btScalar(M_PI));
////			m_joints[1+2*i] = hingeC;
////			m_ownerWorld->addConstraint(m_joints[1+2*i], true);
//		}
//
//	}
//
//	//sinusoidal gps filter
//	float sinGPS(int harmonic, float x) {
//		return sin(x*SPACE_FREQ*pow(2, harmonic))/((float)harmonic);
//	}
//
//	void process(btScalar dt) {
//		//apply inputs
//		unsigned i = 0;
//		for (unsigned h = 0; h < GPS_HARMONICS; h++) {
//			for (unsigned s = 0; s < numParts; s++) {
//				btRigidBody* m = m_bodies[s];
//				brain->ins[i++]->setInput(sinGPS(h, m->getCenterOfMassPosition().getX()));
//				brain->ins[i++]->setInput(sinGPS(h, m->getCenterOfMassPosition().getY()));
//				brain->ins[i++]->setInput(sinGPS(h, m->getCenterOfMassPosition().getZ()));
//			}
//		}
//
//
//		//process brain
//		brain->forward(1.0);
//
//		//read outputs
//		for (int i=0; i<legParts*legs; i++) {
//			btGeneric6DoFConstraint* hingeC = static_cast<btGeneric6DoFConstraint*>(GetJoints()[i]);
//			btScalar fCurAngle      = hingeC->getHingeAngle();
//
//			btScalar fTargetAngle   = (1.0 + getLegTargetAngle(i/2, i%2))*2.0; //0.5 * (1 + sin(2 * M_PI * fTargetPercent));
//
//			btScalar fTargetLimitAngle = hingeC->getLowerLimit() + fTargetAngle * (hingeC->getUpperLimit() - hingeC->getLowerLimit());
//			btScalar fAngleError  = fTargetLimitAngle - fCurAngle;
//			btScalar fDesiredAngularVel = 1000000.f * fAngleError/dt*1000000.;
//			hingeC->enableAngularMotor(true, fDesiredAngularVel, m_fMuscleStrength);
//		}
//
//	}
//
//	btScalar getLegTargetAngle(int leg, bool hipOrKnee) {
//		return brain->outs[leg * legParts + hipOrKnee]->getOutput();
//	}
//
//	virtual	~SpiderBody ()
//	{
//		int i;
//
//		// Remove all constraints
//		for ( i = 0; i < numJoints; ++i)
//		{
//			m_ownerWorld->removeConstraint(m_joints[i]);
//			delete m_joints[i]; m_joints[i] = 0;
//		}
//
//		// Remove all bodies and shapes
//		for ( i = 0; i < numParts; ++i)
//		{
//			m_ownerWorld->removeRigidBody(m_bodies[i]);
//
//			delete m_bodies[i]->getMotionState();
//
//			delete m_bodies[i]; m_bodies[i] = 0;
//			delete m_shapes[i]; m_shapes[i] = 0;
//		}
//	}
//
//	btTypedConstraint** GetJoints() {return &m_joints[0];}
//
//};


void DefaultSpace::mouseFunc(int button, int state, int x, int y) {

	if (m_dialogDynamicsWorld != NULL) {
		if (!m_dialogDynamicsWorld->mouseFunc(button, state, x, y)) {
			SpaceProcess::mouseFunc(button, state, x, y);
		}
	} else {
		SpaceProcess::mouseFunc(button, state, x, y);
	}
}

void DefaultSpace::mouseMotionFunc(int x, int y) {
	if (m_dialogDynamicsWorld != NULL)
		m_dialogDynamicsWorld->mouseMotionFunc(x, y);

	SpaceProcess::mouseMotionFunc(x, y);
}

void motorPreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
	DefaultSpace* motorDemo = (DefaultSpace*) world->getWorldUserInfo();

	motorDemo->setMotorTargets(timeStep);

}

void DefaultSpace::reshape(int w, int h) {
	if (m_dialogDynamicsWorld)
		m_dialogDynamicsWorld->setScreenSize(w, h);
	AbstractSpace::reshape(w, h);
}

void DefaultSpace::initPhysics() {

	setTexturing(true);
	setShadows(true);

	// Setup the basic world

	m_Time = 0;
	m_fCyclePeriod = 2000.f; // in milliseconds

	//	m_fMuscleStrength = 0.05f;
	// new SIMD solver for joints clips accumulated impulse, so the new limits for the motor
	// should be (numberOfsolverIterations * oldLimits)
	// currently solver uses 10 iterations, so:

	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-10000, -10000, -10000);
	btVector3 worldAabbMax(10000, 10000, 10000);
	m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax);

	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase,
			m_solver, m_collisionConfiguration);

	m_dynamicsWorld->setInternalTickCallback(motorPreTickCallback, this, true);

	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(
				btScalar(200.), btScalar(10.), btScalar(200.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -10, 0));
		localCreateRigidBody(btScalar(0.), groundTransform, groundShape);
	}


	//	startOffset.setValue(-2, 0.5, 0);
	//	newSpider(startOffset, 1, 6);

	//init gui
	m_dialogDynamicsWorld = NULL;
	//	{
	//		m_dialogDynamicsWorld = new GL_DialogDynamicsWorld();
	//
	//		//m_dialogDynamicsWorld->createDialog(100,110,200,50);
	//		//m_dialogDynamicsWorld->createDialog(100,00,100,100);
	//		//m_dialogDynamicsWorld->createDialog(0,0,100,100);
	//		GL_DialogWindow* settings = m_dialogDynamicsWorld->createDialog(50, 0,
	//				200, 120, "Settings");
	//		GL_ToggleControl* toggle = m_dialogDynamicsWorld->createToggle(
	//				settings, "Toggle 1");
	//		toggle = m_dialogDynamicsWorld->createToggle(settings, "Toggle 2");
	//		toggle ->m_active = true;
	//		toggle = m_dialogDynamicsWorld->createToggle(settings, "Toggle 3");
	//		GL_SliderControl* slider = m_dialogDynamicsWorld->createSlider(
	//				settings, "Slider");
	//
	//		GL_DialogWindow* dialog = m_dialogDynamicsWorld->createDialog(0, 200,
	//				420, 300, "Help");
	//
	//		GL_TextControl* txt = new GL_TextControl;
	//		dialog->addControl(txt);
	//		txt->m_textLines.push_back("Mouse to move");
	//		txt->m_textLines.push_back("Test 2");
	//		txt->m_textLines.push_back("mouse to interact");
	//		txt->m_textLines.push_back("ALT + mouse to move camera");
	//		txt->m_textLines.push_back("space to reset");
	//		txt->m_textLines.push_back("cursor keys and z,x to navigate");
	//		txt->m_textLines.push_back("i to toggle simulation, s single step");
	//		txt->m_textLines.push_back("q to quit");
	//		txt->m_textLines.push_back(". to shoot box");
	//		txt->m_textLines.push_back("d to toggle deactivation");
	//		txt->m_textLines.push_back("g to toggle mesh animation (ConcaveDemo)");
	//		txt->m_textLines.push_back("h to toggle help text");
	//		txt->m_textLines.push_back("o to toggle orthogonal/perspective view");
	//		//txt->m_textLines.push_back("+- shooting speed = %10.2f",m_ShootBoxInitialSpeed);
	//
	//
	//	}

	clientResetScene();
}

void DefaultSpace::addBody(AbstractBody* a) {
	a->init(m_dynamicsWorld);
	bodies.push_back(a);
}
void DefaultSpace::removeBody(AbstractBody* a) {
	bodies.remove(a);
}


void PreStep() {

}

void DefaultSpace::setMotorTargets(btScalar deltaTime) {

	float ms = deltaTime * 1000000.;
	float minFPS = 1000000.f / 60.f;
	if (ms > minFPS)
		ms = minFPS;

	m_Time += ms;

	//
	// set per-frame sinusoidal position targets using angular motor (hacky?)
	//
	for (int r = 0; r < bodies.size(); r++) {
		AbstractBody* t = bodies[r];
		t->process(deltaTime);
	}

}

void DefaultSpace::clientMoveAndDisplay() {
	static FTFont *font = NULL;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//simple dynamics world doesn't handle fixed-time-stepping
	float deltaTime = getDeltaTimeMicroseconds() / 1000000.f;

	if (m_dynamicsWorld) {
		m_dynamicsWorld->stepSimulation(deltaTime);
		m_dynamicsWorld->debugDrawWorld();
	}
	//draw font
	{
		int now = glutGet(GLUT_ELAPSED_TIME);

		float n = (float) now / 20.;
		float t1 = sin(n / 80);
		float t2 = sin(n / 50 + 1);
		float t3 = sin(n / 30 + 2);

		float ambient[4] = { (t1 + 2.0) / 3, (t2 + 2.0) / 3, (t3 + 2.0) / 3,
				0.3 };
		float diffuse[4] = { 1.0, 0.9, 0.9, 1.0 };
		float specular[4] = { 1.0, 0.7, 0.7, 1.0 };
		float position[4] = { 100.0, 100.0, 0.0, 1.0 };

		float front_ambient[4] = { 0.7, 0.7, 0.7, 0.0 };

		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glEnable(GL_LIGHTING);
		glEnable(GL_DEPTH_TEST);

		glPushMatrix();
		glTranslatef(-0.9, -0.2, -10.0);
		glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
		glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
		glLightfv(GL_LIGHT1, GL_SPECULAR, specular);
		glLightfv(GL_LIGHT1, GL_POSITION, position);
		glEnable(GL_LIGHT1);
		glPopMatrix();

		if (font == NULL) {
			font = new FTExtrudeFont("media/font/DejaVuSans.ttf");
			font->FaceSize(1);
			font->CharMap(ft_encoding_unicode);
		}

		glPushMatrix();
		glMaterialfv(GL_FRONT, GL_AMBIENT, front_ambient);
		glColorMaterial(GL_FRONT, GL_DIFFUSE);
		glTranslatef(5.0, 5.0, 20.0);
		glRotatef(n / 1.11, 0.0, 1.0, 0.0);
		glRotatef(0, 1.0, 0.0, 0.0);
		glRotatef(0, 0.0, 0.0, 1.0);
		glColor3f(1.0, 1.0, 1.0);
		//drawText3D(font[fontindex], "SpaceGraph", 0, 0, 20.0);

		font->Render("SpaceGraph");
		glPopMatrix();

	}

	renderme();

	for (int i = 2; i >= 0; i--) {
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		drawFrame(body->getWorldTransform());
	}

	float ms = getDeltaTimeMicroseconds();
	if (m_dialogDynamicsWorld)
		m_dialogDynamicsWorld->draw(ms / 1000000.f);

	glFlush();
	glutSwapBuffers();
}

void DefaultSpace::displayCallback() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	float ms = getDeltaTimeMicroseconds();
	if (m_dialogDynamicsWorld != NULL)
		m_dialogDynamicsWorld->draw(ms / 1000000.f);

	glFlush();
	glutSwapBuffers();
}

void DefaultSpace::keyboardCallback(unsigned char key, int x, int y) {
	switch (key) {
	case '+':
	case '=':
		m_fCyclePeriod /= 1.1f;
		if (m_fCyclePeriod < 1.f)
			m_fCyclePeriod = 1.f;
		break;
	case '-':
	case '_':
		m_fCyclePeriod *= 1.1f;
		break;
		//	case '[':
		//		m_fMuscleStrength /= 1.1f;
		//		break;
		//	case ']':
		//		m_fMuscleStrength *= 1.1f;
		//		break;
	default:
		SpaceProcess::keyboardCallback(key, x, y);
	}
}

void DefaultSpace::exitPhysics() {

	int i;

	for (i = 0; i < bodies.size(); i++) {
		AbstractBody* rig = bodies[i];
		delete rig;
	}

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them

	for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState()) {
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++) {
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;
}

void DefaultSpace::renderme() {
	myinit();

	updateCamera();

	if (m_dynamicsWorld) {
		if (m_enableshadows) {
			glClear(GL_STENCIL_BUFFER_BIT);
			glEnable(GL_CULL_FACE);
			renderscene(0);

			glDisable(GL_LIGHTING);
			glDepthMask(GL_FALSE);
			glDepthFunc(GL_LEQUAL);
			glEnable(GL_STENCIL_TEST);
			glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
			glStencilFunc(GL_ALWAYS, 1, 0xFFFFFFFFL);
			glFrontFace(GL_CCW);
			glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
			renderscene(1);
			glFrontFace(GL_CW);
			glStencilOp(GL_KEEP, GL_KEEP, GL_DECR);
			renderscene(1);
			glFrontFace(GL_CCW);

			glPolygonMode(GL_FRONT, GL_FILL);
			glPolygonMode(GL_BACK, GL_FILL);
			glShadeModel(GL_SMOOTH);
			glEnable(GL_DEPTH_TEST);
			glDepthFunc(GL_LESS);
			glEnable(GL_LIGHTING);
			glDepthMask(GL_TRUE);
			glCullFace(GL_BACK);
			glFrontFace(GL_CCW);
			glEnable(GL_CULL_FACE);
			glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

			glDepthFunc(GL_LEQUAL);
			glStencilFunc(GL_NOTEQUAL, 0, 0xFFFFFFFFL);
			glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
			glDisable(GL_LIGHTING);
			renderscene(2);
			glEnable(GL_LIGHTING);
			glDepthFunc(GL_LESS);
			glDisable(GL_STENCIL_TEST);
			glDisable(GL_CULL_FACE);
		} else {
			glDisable(GL_CULL_FACE);
			renderscene(0);
		}

		int xOffset = 10;
		int yStart = 20;
		int yIncr = 20;
		char buf[124];

		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);

		if ((m_debugMode & btIDebugDraw::DBG_NoHelpText) == 0) {
			//			setOrthographicProjection();
			//
			//			showProfileInfo(xOffset,yStart,yIncr);
			//
			//#ifdef USE_QUICKPROF
			//
			//
			//			if ( getDebugMode() & btIDebugDraw::DBG_ProfileTimings)
			//			{
			//				static int counter = 0;
			//				counter++;
			//				std::map<std::string, hidden::ProfileBlock*>::iterator iter;
			//				for (iter = btProfiler::mProfileBlocks.begin(); iter != btProfiler::mProfileBlocks.end(); ++iter)
			//				{
			//					char blockTime[128];
			//					sprintf(blockTime, "%s: %lf",&((*iter).first[0]),btProfiler::getBlockTime((*iter).first, btProfiler::BLOCK_CYCLE_SECONDS));//BLOCK_TOTAL_PERCENT));
			//					glRasterPos3f(xOffset,yStart,0);
			//					GLDebugDrawString(BMF_GetFont(BMF_kHelvetica10),blockTime);
			//					yStart += yIncr;
			//
			//				}
			//
			//			}
			//#endif //USE_QUICKPROF
			//			resetPerspectiveProjection();
		}

		glEnable(GL_LIGHTING);

	}

	updateCamera();
}

void DefaultSpace::renderscene(int pass) {

	btScalar m[16];
	btMatrix3x3 rot;
	rot.setIdentity();
	const unsigned numObjects = m_dynamicsWorld->getNumCollisionObjects();
	btVector3 wireColor(1, 0, 0);

//	for (unsigned i = 0; i < bodies.size(); i++) {
//		AbstractBody* metaBody = bodies[i];
//
//		for (unsigned p = 0; p < metaBody->bodies.size(); p++) {
//			//btCollisionObject* colObj = metaBody->shapes[p];
//
//			//btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray();
//			btRigidBody* body = btRigidBody::upcast(metaBody->bodies[p]);
//
//
//		}
//	}

	for (unsigned int i = 0; i < numObjects; i++) {
		btCollisionObject* colObj =	m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		btCollisionShape* shape = body->getCollisionShape();

		if (body && body->getMotionState()) {
			btDefaultMotionState* myMotionState =
					(btDefaultMotionState*) body->getMotionState();
			myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
			rot = myMotionState->m_graphicsWorldTrans.getBasis();
		} else {
			colObj->getWorldTransform().getOpenGLMatrix(m);
			rot = colObj->getWorldTransform().getBasis();
		}
		//		btVector3 wireColor(1.f,1.0f,0.5f); //wants deactivation
		//		if(i&1) wireColor=btVector3(0.f,0.0f,1.f);
		//		///color differently for active, sleeping, wantsdeactivation states
		//		if (colObj->getActivationState() == 1) //active
		//		{
		//			if (i & 1)
		//			{
		//				wireColor += btVector3 (1.f,0.f,0.f);
		//			}
		//			else
		//			{
		//				wireColor += btVector3 (.5f,0.f,0.f);
		//			}
		//		}
		//		if(colObj->getActivationState()==2) //ISLAND_SLEEPING
		//		{
		//			if(i&1)
		//			{
		//				wireColor += btVector3 (0.f,1.f, 0.f);
		//			}
		//			else
		//			{
		//				wireColor += btVector3 (0.f,0.5f,0.f);
		//			}
		//		}

		btVector3 aabbMin, aabbMax;
		m_dynamicsWorld->getBroadphase()->getBroadphaseAabb(aabbMin,
				aabbMax);

		aabbMin
				-= btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
		aabbMax
				+= btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
		//		printf("aabbMin=(%f,%f,%f)\n",aabbMin.getX(),aabbMin.getY(),aabbMin.getZ());
		//		printf("aabbMax=(%f,%f,%f)\n",aabbMax.getX(),aabbMax.getY(),aabbMax.getZ());
		//		m_dynamicsWorld->getDebugDrawer()->drawAabb(aabbMin,aabbMax,btVector3(1,1,1));

		btVector3 shapeColor(0.75, 0.75, 0.75);

		//TODO this is a hack to find the AbstractBody that manages a specific part
		for (unsigned j = 0; j < bodies.size(); j++) {
			AbstractBody* metaBody = bodies[j];
			int index = metaBody->indexOfShape(shape);
			if (index!=-1) {
				shapeColor = metaBody->getColor(shape);
				break;
			}
		}

		switch (pass) {
		case 0:
			m_shapeDrawer->drawOpenGL(m, shape,
					shapeColor, getDebugMode(), aabbMin, aabbMax);
			break;
		case 1:
			m_shapeDrawer->drawShadow(m, m_sundirection * rot,
					shape, aabbMin, aabbMax);
			break;
		case 2:
			m_shapeDrawer->drawOpenGL(m, shape,
					shapeColor * 0.3, 0, aabbMin, aabbMax);
			break;
		}

	}
}
