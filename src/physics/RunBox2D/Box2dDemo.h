/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef BOX2D_DEMO_H
#define BOX2D_DEMO_H

#include "../OpenGL/AbstractSpace.h"
#include "../LinearMath/btAlignedObjectArray.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
class GL_DialogDynamicsWorld;

///Box2dDemo is good starting point for learning the code base and porting.
class Box2dDemo : public AbstractSpace
{

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	GL_DialogDynamicsWorld*	m_dialogDynamicsWorld;

	public:

	Box2dDemo() : m_dialogDynamicsWorld(0)
	{
	}

	virtual ~Box2dDemo()	{
		exitPhysics();
	}

	virtual void reshape(int w, int h);

	void	initPhysics();

	void	exitPhysics();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();
	
	static SpaceProcess* Create()
	{
		Box2dDemo* demo = new Box2dDemo;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	virtual void mouseFunc(int button, int state, int x, int y);
	virtual void	mouseMotionFunc(int x,int y);

	
};

void runBox2D();

#endif //BOX2D_DEMO_H

