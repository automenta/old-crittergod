//============================================================================
// Name        : crittergod4.cpp
// Author      : crittergod.sourceforge.net
// Version     :
// Copyright   : Copyright (C) 2010 CritterGod. All Rights Reserved.
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <string.h>

using namespace std;

#include "neural/Brain.h"
#include "video/GLWindow.h"
#include "objects/BrainVis.h"

#include "graph/Graph.h"

#include "physics/RunBox2D/Box2dDemo.h"
#include "space/DefaultSpace.h"
#include "physics/OpenGL/GlutStuff.h"

//#include "physics/optic/Raytracer.h"

#include "video/font/FontDemo1.h"

#define VERSION "CritterGod 0.01"

#include "space/AbstractBody.h"
#include "space/CritterBody.h"
#include "space/BoxBody.h"

void runSim() {
	DefaultSpace* ds = new DefaultSpace();

	SnakeBody* snake1 = new SnakeBody(btVector3(0,0,0), 8);
	ds->addBody(snake1);

	SpiderBody* spider= new SpiderBody(6, btVector3(0,0,5));
	ds->addBody(spider);

	BoxBody* box = new BoxBody(btVector3(3,3,3), btVector3(1,0.5, 0.5));
	ds->addBody(box);

	runGLWindow(0, NULL, 1024, 800, VERSION, ds);
}

//void runRT() {
//    Raytracer* raytraceDemo = new Raytracer();
//
//
//    raytraceDemo->initPhysics();
//
//    raytraceDemo->setCameraDistance(6.f);
//
//    std::vector<SpaceProcess*> procs;
//	procs.push_back(raytraceDemo);
//
//
//	runGLWindow(0, NULL, 640, 640,"Bullet GJK Implicit Shape Raytracer Demo",&procs);
//}

void runBrainzDemo() {
	Brain* b = new Brain(48, 32, 16384, 1, 8, 0.4);
	b->printSummary();
	GLWindow* g = new GLWindow();
	g->create("Brain", 800, 600);
	g->start(new BrainVis(b));
}

void testGraph() {
	Graph g;

	string a = string("a");
	string b = string("b");

	g.addEdge(new string("ab"), &a, &b);
	g.addEdge(new string("cd"), &a, &b);

	g.print();

}

int main() {

	runSim();
	//runBrainzDemo();
	//runRT();
	//testGraph();

	//runFontDemo();

	return 0;
}
