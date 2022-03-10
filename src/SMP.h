#pragma once
#include "nodeStruct.h"
#include "ofMain.h"
#include "obstacle.h"
#include <set>

class SMP
{
public:
	SMP();
	void addNode(Nodes n, std::list<Nodes>& nodes);
	Nodes* nearestNode(Nodes n, std::list<Nodes>& nodes);
	Nodes* nearestNode(Nodes n, std::list<Nodes*>& nodes);
	bool checkCollision(Nodes n1, Nodes n2, list<obstacles*> obst);
	bool checkSample(Nodes n, list<obstacles*> obst);
	Nodes sampler();
	bool goalFound = false;
	bool sampledInGoalRegion = false;
	bool moveNow = false;
	ofVec2f start;
	ofVec2f goal;
	Nodes* root;
	Nodes* target = NULL;
	Nodes* nextTarget = NULL;
};