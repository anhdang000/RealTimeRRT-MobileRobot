#pragma once
#include "simulationParam.h"
#include "nodeStruct.h"
#include "obstacle.h"
#include <list>
#include <vector>
#include "SMP.h"
#include "Robot.h"
#include "RRTstar.h"
#include "InformedRRTstar.h"
#include "RT-RRTstar.h"
#include "ofxGui.h"
#include <thread>

//--------------------------------------------------------------class
class Environment
{
public:
	//--------------------------------------------------------------Function
	// Default constructor  
	Environment() { setup(); };
	Environment(ofVec2f _start) { setup(_start); };
	// Default destructor  
	~Environment() {};
	// Setup method
	void setup();
	void setup(ofVec2f _start);
	void update(Robot * car, list<obstacles*> obst);
	void targetSet(ofVec2f loc);
	// Update method
	void update(Robot *car);
	// Render method draw nodes in Environment.
	void render();
	float numofnode() { return nodes.size(); };
	void renderGrid();
	//--------------------------------------------------------------Variables
	bool grid = false;
	bool goalin = false;
	ofxFloatSlider guiRad,guiEpsilon;
	ofxPanel gui;
private:
	//--------------------------------------------------------------Variables
public:
	//--------------------------------------------------------------Variables
	std::list<Nodes> nodes;
	//std::list<obstacles> obst;
	std::list<Nodes*> path;
	RTRRTstar rtrrtstar;
	bool rrtFlag = true;
	bool planner = true;

	ofVec2f goal;
	ofVec2f home;

};

inline void Environment::setup()
{
	home.set(startx, starty);
	Nodes start(startx, starty, 0);
	this->nodes.push_back(start);
	rtrrtstar.start.set(startx, starty);
	rtrrtstar.goalFound = false;
}

inline void Environment::setup(ofVec2f _start)
{
	gui.setup();
	gui.add(guiRad.setup("Radius", rrtstarradius, 10, 200));
	gui.add(guiEpsilon.setup("Epsilon",epsilon , 5, 150));
	home = _start;

	Nodes start(home.x, home.y, 0);
	this->nodes.push_back(start);

	rtrrtstar.root = &(this->nodes.front());
	goal.set(goalx, goaly);
	rtrrtstar.start.set(startx, starty);
	rtrrtstar.goalFound = false;
}

inline void Environment::update(Robot *car,list<obstacles*> obst) {
	if (car->getLocation().distance(rtrrtstar.goal) < converge)
		planner = false;

	if (planner)
	{
		car->fillEnvironment(obst, nodes);
		car->controller(rtrrtstar.root->location);
		car->update();
	}

	rtrrtstar.nextIter(nodes, obst, car);

	if (planner && rtrrtstar.target != NULL)
	{
		path = rtrrtstar.currPath;
		rtrrtstar.currPath.clear();
	}
}

inline void Environment::targetSet(ofVec2f loc)
{
	goal = loc;
	rtrrtstar.goal = goal;
	rtrrtstar.goalDefined = true;
	
	planner = true;
	std::list<Nodes>::iterator it = nodes.begin();
	while (it != nodes.end())
	{
		if ((*it).location.distance(loc) < converge)
		{
			rtrrtstar.target = &(*it);
			return;
		}
		it++;
	}
	rtrrtstar.goalFound = false;
	rtrrtstar.target = NULL;
	path.clear();
	goalin = true;
}

inline void Environment::render()
{
	//gui.draw();
	ofEnableAlphaBlending();

	ofSetColor({150, 0, 255});
	if (goalin) {
		ofFill();
		ofDrawCircle(goal.x, goal.y, NODE_RADIUS+2);
		ofNoFill();
		ofSetLineWidth(2);
		ofDrawCircle(goal.x, goal.y, converge);
	}

	for (auto i : this->nodes)
	{
		ofSetColor({ 10,10,150 }, 50);

		if (i.costToStart == inf) ofSetColor({ 200,0,0 },50);
		
		ofSetLineWidth(2);
		if (i.parent != NULL) {
			ofPoint pt;ofPolyline line;
			pt.set(i.location.x, i.location.y);line.addVertex(pt);
			pt.set(i.parent->location.x, i.parent->location.y);line.addVertex(pt);
			line.draw();
		}
		ofSetLineWidth(1);
	}
	if (!path.empty())
	{
		ofSetColor({ 30,195,152 });
		ofSetLineWidth(5);
		for (auto i : path) {
			if (i->parent != NULL) {
				ofPoint pt; ofPolyline line;
				pt.set(i->location.x, i->location.y); line.addVertex(pt);
				pt.set(i->parent->location.x, i->parent->location.y); line.addVertex(pt);
				line.draw();
			}
		}
		ofSetLineWidth(1);
	}
	ofDisableAlphaBlending();
}

inline void Environment::renderGrid()
{
	ofEnableAlphaBlending();
	ofSetColor(20, 130, 0, 50);
	for (int i = 0; i < ofGetWindowWidth(); i += 5)
	{
		cout << i << endl;
		ofDrawLine(i, 0, i, ofGetWindowHeight());
	}
	for (int j = 0; j < ofGetWindowHeight(); j += 5)
	{
		ofDrawLine(0, j, ofGetWindowWidth(), j);
	}
	ofDisableAlphaBlending();
}