#pragma once
#include "Robot.h"
#include <vector>

class MultiRobot
{
public:
	//--------------------------------------------------------------Function
	// Default constructor  
	MultiRobot() { setup(); }
	MultiRobot(ofVec2f loc) { setup(loc); }
	// Default destructor  
	~MultiRobot() {};
	void setup();
	void setup(ofVec2f loc);

	int getNumRobots() { return robots.size(); }
	// Add agent
	void addAgent();
	void addAgent(ofVec2f loc);
	// Update method
	void update();
	// Render method
	void render();
	// Compute force addition
	void addForce(vector<ofVec2f> forces);
	// Controller genrate force toward target
	void controller(vector<ofVec2f> targets);

	void fillEnvironment(const list<obstacles*> obst, vector<list<Nodes>> &nodes);
	void updateEnvironment(vector<list<Nodes>> &nodes, obstacles *obst);
	//--------------------------------------------------------------Variables

	vector<Robot> robots;
};

