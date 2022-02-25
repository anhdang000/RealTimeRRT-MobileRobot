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
	// Update method
	void update();
	//Render method
	void render();
	void fillEnviroment(const list<obstacles*> obst, vector<list<Nodes>> &nodes);
	void updateEnviroment(vector<list<Nodes>> &nodes, obstacles *obst);
	//--------------------------------------------------------------Variables
private:
	vector<Robot> robots;
};

