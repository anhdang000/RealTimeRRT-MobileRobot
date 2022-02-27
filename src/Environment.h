#pragma once
#include "simulationParam.h"
#include "nodeStruct.h"
#include "obstacle.h"
#include <list>
#include "SMP.h"
#include "Robot.h"
#include "MultiRobot.h"
#include "RRTstar.h"
#include "InformedRRTstar.h"
#include "RT-RRTstar.h"
#include "ofxGui.h"


class Color {
public:
	Color() { 
		this->r = ofRandom(0, 256);  
		this->g = ofRandom(0, 256);
		this->b = ofRandom(0, 256);
	}
	Color(float rMin, float rMax, float gMin, float gMax, float bMin, float bMax) {
		this->r = ofRandom(rMin, rMax);
		this->g = ofRandom(gMin, gMax);
		this->b = ofRandom(bMin, bMax);
	}
	Color(int r, int g, int b) {
		this->r = r;
		this->g = g;
		this->b = b;
	}
	void set(int r, int g, int b) {
		this->r = r;
		this->g = g;
		this->b = b;
	}
	int getR() { return r; }
	int getG() { return g; }
	int getB() { return b; }

private:
	int r;
	int g;
	int b;

};

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

	// Add elements to attribute vectors
	void addElements(ofVec2f loc);

	// Setup colors
	void addColorsSet();
	int getNumElements() { return smps.size(); }
	bool assertNumElements();
	void update(MultiRobot *multiRobot, list<obstacles*> obst);
	void setTarget(ofVec2f loc, int i);
	void addTarget(ofVec2f loc);
	// Update method
	void update(MultiRobot *multiRobot);
	// Render method draw nodes in enviroment.
	void render();
	vector<int> numOfNode();
	void renderGrid();
	//--------------------------------------------------------------Variables
	bool grid = false;
	vector<bool> goalsIn;

	ofxFloatSlider guiRad,guiEpsilon;
	ofxPanel gui;
private:
	//--------------------------------------------------------------Variables
protected:
	//--------------------------------------------------------------Variables
	vector<SMP> smps;
	vector<list<Nodes>> nodes;
	vector<list<Nodes*>> paths;
	vector<RRTstar> rrtstars;
	vector<InformedRRTstar> irrtstars;
	vector<RTRRTstar> rtrrtstars;
	vector<bool> rrtFlags;
	vector<bool> planners;

	vector<ofVec2f> goals;
	vector<ofVec2f> homes;
	
	// For nodes visualization
	vector<Color> goalColors;
	vector<Color> orgColors;
	vector<Color> occupiedColors;
	vector<Color> pathColors;
};

