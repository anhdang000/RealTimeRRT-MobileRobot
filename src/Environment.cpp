#include "Environment.h"
#include <set>


void Environment::setup()
{
	gui.setup();
	gui.add(guiRad.setup("Radius", rrtstarradius, 10, 200));
	gui.add(guiEpsilon.setup("Epsilon", epsilon, 5, 150));

	ofVec2f home(startx, starty);
	homes.push_back(home);

	ofVec2f goal(goalx, goaly);
	goals.push_back(goal);

	Nodes start(startx, starty, 0);
	list<Nodes> nodesPerBot;
	nodesPerBot.push_back(start);
	nodes.push_back(nodesPerBot);

	SMP smp;
	smp.start.set(startx, starty);
	smp.goal.set(goalx, goaly);
	smp.goalFound = false;
	smps.push_back(smp);

	list<Nodes*> path;
	paths.push_back(path);

	addColorsSet();

	RRTstar rrtstar;
	rrtstars.push_back(rrtstar);

	InformedRRTstar irrtstar;
	irrtstars.push_back(irrtstar);

	RTRRTstar rtrrtstar;
	rtrrtstars.push_back(rtrrtstar);

	bool rrtFlag = true;
	rrtFlags.push_back(rrtFlag);

	bool planner = true;
	planners.push_back(planner);

	bool goalIn = false;
	goalsIn.push_back(goalIn);
}

void Environment::setup(ofVec2f _start)
{
	gui.setup();
	gui.add(guiRad.setup("Radius", rrtstarradius, 10, 200));
	gui.add(guiEpsilon.setup("Epsilon", epsilon, 5, 150));
	
	homes.push_back(_start);

	ofVec2f goal(goalx, goaly);
	goals.push_back(goal);

	Nodes start(_start.x, _start.y, 0);
	list<Nodes> nodesPerBot;
	nodesPerBot.push_back(start);
	nodes.push_back(nodesPerBot);

	SMP smp;
	smp.root = &(nodesPerBot.front());
	smp.goal.set(goalx, goaly);
	smp.start.set(startx, starty);
	smp.goalFound = false;
	smps.push_back(smp);

	list<Nodes*> path;
	paths.push_back(path);

	addColorsSet();

	RRTstar rrtstar;
	rrtstars.push_back(rrtstar);

	InformedRRTstar irrtstar;
	irrtstars.push_back(irrtstar);

	RTRRTstar rtrrtstar;
	rtrrtstars.push_back(rtrrtstar);

	bool rrtFlag = true;
	rrtFlags.push_back(rrtFlag);

	bool planner = true;
	planners.push_back(planner);

	bool goalIn = false;
	goalsIn.push_back(goalIn);
}

void Environment::addColorsSet() {
	// Random around: {150, 0, 255}
	Color goalColor(140, 155, 0, 20, 230, 250);
	goalColors.push_back(goalColor);

	// Random around: {10, 10, 10}
	Color orgColor(5, 15, 5, 15, 5, 15);
	orgColors.push_back(orgColor);

	// Random around: {200, 0, 0}
	Color occupiedColor(150, 200, 0, 10, 0, 10);
	occupiedColors.push_back(occupiedColor);

	// Random around: {10, 250, 10}
	Color pathColor(5, 15, 220, 250, 5, 15);
	pathColors.push_back(pathColor);
}

void Environment::addElements(ofVec2f loc) {
	homes.push_back(loc);

	ofVec2f goal(goalx, goaly);
	goals.push_back(goal);

	Nodes start(loc.x, loc.y, 0);
	list<Nodes> nodesPerBot;
	nodesPerBot.push_back(start);
	nodes.push_back(nodesPerBot);

	SMP smp;
	smp.root = &(nodesPerBot.front());
	smp.goal.set(goalx, goaly);
	smp.start.set(startx, starty);
	smp.goalFound = false;
	smps.push_back(smp);

	list<Nodes*> path;
	paths.push_back(path);

	addColorsSet();

	RRTstar rrtstar;
	rrtstars.push_back(rrtstar);

	InformedRRTstar irrtstar;
	irrtstars.push_back(irrtstar);

	RTRRTstar rtrrtstar;
	rtrrtstars.push_back(rtrrtstar);

	bool rrtFlag = true;
	rrtFlags.push_back(rrtFlag);

	bool planner = true;
	planners.push_back(planner);

	bool goalIn = false;
	goalsIn.push_back(goalIn);
}

bool Environment::assertNumElements() {
	vector<int> nums;
	nums.push_back(nodes.size());
	nums.push_back(paths.size());
	nums.push_back(rrtstars.size());
	nums.push_back(irrtstars.size());
	nums.push_back(rrtFlags.size());
	nums.push_back(planners.size());
	nums.push_back(goals.size());
	nums.push_back(homes.size());

	set<int> nums_set(nums.begin(), nums.end());
	return nums_set.size() == 1;
}

void Environment::update(MultiRobot *multiRobot, list<obstacles*> obst)
{
	for (int i = 0; i < getNumElements(); i++) {
		Robot *rbt = &(multiRobot->robots[i]);
		if (rbt->getLocation().distance(smps[i].goal) < converge)
			planners[i] = false;

		if (planners[i])
		{
			rbt->fillEnvironment(obst, nodes[i]);
			rbt->controller(smps[i].root->location);
			rbt->update();
		}

		rtrrtstars[i].nextIter(nodes[i], obst, rbt);

		if (planners[i] && smps[i].target != NULL)
		{
			paths[i] = rtrrtstars[i].currPath;
			rtrrtstars[i].currPath.clear();
		}
	}
}

void Environment::setTarget(ofVec2f loc, int i) {
	goals[i] = loc;
	smps[i].goal = loc;
	rtrrtstars[i].goalDefined = true;

	planners[i] = true;
	std::list<Nodes>::iterator it = nodes[i].begin();
	while (it != nodes[i].end())
	{
		if ((*it).location.distance(loc) < converge)
		{
			smps[i].target = &(*it);
			return;
		}
		it++;
	}
	smps[i].goalFound = false;
	smps[i].target = NULL;
	paths[i].clear();
	goalsIn[i] = true;
}

void Environment::render()
{
	for (int i = 0; i < getNumElements(); i++) {
		//gui.draw();
		ofEnableAlphaBlending();

		ofSetColor(goalColors[i].getR(), goalColors[i].getG(), goalColors[i].getB());
		if (goalsIn[i]) {
			ofFill();
			ofDrawCircle(goals[i].x, goals[i].y, NODE_RADIUS + 2);
			ofNoFill();
			ofSetLineWidth(2);
			ofDrawCircle(goals[i].x, goals[i].y, converge);
		}

		for (auto node : this->nodes[i])
		{
			ofSetColor(orgColors[i].getR(), orgColors[i].getG(), orgColors[i].getB(), 50);

			if (node.costToStart == inf) {
				ofSetColor(occupiedColors[i].getR(), occupiedColors[i].getG(), occupiedColors[i].getB(), 50);
			}
			ofSetLineWidth(2);
			if (node.parent != NULL) {
				ofPoint pt; ofPolyline line;
				pt.set(node.location.x, node.location.y); line.addVertex(pt);
				pt.set(node.parent->location.x, node.parent->location.y); line.addVertex(pt);
				line.draw();
			}
			//ofSetColor({ 10,10,250 },80);
			ofSetLineWidth(1);
			//if (i.prevParent != NULL) {
			//	
			//	ofPoint pt; ofPolyline line;
			//	pt.set(i.location.x, i.location.y); line.addVertex(pt);
			//	pt.set(i.prevParent->location.x, i.prevParent->location.y); line.addVertex(pt);
			//	line.draw();
			//}
			//int hue = i.alive ? 130 : 80;
			//ofSetColor(i.color, hue);
			//ofDrawCircle(i.location.x, i.location.y, NODE_RADIUS);
		}
		if (!paths[i].empty())
		{
			ofSetColor(pathColors[i].getR(), pathColors[i].getG(), pathColors[i].getB());
			ofSetLineWidth(5);
			for (auto i : paths[i]) {
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
}

void Environment::renderGrid()
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