#include "MultiRobot.h"

void MultiRobot::setup()
{
	robots.clear();
	Robot rbt;
	robots.push_back(rbt);
}

void MultiRobot::setup(ofVec2f loc)
{
	robots.clear();
	Robot rbt(loc);
	robots.push_back(rbt);
}

void MultiRobot::addAgent() {
	Robot rbt;
	robots.push_back(rbt);
}

void MultiRobot::addAgent(ofVec2f loc) {
	Robot rbt(loc);
	robots.push_back(rbt);
}

void MultiRobot::update()
{
	for (auto& rbt : robots) {
		rbt.update();
	}
}

void MultiRobot::render()
{
	for (auto& rbt : robots) {
		rbt.render();
	}

}

void MultiRobot::addForce(vector<ofVec2f> forces) {
	for (int i = 0; i < robots.size(); i++) {
		robots[i].addForce(forces[i]);
	}
}

void MultiRobot::controller(vector<ofVec2f> targets) {
	for (int i = 0; i < robots.size(); i++) {
		robots[i].controller(targets[i]);
	}
}

void MultiRobot::fillEnvironment(const list<obstacles*> obst, vector<list<Nodes>>& nodes)
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i].fillEnvironment(obst, nodes[i]);
	}
}

void MultiRobot::updateEnvironment(vector<list<Nodes>>& nodes, obstacles *obst)
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i].updateEnvironment(nodes[i], obst);
	}
}
