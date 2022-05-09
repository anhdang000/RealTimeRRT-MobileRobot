#include "Robot.h"

void Robot::setup()
{
		alive = true; mass = 5.0; scanRadius = sensorRadius; accuracy = accur;
		//battery = 100;
		//float x = ofRandom(0, ofGetWindowWidth()); float y = ofRandom(0, ofGetWindowHeight());`
		location.set(0.0,0.0); HOME = location;
		velocity.set(0.0, 0.0);
		accelaration.set(0.0, 0.0);
		maxVelocity.set(mVal, mVal);
		maxForce.set(mForce, mForce);
		//color = { ofRandom(0,255),ofRandom(0,255) ,ofRandom(0,255) };
		color = {50,145,80};
}

void Robot::setup(ofVec2f loc)
{
	alive = true; mass = 5.0; scanRadius = sensorRadius; accuracy = accur;
	location = loc;
	HOME = location;
	velocity.set(0.0, 0.0);
	accelaration.set(0.0, 0.0);
	maxVelocity.set(mVal, mVal);
	maxForce.set(mForce, mForce);
	color = { ofRandom(0,250),ofRandom(0,250) ,ofRandom(0,250) };
}

void Robot::update()
{
	velocity += accelaration;
	velocity = (velocity.length() <= maxVelocity.length()) ? velocity : (velocity.normalized() *mVal);
	location += velocity;
	accelaration *= 0.0;
	pt.set(location.x, location.y);
	line.addVertex(pt);
}

void Robot::render()
{

	int r = 6;
	ofEnableAlphaBlending();
	ofFill();
	ofSetColor(color);
	ofSetLineWidth(3);

	this->line.draw();

	ofSetLineWidth(1);
	ofNoFill();
	ofSetColor(color);
	ofDrawCircle(location.x,location.y,scanRadius);

	ofPushMatrix();
	ofTranslate(location.x,location.y);
	ofRotate(ofRadToDeg(atan2(velocity.y, velocity.x)));
	ofFill();
	ofDrawRectangle(-rWidth/2, -rLength/2, rWidth, rLength);
	/*ofSetColor(250, 180, 0);
	ofSetLineWidth(3);
	ofDrawLine(0, 0, rWidth / 2, 0);*/
	ofPopMatrix();

	/*ofSetColor(color, 80);
	ofDrawCircle(location.x, location.y, ofGetFrameNum() % int(scanRadius));
	ofNoFill();
	ofDisableAlphaBlending();*/
}

void Robot::addForce(ofVec2f force)
{
	accelaration += (force / mass);
}

void Robot::controller(ofVec2f target)
{
	ofVec2f error = (target - location);
	//error.normalize();
	//error *= 1.5;
	//accelaration = error;
	float m;
	if (error.length() < converge) {
		m = ofMap(error.length(), 0, converge, 0, mVal);
	}
	else {
		m = mVal;
	}

	ofVec2f temp = error.normalized()*m;
	ofVec2f steer = (temp - velocity);
	steer = (steer.length() <= maxForce.length()) ? steer : (steer.normalized() *mForce);
	addForce(steer);
}

void Robot::fillEnvironment(const list<obstacles*> obst, list<Nodes>& node)
{
	//check for Environment
	for (auto index : obst) {
		float dist = this->location.distance(index->loc());
		if (dist <= this->scanRadius + index->rad()) {
			updateEnvironment(node, index);
		}
	}
}

void Robot::updateEnvironment(list<Nodes>& node,obstacles *obst)
{
	std::list<Nodes>::iterator it = node.begin();
	while (it != node.end())
	{
		float dist = it->location.distance(obst->loc());
		if (dist <= obst->rad()) {
			it->costToStart = inf;
			it->alive = false;
		}
		it++;
	}
}