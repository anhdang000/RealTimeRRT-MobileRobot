#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
#ifdef randomSeed
	ofSeedRandom(randomSeed);
#endif // randomSeed
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG
	ofSetVerticalSync(true);
	ofSetFrameRate(30);
	ofSetWindowTitle("Real-time RRT");
	ofBackground(200, 200, 200, 200);
	myfont.load("Roboto-Regular.ttf", 10);

	/*ofVec2f start, goal;
	start.set(100, 100);
	goal.set(ofGetWidth() - 100, ofGetHeight() - 100);
	car = new Robot(start);
	map = new Environment(car->getLocation());
	map->targetSet(goal);*/

	ofVec2f w;
	w.set(ofGetWidth() / 2, 0);
	wall = new maze(w);
	obstacles *ob = wall;
	obst.push_back(ob);

	w.set(ofGetWidth() / 2, 0.6*ofGetHeight());
	wall = new maze(w);
	ob = wall;
	obst.push_back(ob);

	w.set(ofGetWidth() / 4, 0.4*ofGetHeight());
	wall = new maze(w, 60, 0.2*ofGetHeight());
	ob = wall;
	obst.push_back(ob);

	w.set(0.75*ofGetWidth(), 0.4*ofGetHeight());
	wall = new maze(w, 60, 0.2*ofGetHeight());
	ob = wall;
	obst.push_back(ob);

	for (unsigned int i = 0; i < numberOfobst; i++)
	{
		obstacles *ob = new obstacles();
		// OBST = new movingObst();
		// obstacles *ob = OBST;
		obst.push_back(ob);
	}
	//
	OBST = new movingObst();
	ob = OBST;
	obst.push_back(ob);

	cout << "Obst size: " << obst.size() << endl;

#ifdef randomSeed
	std::cout << "RandomSeed:" << randomSeed << endl;
#endif

#ifdef CLK
	auto end = std::chrono::steady_clock::now();
	std::cout << std::endl << "Setup:" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;
#endif // DEBUG
}

//--------------------------------------------------------------
void ofApp::update(){
	if (!updateFlag) return;
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG

#ifdef automatic
	for (auto i : obst) {
		i->move(obst);
		//cout << "location: " << i->loc() << "Radius: " << i->rad() << endl;
		//cout << i.getX() << "  " << i.getY() << endl;
	}
#endif // automatic

	if (map != NULL) {
		map->update(multiRobot, obst);
	}
#ifdef CLK
	auto end = std::chrono::steady_clock::now();
	/*std::cout << std::endl << "Update:" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;*/
	updateTime = std::chrono::duration<double, std::milli>(end - start).count();
#endif // DEBUG
}

//--------------------------------------------------------------
void ofApp::draw(){
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG

	for (auto i : obst) {
		i->render();
	}
	if (map != NULL) {
		map->render();
		multiRobot->render();
	}
	char fpsStr[255]; // an array of chars
	ofSetColor({ 255,0,0 });
	sprintf(fpsStr, "Frame rate: %d", int(ofGetFrameRate()));
	myfont.drawString(fpsStr, ofGetWindowWidth() - 150, ofGetWindowHeight() - 25);
	if (map != NULL) {
		char numNode[255];
		// sprintf(numNode, "Number of nodes: %d", int(map->numofnode()));
		myfont.drawString(numNode, ofGetWindowWidth() - 150, ofGetWindowHeight() - 10);
	}

#ifdef CLK
	auto end = std::chrono::steady_clock::now();
	/*std::cout << std::endl << "Draw:" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;*/
	drawTime = std::chrono::duration<double, std::milli>(end - start).count();

	char time[255];
	sprintf(time, "Update rate: %f", updateTime);
	myfont.drawString(time, ofGetWindowWidth() - 140, ofGetWindowHeight() - 755);
	sprintf(time, "Draw rate: %f", drawTime);
	myfont.drawString(time, ofGetWindowWidth() - 140, ofGetWindowHeight() - 740);

#endif // DEBUG
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 'p')
	{
		updateFlag = !updateFlag;
	}
	else if(key=='g')
	{
		map->grid = !map->grid;
	}
	else if (key == 'x') {
		ofImage img;
		img.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
		img.save("screenshot.png");
	}
#ifdef manual
	OBST->move(key);
#endif // manual

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	ofVec2f loc;
	loc.set(x, y);
	if (button == 0) {
		int idx = map->getNumElements();
		map->setTarget(loc, idx);
	}
	else if (button == 2) {
		if (!map) {
			multiRobot = new MultiRobot(loc);
			map = new Environment(loc);
		}
		else {
			multiRobot->addAgent(loc);
		}
	}
	else
	{

	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
