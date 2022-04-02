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
	ofSetFrameRate(120);
	ofSetWindowTitle("Dynamic-obstacles");
	ofBackground(200,200,200,200);
	myfont.loadFont("Roboto-Regular.ttf", 10);

	for (unsigned int i = 0; i < numStaObst; i++)
	{
		obstacles *ob = new obstacles();
		obst1.push_back(ob);
		obst2.push_back(ob);
		obst3.push_back(ob);
		if (i < numMovObst) {
			OBST = new movingObst();
			obstacles *ob = OBST;
			obst1.push_back(ob);
			obst2.push_back(ob);
			obst3.push_back(ob);
		}
	}

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
	for (auto i : obst1) {
		i->move(obst1);
	}
	for (auto i : obst2) {
		i->move(obst2);
	}
	for (auto i : obst3) {
		i->move(obst3);
	}
#endif // automatic

#ifdef readARMarkers
	// Process simulation video
	cv::Mat frame;
	bool isSuccess = vid_capture.read(frame);

	if (isSuccess) {
		// Process AR markers
		vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;
		vector<int> markerIds;
		cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
		cv::Mat outputImage = frame.clone();
		cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
		
		// Compute obstacles' positions
		obst1.clear();
		obst2.clear();
		ofVec2f loc;
		obstacles *ob;
		for (auto markerCorner : markerCorners){
			loc.set(markerCorner[0].x, markerCorner[0].y);
			ob = new obstacles(loc);
			obst1.push_back(ob);
			obst2.push_back(ob);
		}
		// Set mutual obstacle
		if (car1 != NULL) {
			loc.set(car1->x(), car1->y());
			ob = new obstacles(loc);
			obst2.push_back(ob);
		}
		if (car2 != NULL) {
			loc.set(car2->x(), car2->y());
			ob = new obstacles(loc);
			obst1.push_back(ob);
		}

		// For debugging
		/*char fileName[50];
		sprintf(fileName, "bin\\data\\frames\\frame_%d.jpg", frameIdx);
		std::cout << "Writing AR results in: " << fileName << std::endl;
		cv::imwrite(fileName, outputImage);*/

		frameIdx++;
	}
	else {
		vid_capture.open(vid_source);
		frameIdx = 0;
	}
#endif

	// Set mutual obstacles
	ofVec2f loc;
	obstacles *ob;

	// Obstacle 1: assign car2 + car3 as obstacles
	while (obst1.size() > numObs) {
		obst1.pop_back();
	}
	if (car2 != NULL) {
		loc.set(car2->x(), car2->y());
		ob = new obstacles(loc);
		obst1.push_back(ob);
	}
	if (car3 != NULL) {
		loc.set(car3->x(), car3->y());
		ob = new obstacles(loc);
		obst1.push_back(ob);
	}

	// Obstacle 2: assign car3 + car1 as obstacles
	while (obst2.size() > numObs) {
		obst2.pop_back();
	}
	if (car3 != NULL) {
		loc.set(car3->x(), car3->y());
		ob = new obstacles(loc);
		obst2.push_back(ob);
	}
	if (car1 != NULL) {
		loc.set(car1->x(), car1->y());
		ob = new obstacles(loc);
		obst2.push_back(ob);
	}

	// Obstacle 3: assign car1 + car2 as obstacles
	while (obst3.size() > numObs) {
		obst3.pop_back();
	}
	if (car1 != NULL) {
		loc.set(car1->x(), car1->y());
		ob = new obstacles(loc);
		obst3.push_back(ob);
	}
	if (car2 != NULL) {
		loc.set(car2->x(), car2->y());
		ob = new obstacles(loc);
		obst3.push_back(ob);
	}

	if (map1 != NULL) {
		map1->update(car1, obst1);
	}
	if (map2 != NULL) {
		map2->update(car2, obst2);
	}
	if (map3 != NULL) {
		map3->update(car3, obst3);
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
	
	list<obstacles*>::iterator it;
	for (it = obst1.begin(); std::distance(obst1.begin(), it) < numObs; it++) {
		(*it)->render();
	}

	if (map1 != NULL) map1->render();
	if (map2 != NULL) map2->render();
	if (map3 != NULL) map3->render();

	if (car1 != NULL) car1->render();
	if (car2 != NULL) car2->render();
	if (car3 != NULL) car3->render();

	if (map1 != NULL) {
		char numNode[255];
		sprintf(numNode, "Number of nodes: %d", int(map1->numofnode()));
		myfont.drawString(numNode, ofGetWindowWidth() - 140, ofGetWindowHeight() - 10);
	}
	
	char fpsStr[255]; // an array of chars
	ofSetColor({ 255,0,0 });
	sprintf(fpsStr, "Frame rate: %d", int(ofGetFrameRate()));
	myfont.drawString(fpsStr, ofGetWindowWidth() - 140, ofGetWindowHeight() - 25);

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
		map1->grid = !map1->grid;
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
		if (car1 != NULL) {
			map1->targetSet(loc);
		}
		if (car2 != NULL) {
			map2->targetSet(loc);
		}
		if (car3 != NULL) {
			map3->targetSet(loc);
		}
	}
	else if (button == 2) {
		if (car1 == NULL){
			car1 = new Robot(loc);
			map1 = new Environment(car1->getLocation());
		}
		else if (car2 == NULL) {
			car2 = new Robot(loc);
			map2 = new SubEnvironment(car2->getLocation());
		}
		else if (car3 == NULL) {
			car3 = new Robot(loc);
			map3 = new SubEnvironment1(car3->getLocation());
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
