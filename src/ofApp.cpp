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
	ofSetWindowTitle("Dynamic-obstacles");
	ofBackground(200,200,200,200);
	myfont.loadFont("Roboto-Regular.ttf", 10);

	
	ofVec2f w;
	w.set(ofGetWidth() / 2, 0);
	wall = new maze(w);
	obstacles *ob = wall;
	obst1.push_back(ob);
	obst2.push_back(ob);

	w.set(ofGetWidth() / 2, 0.6*ofGetHeight());
	wall = new maze(w);
	ob = wall;
	obst1.push_back(ob);
	obst2.push_back(ob);

	w.set(ofGetWidth() / 4, 0.4*ofGetHeight());
	wall = new maze(w, 60, 0.2*ofGetHeight());
	ob = wall;
	obst1.push_back(ob);
	obst2.push_back(ob);

	w.set(0.75*ofGetWidth(), 0.4*ofGetHeight());
	wall = new maze(w, 60, 0.2*ofGetHeight());
	ob = wall;
	obst1.push_back(ob);
	obst2.push_back(ob);

	for (unsigned int i = 0; i < numberOfobst; i++)
	{
		obstacles *ob = new obstacles();
		// OBST = new movingObst();
		// obstacles *ob = OBST;
		obst1.push_back(ob);
		obst2.push_back(ob);
	}
	//
	OBST = new movingObst();
	ob = OBST;
	obst1.push_back(ob);
	obst2.push_back(ob);
	
#ifdef readARMarkers
	std::thread t1(&ofApp::readAR, this);
	t1.detach();
#endif

#ifdef randomSeed
	std::cout << "RandomSeed:" << randomSeed << endl;
#endif

#ifdef CLK
	auto end = std::chrono::steady_clock::now();
	std::cout << std::endl << "Setup:" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;
#endif // DEBUG
}

void ofApp::readAR() {
	// Process simulation video
	cv::Mat frame;
	while (true) {
		bool isSuccess = vid_capture.read(frame);
		if (isSuccess) {
			// Process AR markers
			cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

			// Compute obstacles' positions
			obst1.clear();
			obst2.clear();
			ofVec2f loc;
			obstacles *ob;
			for (auto markerCorner : markerCorners) {
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
			/*cv::Mat outputImage = frame.clone();
			cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
			char fileName[50];
			sprintf(fileName, "bin\\data\\frames\\frame_%d.jpg", frameIdx);
			std::cout << "Writing AR results in: " << fileName << std::endl;
			cv::imwrite(fileName, outputImage);*/

			// frameIdx++;
		}
		else {
			vid_capture.open(vid_source);
			// frameIdx = 0;
		}
	}
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
#endif // automatic

	if (map1 != NULL) {
		map1->update(car1, obst1);
	}
	if (map2 != NULL) {
		map2->update(car2, obst2);
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
	if (car2 != NULL) {
		for (it = obst1.begin(); std::distance(it, obst1.end()) > 1; it++) {
			(*it)->render();
		}
	}
	else {
		for (it = obst1.begin(); std::distance(it, obst1.end()) > 0; it++) {
			(*it)->render();
		}
	}

	if (map1 != NULL) map1->render();
	if (map2 != NULL) map2->render();
	if (car1 != NULL) car1->render();
	if (car2 != NULL) car2->render();

	if (map1 != NULL) {
		char numNode[255];
		ofSetColor({ 234,97,50 });
		sprintf(numNode, "Number of nodes: %d", int(map1->numofnode()));
		myfont.drawString(numNode, ofGetWindowWidth() - 140, ofGetWindowHeight() - 10);
	}
	
	if (car1 != NULL) {
		char car1Infos[255];
		ofSetColor({ 9, 139, 50 });
		sprintf(car1Infos, "Robot 1: (x, y) = (%.2f, %.2f); orientation = %.2f", car1->x(), car1->y(), car1->getAngle());
		myfont.drawString(car1Infos, 50, ofGetWindowHeight() - 30);
	}

	if (car2 != NULL) {
		char car2Infos[255];
		ofSetColor({ 9, 102, 139 });
		sprintf(car2Infos, "Robot 2: (x, y) = (%.2f, %.2f); orientation = %.2f", car2->x(), car2->y(), car2->getAngle());
		myfont.drawString(car2Infos, 50, ofGetWindowHeight() - 10);
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
