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
	myfont.load("Roboto-Regular.ttf", 10);

	for (unsigned int i = 0; i < numberOfobst; i++)
	{
		obstacles *ob = new obstacles();
		obst.push_back(ob);
	}
	
#ifdef readARMarkers
	std::thread t1(&ofApp::readAR, this);
	t1.detach();
#endif

#ifdef CLK
	auto end = std::chrono::steady_clock::now();
	std::cout << std::endl << "Setup:" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;
#endif // DEBUG
}

void ofApp::readAR() {
	// Process simulation video
	while (true) {
		cv::Mat frame;
		vid_capture >> frame;
		if (!frame.empty()) {
			std::vector<int> ids;
			std::vector<std::vector<cv::Point2f>> corners;
			cv::aruco::detectMarkers(frame, dictionary, corners, ids);
			
			// Objects: robot, goal, obstacles, goods
			obst.clear();
			ofVec2f loc;
			obstacles *ob;
			
			// if at least one marker detected
			if (ids.size() > 0) {
				cv::aruco::drawDetectedMarkers(frame, corners, ids);
				std::vector<cv::Vec3d> rvecs, tvecs;
				cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
				// draw axis for each marker
				for (int i = 0; i < ids.size(); i++) {
					switch (ids[i]) {
					// Robot
					case 33: {
						ofVec2f r_pos(tvecs[i][0], tvecs[i][1]);
						float r_angle = rvecs[i][2];
						float target_angle = atan2(
							map->rtrrtstar.root->location.y - r_pos.y,
							map->rtrrtstar.root->location.x - r_pos.x
						);
						std::cout << "delta: " << target_angle - r_angle << std::endl;
						break;
					}
					// Goal
					case 50:

						break;
					// Obstacles
					case 56:
						loc.set(tvecs[i][0], tvecs[i][1]);
						ob = new obstacles(loc);
						obst.push_back(ob);
						break;
					// Goods
					case 82:

						break;
					}

					cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

				}
			}
		}
		else {
			vid_capture.open(vid);
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
	if (map != NULL) {
		map->update(car, obst);
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
	if (map != NULL) {
		map->render();
		char numNode[255];
		ofSetColor({ 234,97,50 });
		sprintf(numNode, "Number of nodes: %d", int(map->numofnode()));
		myfont.drawString(numNode, ofGetWindowWidth() - 140, ofGetWindowHeight() - 10);
	}
	
	if (car != NULL) {
		car->render();
		char carInfos[255];
		ofSetColor({ 9, 102, 139 });
		sprintf(carInfos, "Robot: (x, y) = (%.2f, %.2f); angle = %.2f", car->x(), car->y(), car->getAngle());
		myfont.drawString(carInfos, 50, ofGetWindowHeight() - 30);
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
		if (car != NULL) {
			map->targetSet(loc);
		}
	}
	else if (button == 2) {
		if (car == NULL){
			car = new Robot(loc);
			map = new Environment(car->getLocation());
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
