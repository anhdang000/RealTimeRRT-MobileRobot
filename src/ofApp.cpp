#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
#ifdef randomSeed
	ofSeedRandom(randomSeed);
#endif // randomSeed
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG
	// Storage files
	posErrorFile_1.open("bin\\data\\pos_errors_1.txt");
	posErrorFile_2.open("bin\\data\\pos_errors_2.txt");

	// Display
	ofSetVerticalSync(true);
	ofSetFrameRate(120);
	ofSetWindowTitle("Dynamic-obstacles");
	ofBackground(200,200,200,200);
	myfont.loadFont("Roboto-Regular.ttf", 10);

	// Robots and Goals setup
	start1.set(100, 100);
	goal1.set(700, 400);
	car1 = new Robot(start1);
	map1 = new Environment(car1->getLocation());
	map1->targetSet(goal1);

	start2.set(100, 700);
	goal2.set(700, 400);
	car2 = new Robot(start2);
	map2 = new SubEnvironment(car2->getLocation());
	map2->targetSet(goal2);

	// Start time
	initTime = std::chrono::system_clock::now();

	// Obstacles
	vector<ofVec2f> obsLoc = { ofVec2f(400, 250), ofVec2f(400, 400), ofVec2f(400, 550) };
	for (unsigned int i = 0; i < obsLoc.size(); i++)
	{
		obstacles *ob = new obstacles(obsLoc[i]);
		obst1.push_back(ob);
		obst2.push_back(ob);
	}

	for (unsigned int i = 0; i < numMovObst; i++) {
		OBST = new movingObst();
		obstacles *ob = OBST;
		obst1.push_back(ob);
		obst2.push_back(ob);
	}

	drawAtInit();
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

	// Obstacle 2: assign car3 + car1 as obstacles
	while (obst2.size() > numObs) {
		obst2.pop_back();
	}
	if (car1 != NULL) {
		loc.set(car1->x(), car1->y());
		ob = new obstacles(loc);
		obst2.push_back(ob);
	}

	// Obstacle 3: assign car1 + car2 as obstacles

	if (map1 != NULL) {
		map1->update(car1, obst1);
	}
	if (map2 != NULL) {
		map2->update(car2, obst2);
	}

	// Write to files
	// "x    y    error    num_nodes    timestamp"
	std::chrono::duration<double> diff = std::chrono::system_clock::now() - initTime;
	float error1 = car1->computeDeviationError(start1, goal1);
	float error2 = car2->computeDeviationError(start2, goal2);
	posErrorFile_1 << car1->x() << "\t" << car1->y() << "\t" << error1 << "\t" << map1->numofnode() << "\t" << diff.count() << "\n";
	posErrorFile_2 << car2->x() << "\t" << car2->y() << "\t" << error2 << "\t" << map2->numofnode() << "\t" << diff.count() << "\n";

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
	// Draw standard paths
	ofSetColor({ 150, 50, 25 });
	ofDrawLine(start1.x, start1.y, goal1.x, goal1.y);
	ofDrawLine(start2.x, start2.y, goal2.x, goal2.y);

	// Obstacles
	list<obstacles*>::iterator it;
	for (it = obst1.begin(); std::distance(obst1.begin(), it) < numObs; it++) {
		(*it)->render();
	}

	if (map1 != NULL) map1->render();
	if (map2 != NULL) map2->render();

	if (car1 != NULL) car1->render();
	if (car2 != NULL) car2->render();

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
void ofApp::drawAtInit() {
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG
	// Draw standard paths
	ofSetColor({ 150, 50, 25 });
	ofDrawLine(start1.x, start1.y, goal1.x, goal1.y);
	ofDrawLine(start2.x, start2.y, goal2.x, goal2.y);

	list<obstacles*>::iterator it;
	for (it = obst1.begin(); std::distance(obst1.begin(), it) < numObs; it++) {
		(*it)->render();
	}

	if (map1 != NULL) map1->render();
	if (map2 != NULL) map2->render();

	if (car1 != NULL) car1->render();
	if (car2 != NULL) car2->render();

	ofImage img;
	img.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
	img.save("screenshots/at_init.jpg");
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
		char name[50];
		sprintf(name, "screenshots/screen_%d.jpg", std::time(0));
		std::cout << "Saved screen at: " << name << std::endl;
		img.save(name);
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
	if (button == 0) {
	}
	else if (button == 2) {
		
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
