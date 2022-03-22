#pragma once

#include "ofMain.h"
#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
#include <filesystem>
#include "Environment.h"
#include "SubEnvironment.h"
#include <thread>

#define _CRT_SECURE_NO_WARNINGS

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void readAR();
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
	private:
		ofTrueTypeFont myfont;
		bool updateFlag = true;
		Environment *map1;
		SubEnvironment *map2;
		Robot *car1;
		Robot *car2;
		list<obstacles*> obst1;
		list<obstacles*> obst2;
		movingObst *OBST;
		maze *wall;
		double updateTime = 0, drawTime = 0;

		string vid_source = "bin\\data\\simulation_AR_0.avi";
		cv::VideoCapture vid_capture = cv::VideoCapture(vid_source);
		int frameIdx = 0;
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
		cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
		vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;
		vector<int> markerIds;
};
