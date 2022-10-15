#pragma once

#include "ofMain.h"
#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
#include <filesystem>
#include "Environment.h"
#include "SubEnvironment.h"
#include <thread>
#include <functional>

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
		Environment *map;
		Robot *car;
		list<obstacles*> obst;
		movingObst *OBST;
		maze *wall;
		double updateTime = 0, drawTime = 0;

		cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 895.35076989, 0, 248.37585485, 0, 892.32692372, 140.3891681, 0, 0, 1);
		cv::Mat distCoeffs = (cv::Mat1d(1, 5) << 0.03825588, 0.14136728, -0.01338866, -0.00669156, -1.08208082);
		
		int cam_idx = 0;
		string vid = "bin\\data\\sample_video_0.avi";
		cv::VideoCapture vid_capture = cv::VideoCapture(vid);
		int frameIdx = 0;
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
		cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
		vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;
		vector<int> markerIds;
};
