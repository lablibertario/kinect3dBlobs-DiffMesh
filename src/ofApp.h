#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxKinectBlobFinder.h"
#include "ofxKinectBlobTracker.h"

using namespace ofxCv;
using namespace cv;

// Windows users:
// You MUST install the libfreenect kinect drivers in order to be able to use
// ofxKinect. Plug in the kinect and point your Windows Device Manager to the
// driver folder in:
//
//     ofxKinect/libs/libfreenect/platform/windows/inf
//
// This should install the Kinect camera, motor, & audio drivers.
//
// You CANNOT use this driver and the OpenNI driver with the same device. You
// will have to manually update the kinect device to use the libfreenect drivers
// and/or uninstall/reinstall it in Device Manager.
//
// No way around the Windows driver dance, sorry.

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class ofApp : public ofBaseApp, public ofxKinectBlobListener {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);

    void blobOn( ofVec3f centroid, int id, int order );
    void blobMoved( ofVec3f centroid, int id, int order);
    void blobOff( ofVec3f centroid, int id, int order );

    // brightness filter borrowed from cuinjune https://forum.openframeworks.cc/t/ofimage-setbrightness-setcontrast/26463
    // notes - for use on the point cloud when colour is taken from the frame differencing ofImage to prevent completely black points
    void setBrightness(ofImage &image, const int brightness);
	
	ofxKinect kinect;

    ofxKinectBlobFinder  blobFinder;
    ofxKinectBlobTracker blobTracker;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
    ofxCv::ContourFinder contourFinder;

    // 3D blobs
    ofxCvGrayscaleImage     bgImage;
    ofxCvGrayscaleImage 	grayDiff;
    // for KinectBlobTracker
    ofImage grayDiffOfImage;

    bool bDrawIDs;
    bool bLearnBakground;

    int timeLapse;
    ofTrueTypeFont font;

    int angle;
    int numPixels;

    int minBlobPoints;
    float minBlobVol;
    float maxBlobVol;

    ofVec3f cropBoxMin;
    ofVec3f cropBoxMax;
    ofVec3f thresh3D;
    int thresh2D;
    unsigned int maxBlobs;
    //

    ofImage scaleIR;
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	
	int nearThreshold;
	int farThreshold;

    int kinectRes;

    float add;

    //ofxCv
    Mat frameMat, diffMat, accumMat, accumMatScaled;
    ofImage kinectDim;
    ofPixels previous;
    ofImage diff;
    ofImage brightDiff;
    ofImage kinectImage;
    Scalar diffMean;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;

    int frames;
    int distMap;

    int sum;

    ofColor targetColor;


    ofParameter<float> threshold;
    ofParameter<bool> trackHs;
    ofParameter<bool> holes;
};
