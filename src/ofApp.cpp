#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

/*
    If you are struggling to get the device to connect ( especially Windows Users )
    please look at the ReadMe: in addons/ofxKinect/README.md
*/

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #

    //Turn LED off
    kinect.setLed(ofxKinect::LED_OFF);

    // GL point size for kinect, increase if experiencing low framerate
    //kinectRes = 1;
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

    // 3d blobs //
    blobFinder.init(&kinect, false); // standarized coordinate system: z in the direction of gravity
    blobFinder.setResolution(BF_LOW_RES);
    blobFinder.setRotation( ofVec3f( angle, 0, 0) );
    blobFinder.setTranslation(ofVec3f(0,0,0));
    blobFinder.setScale(ofVec3f(0.001, 0.001, 0.001)); // mm to meters
    // bind our kinect to the blob finder
    // in order to do this we need to declare in testApp.h: class testApp : public ofBaseApp, public ofxKinectBlobListener
    blobTracker.setListener( this );

    bgImage.allocate(kinect.width, kinect.height);
    grayDiff.allocate(kinect.width, kinect.height);
    grayDiffOfImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);

    numPixels = kinect.width*kinect.height;

    // NOTE: measurement units in meters!!!
    minBlobVol = 0.2f;
    maxBlobVol = 2.0f;
    //no cropping
    cropBoxMin = ofVec3f(-10, -10, -10);
    cropBoxMax = ofVec3f(10, 10, 10);
    //
    thresh3D = ofVec3f(0.2,0.2,0.3);
    // xy pixel search range
    thresh2D = 1;
    maxBlobs = 5;

    float sqrResolution = blobFinder.getResolution();
    sqrResolution *= sqrResolution;
    minBlobPoints = (int)(0.001*(float)numPixels/sqrResolution);

    //printf("min %f\n", minBlobVol);

    bLearnBakground = true;
    timeLapse = 0;
    font.loadFont("PerfectDOSVGA437.ttf",15, true, true, true);
    bDrawIDs = false;
    // 3d blobs //
	
    nearThreshold = 255;
    farThreshold = 0;
    bThreshWithOpenCV = false;

    //set size of frame diff images (from ofxCv 'example-difference')
    imitate(previous, kinect);
    imitate(diff, kinect);
	
    ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
    bDrawPointCloud = true;

    frames = 0;

    contourFinder.setMinAreaRadius(10);
    contourFinder.setMaxAreaRadius(150);
}

//--------------------------------------------------------------
void ofApp::update() {

    frames++;
	
    ofBackground(0);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

        contourFinder.setTargetColor(255);
        contourFinder.setThreshold(threshold);
        contourFinder.setFindHoles(holes);
        contourFinder.findContours(kinect);

        // distance stuff
        int dist = kinect.getDistanceAt(320,240);

//      distMap = ofMap(dist, 550, 4000, 120, 0);
        distMap = -dist;

//        add = add + .1;
//        angle = sin(ofDegToRad(add)) * 30;
//        kinect.setCameraTiltAngle(angle);

//        if (dist == 0 && angle > 20){
//            angle--;
//            kinect.setCameraTiltAngle(angle);
//        }
//        else if (dist == 0 && angle < -20) {

//        }
//        else if (dist > 1 && dist < 750 && angle < 20) {
//            angle++;
//            kinect.setCameraTiltAngle(angle);
//        }
//        else {
//            angle = 0;
//            kinect.setCameraTiltAngle(angle);
//        }

        //std::cout << "dist: " << dist << endl;
        //std::cout << "distMap: " << distMap << endl;
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels());

		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			ofPixels & pix = grayImage.getPixels();
			int numPixels = pix.size();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;                    

				} else {
                    pix[i] = 0;                 
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
        //contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);

        // adapted from ofxCv 'example-difference'
        frames++;

        frameMat = toCv(kinect.getPixels());

        if(accumMat.empty()) {

            frameMat.convertTo(accumMat, CV_32F);
        }
        accumulateWeighted(frameMat, accumMat, 0.01);
        convertScaleAbs(accumMat, accumMatScaled);
        absdiff(frameMat, accumMatScaled, diff);

        diff.update();

        // for frameDIff without accum weight
        //copy(kinect, previous);

        //kinect ofImage
        kinectImage.setFromPixels(kinect.getPixels());

        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels().getData(), kinect.width, kinect.height);
        // background subtraction
        if ((bLearnBakground) && (ofGetElapsedTimeMillis() >= timeLapse)) {
            bgImage = grayImage;   // let this frame be the background image from now on
            bLearnBakground = false;
        }
        cvAbsDiff(bgImage.getCvImage(), grayImage.getCvImage(), grayDiff.getCvImage());
        cvErode(grayDiff.getCvImage(), grayDiff.getCvImage(), NULL, 2);
        cvDilate(grayDiff.getCvImage(), grayDiff.getCvImage(), NULL, 1);
        // threshold ignoring little differences
        cvThreshold(grayDiff.getCvImage(), grayDiff.getCvImage(), 3, 255, CV_THRESH_BINARY);
        grayDiff.flagImageChanged();
        // update the ofImage to be used as background mask for the blob finder
        grayDiffOfImage.setFromPixels(grayDiff.getPixels().getData(), kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);

        blobFinder.findBlobs( &grayDiffOfImage,
                           cropBoxMin, cropBoxMax,
                           thresh3D, thresh2D,
                           minBlobVol, maxBlobVol, minBlobPoints,  maxBlobs);
        blobTracker.trackBlobs( blobFinder.blobs );
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void ofApp::draw() {

    // draw 2d diff image affected by dist
    //ofSetColor(255, 255, 255, 140 - distMap);
    //diff.draw(0, 0, 1920, 1440);

	if(bDrawPointCloud) {
		easyCam.begin();

		drawPointCloud();

        ofPushMatrix();
        ofPushStyle();
        //ofRotateZ(180);
        ofScale(2,-2, 1);

        ofSetLineWidth(1.0);
        ofSetColor(255, 0, 0, 127);
        ofTranslate(-391, -200, 10);

        contourFinder.draw();

        int n = contourFinder.size();
        for(int i = 0; i < n; i++) {
            // smallest rectangle that fits the contour

            ofSetColor(0,0,0,0);
            ofPolyline minAreaRect = toOf(contourFinder.getMinAreaRect(i));
            minAreaRect.draw();
        }

        ofPopMatrix();
        ofPopStyle();
		easyCam.end();

	} else {
		// draw from the live kinect
        kinect.drawDepth(10, 10, 400, 300);
		kinect.draw(420, 10, 400, 300);
		
        grayImage.draw(10, 320, 400, 300);
        //contourFinder.draw(10, 320, 400, 300);
		
#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}   
}

void ofApp::drawPointCloud() {

    //make a copy to make a brighter version
    ofImage brightDiff = diff;

    setBrightness(brightDiff, 30-angle);

    ofEnableAlphaBlending();
    ofEnableDepthTest();

    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 2;
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(kinect.getDistanceAt(x, y) > 0) {
                //ofColor mixClr(kinect.getColorAt(x,y), 127);
                ofColor mixClr(diff.getColor(x,y), 127);
                //kinect ir image is not synched with mesh, this is an approx conversion
                //ofColor mixClr(diff.getColor((x*1.1)-10,(y*1.1)-45), 127);
                mesh.addColor(mixClr);
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
            }
        }
    }

    ofPushMatrix();
    ofScale(1000.0,-1000.0,-1000.0);
    //ofScale(1, -1, -1);
    //ofTranslate(0, 0, -10); // center the points a bit

    glPointSize(2);
    // draw blobs
    for (unsigned int i=0; i < blobFinder.blobs.size(); i++) {
        //ofSetLineWidth(4.0);

        //ofSetColor(255,0,0,127);
        //ofSetColor(25*i,25*i,255-25*i, 100);
        // draw blobs

        ofFill;
        blobFinder.blobs[i].draw();
        // plot blobs IDs
        if (bDrawIDs) {
            ofPushMatrix();
            ofTranslate(blobTracker.blobs[i].massCenter.x, blobTracker.blobs[i].massCenter.y, blobTracker.blobs[i].maxZ.z);
            ofRotateX(-90);
            ofScale(0.01f, 0.01f, 0.01f);
            ofSetColor(255,255,255);
            font.drawStringAsShapes(ofToString(blobTracker.blobs[i].id), 0, 0);
            ofPopMatrix();
        }


        // draw trajectory as a line
        vector <ofVec3f> trajectory;
        blobTracker.getTrajectoryById(blobTracker.blobs[i].id, trajectory);
        unsigned int trjSize = trajectory.size();
        if (trjSize > 1) {
            ofPushMatrix();
            ofSetColor(255,255,0);
            ofSetLineWidth(3);
            glBegin(GL_LINE);
            for (unsigned int j = 0; j < trjSize; j++) {
                glVertex3f( trajectory[j].x, trajectory[j].y, trajectory[j].z );
            }
            glEnd();
            ofPopMatrix();
            trajectory.clear();
        }
    }


    ofPopMatrix();

    //ocd
    ofPushMatrix();
    glPointSize(2);
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    //ofTranslate(0, 0, -1000); // center the points a bit

    mesh.drawVertices();

    ofPopMatrix();
    ofDisableDepthTest();
}

// ofImage brightness effect borrowed from https://forum.openframeworks.cc/t/ofimage-setbrightness-setcontrast/26463
void ofApp::setBrightness(ofImage &image, const int brightness) {

    int numChannels;

    switch (image.getImageType()) {

        case OF_IMAGE_GRAYSCALE:
            numChannels = 1;
            break;
        case OF_IMAGE_COLOR:
            numChannels = 3;
            break;
        case OF_IMAGE_COLOR_ALPHA:
            numChannels = 4;
            break;
        default:
            break;
    }
    ofPixels &pix = image.getPixels();
    const size_t pixSize = static_cast<size_t>(image.getWidth() * image.getHeight() * numChannels);

    if (numChannels == 1) {

        for (size_t i=0; i<pixSize; ++i) {

            const int g = pix[i] + brightness;
            pix[i] = static_cast<unsigned char>(g < 0 ? 0 : g > 255 ? 255 : g);
        }
    }
    else {

        for (size_t i=0; i<pixSize; i+=numChannels) {

            const int r = pix[i] + brightness;
            const int g = pix[i+1] + brightness;
            const int b = pix[i+2] + brightness;
            pix[i] = static_cast<unsigned char>(r < 0 ? 0 : r > 255 ? 255 : r);
            pix[i+1] = static_cast<unsigned char>(g < 0 ? 0 : g > 255 ? 255 : g);
            pix[i+2] = static_cast<unsigned char>(b < 0 ? 0 : b > 255 ? 255 : b);
        }
    }
    image.update();
}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
	
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
    //targetColor = kinect.getPixels().getColor(x, y);
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{

}

/*
 *
 *	blob section
 *
 *	from here on in it's blobs
 *	thanks to stefanix and the opencv library :)
 *
 */

//--------------------------------------------------
void ofApp::blobOn( ofVec3f centroid, int id, int order ) {
   // cout << "blobOn() - id:" << id << " order:" << order << endl;
}

void ofApp::blobMoved( ofVec3f centroid, int id, int order) {
  //  cout << "blobMoved() - id:" << id << " order:" << order << endl;
  // full access to blob object ( get a reference)
  //  ofxKinectTrackedBlob blob = blobTracker.getById( id );
  // cout << "volume: " << blob.volume << endl;
}

void ofApp::blobOff( ofVec3f centroid, int id, int order ) {
   // cout << "blobOff() - id:" << id << " order:" << order << endl;
}

