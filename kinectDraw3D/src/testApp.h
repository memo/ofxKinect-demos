
#ifndef _TEST_APP
#define _TEST_APP

#include "ofMain.h"
#include "ofxKinect.h"

class testApp : public ofBaseApp
{

	public:

		void setup();
		void update();
		void draw();
        void exit();

		void keyPressed  (int key);

		ofxKinect kinect;
};

#endif
