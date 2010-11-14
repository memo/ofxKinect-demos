/*
 Kinect demo for openFrameworks
 Copyright (c) 2010 Memo Akten http://www.memo.tv
 
 
 Using ofxKinect by Theo Watson http://www.theowatson.com/
 
 
 based on libfreenect - an open source Kinect driver
 
 Copyright (C) 2010  Hector Martin "marcan" <hector@marcansoft.com>
 
 This code is licensed to you under the terms of the GNU GPL, version 2 or version 3;
 see:
 http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 http://www.gnu.org/licenses/gpl-3.0.txt
 */


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
