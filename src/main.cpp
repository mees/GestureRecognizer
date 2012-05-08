/*
* Copyright (c) 2012, Richard Hertel, Oier Mees
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright
*      notice, this list of conditions and the following disclaimer in the
*      documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdlib.h>

#include "GestureLearner.h"
#include "RPSGame.h"

using namespace std;
using namespace pcl;

/// Focal Length of kinect X Box 360 camera
int focalLength = 525;
cv::Mat OldKinectImg = cv::Mat(480, 640, CV_8UC1);
int movementCounter = 0;
int pointCounterPointer = 0;
// Program mode: 1: learning mode   2 : playing mode
int mode;

KdTreeManager kdTreeManager("Training/");
GestureLearner gestureLearner(kdTreeManager);
RPSGame rpsGame(kdTreeManager);

void calcUVPointFromXYZ(Eigen::Vector3f Vec, cv::Point &UVout) {
	UVout.x = (((focalLength * Vec.x()) / Vec.z())) + 320;
	UVout.y = (-1 * ((-((focalLength * Vec.y()) / Vec.z())) - 240));
}

// count the White Pixels in an greyscale Image
int countWhitePixels(cv::Mat inputImg) {
	int result = 0;
	for (int i = 0; i < inputImg.size().height * inputImg.size().width; i++) {
		if (inputImg.data[i] == 255)
			result = result + 1;
	}
	return result;
}

void KinectPCCallback(const sensor_msgs::PointCloud2 & PC2Msg) {

	// In the Local Pointcloud our actual Handcloud will be saved
	pcl::PointCloud<pcl::PointXYZ> LocalPointCloud;
	pcl::fromROSMsg(PC2Msg, LocalPointCloud); // The message (~= binary blob) received needs to be converted to PC.

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(LocalPointCloud.makeShared());
	// output datasets
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<
			pcl::VFHSignature308>());

	cv::Mat KinectImg = cv::Mat(480, 640, CV_8UC1);
	memset((void *) KinectImg.data, 0, 640 * 480 * sizeof(uchar)); // Initialize the window image to zero=black
	cv::Point UVCoords;
	Eigen::Vector3f PointCoords;

	//iterate through the Handcloud and make the corresponding points in the Image white.
	for (size_t j = 0; j < LocalPointCloud.points.size(); ++j) {
		PointCoords << LocalPointCloud.points[j].x, LocalPointCloud.points[j].y, LocalPointCloud.points[j].z;
		// calculates the corresponding pixel in the Image
		calcUVPointFromXYZ(PointCoords, UVCoords);
		cv::circle(KinectImg, UVCoords, 2, cv::Scalar(255));
	}

	cv::imshow("Kinect", KinectImg);
	cvWaitKey(100);

	// saves the difference between two sequential frames
	cv::Mat Difference = cv::Mat(480, 640, CV_8UC1);
	cv::bitwise_xor(KinectImg, OldKinectImg, Difference);
	cv::imshow("Difference", Difference);
	int diffcounter = countWhitePixels(Difference);

	// if the two frames are more or less equal, a counter is increased, so we can see how long the hand didn't move.
	if (diffcounter < 900)
		movementCounter++;
	else
		movementCounter = 0;

	// Save old image for movement detection
	KinectImg.copyTo(OldKinectImg);

	//if the last frames have been more or less equal, then classify gesture

	switch (mode) {
	case 1:
		if (movementCounter >= 4) {
			gestureLearner.processPointCloud(LocalPointCloud);
			movementCounter = -5;
		}
		break;
	case 2:
		if (movementCounter >= 4) {
			rpsGame.processPointCloud(LocalPointCloud);
			movementCounter = -10;
		}
		break;
	case 3:
		if (movementCounter >= 2) {
			gestureLearner.bootStrapping(LocalPointCloud);
			movementCounter = 0;
		}
		break;
	default:
		if (movementCounter >= 4) {
			rpsGame.processPointCloud(LocalPointCloud);
			movementCounter = -10;
		}
		break;
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "RPSProcess");
	ros::NodeHandle nh;
	string input = "";
	cout << " Which mode do you want to start?" << endl << "1 : Learning mode"
			<< endl << "2 : Playing mode" << endl << "3 : Bootstrapping mode"
			<< endl;
	getline(cin, input);
	istringstream inputstream(input);
	inputstream >> mode;
	cout
			<< "Please stand back and raise your hands in order to track your skeleton"
			<< endl;

	ros::Subscriber PointCloud2Sub = nh.subscribe("/hand1_fullcloud", 1,
			KinectPCCallback);
	ros::spin();
	return 0;
}
