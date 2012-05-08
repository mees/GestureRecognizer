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

#ifndef GESTURELEARNER_H_
#define GESTURELEARNER_H_
#include "KdTreeManager.h"

class GestureLearner {
public:
	GestureLearner();
	GestureLearner(KdTreeManager & kdTreeManager);
	virtual ~GestureLearner();
	// Takes a pointcloud and the corresponding gesturetype and decides if the gesture should be learned
	// and if another gesture should be unlearned
	void processPointCloud(const pcl::PointCloud<pcl::PointXYZ> &pointCloud, KdTreeManager::GestureType correctGesture);
	void processPointCloud(const pcl::PointCloud<pcl::PointXYZ> &pointCloud);
	void bootStrapping(const pcl::PointCloud<pcl::PointXYZ> &pointCloud);
	float calculateRecognitionSuccessRate();

private:
	KdTreeManager::GestureType bootStrapGestureType;
	int bootStrapCounter;
	int recognizedCorrect;
	int recognizedWrong;
	KdTreeManager kdTreeManager;
	std::set<std::string> wronglyClassifiedSet;
	const std::string wronglyClassifiedListFileName;
	void saveHistogramAndReloadTrainingData(const pcl::PointCloud<pcl::PointXYZ> & pointCloud, KdTreeManager::GestureType correctGesture);
    void addIncorrectHistogramFileToList(const std::string &fileName);
	void saveUpdatedListToFile();
	bool loadWronglyClassifiedList();
};

#endif /* GESTURELEARNER_H_ */
