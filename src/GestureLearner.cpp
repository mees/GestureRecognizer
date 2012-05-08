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

#include "GestureLearner.h"

using namespace std;
// kdTreeManager has no standard constructor, so it should be initialized with its path as argument.
GestureLearner::GestureLearner() :
	kdTreeManager("Training/"), wronglyClassifiedListFileName(
			"wrongly_classified_list.txt") {
	loadWronglyClassifiedList();
	bootStrapCounter = 0;
	recognizedWrong = 0;
	recognizedCorrect = 0;
}

// kdTreeManager has no standard constructor, so we use the standard copy-constructor to initialize it.
GestureLearner::GestureLearner(KdTreeManager & kdTreeManager) :
	kdTreeManager(kdTreeManager), wronglyClassifiedListFileName(
			"wrongly_classified_list.txt") {
	loadWronglyClassifiedList();
	recognizedWrong = 0;
	recognizedCorrect = 0;
}

GestureLearner::~GestureLearner() {
	// TODO Auto-generated destructor stub
}

void GestureLearner::saveHistogramAndReloadTrainingData(const pcl::PointCloud<
		pcl::PointXYZ> & pointCloud, KdTreeManager::GestureType correctGesture) {
	kdTreeManager.saveHistogram(pointCloud, correctGesture);
	kdTreeManager.buildTrainingDataMain();
	kdTreeManager.loadTrainingFiles();
}

void GestureLearner::processPointCloud(
		const pcl::PointCloud<pcl::PointXYZ> &pointCloud,
		KdTreeManager::GestureType correctGesture) {
	set<KdTreeManager::Gesture, KdTreeManager::compareGesture> nearestGestures =
			kdTreeManager.getKNearestGestures(pointCloud, 2);
	KdTreeManager::GestureType recognizedGesture;
	if (!nearestGestures.empty()) {
		recognizedGesture = nearestGestures.begin()->gestureType;
	} else {
		recognizedGesture = KdTreeManager::INVALID;
		return;
	}
	if (recognizedGesture != correctGesture) {
		if (!nearestGestures.empty()) {
			addIncorrectHistogramFileToList(nearestGestures.begin()->fileName);
		}
		recognizedWrong++;
		saveHistogramAndReloadTrainingData(pointCloud, correctGesture);
	} else {
		recognizedCorrect++;
		if (nearestGestures.size() >= 2) {
			if (correctGesture != (++nearestGestures.begin())->gestureType) {
				saveHistogramAndReloadTrainingData(pointCloud, correctGesture);
				cout
						<< "Second nearest gesture was different, so the histogram was saved"
						<< endl;
			}
		}
	}
	cout << "Success rate: " << calculateRecognitionSuccessRate() << endl;
}

void GestureLearner::processPointCloud(
		const pcl::PointCloud<pcl::PointXYZ> &pointCloud) {
	cout << "Which gesture are you doing?" << endl;
	cout << "1 : Scissor" << endl << "2 : Rock" << endl << "3 : Paper" << endl
			<< "0 : Invalid" << endl;
	int option;
	string input = "";
	getline(cin, input);
	istringstream inputstream(input);
	inputstream >> option;
	KdTreeManager::GestureType gestureType;
	switch (option) {
	case 1:
		gestureType = KdTreeManager::SCISSOR;
		break;
	case 2:
		gestureType = KdTreeManager::ROCK;
		break;
	case 3:
		gestureType = KdTreeManager::PAPER;
		break;
	default:
		gestureType = KdTreeManager::INVALID;
		break;
	}
	if (gestureType != KdTreeManager::INVALID) {
		processPointCloud(pointCloud, gestureType);
	}
}

void GestureLearner::bootStrapping(
		const pcl::PointCloud<pcl::PointXYZ> &pointCloud) {
	if (bootStrapCounter <= 0) {
		cout << "Which gesture do you want do learn?" << endl;
		cout << "1 : Scissor" << endl << "2 : Rock" << endl << "3 : Paper"
				<< endl << "0 : Restart bootstrapping" << endl;
		int option;
		string input = "";
		getline(cin, input);
		istringstream inputstream(input);
		inputstream >> option;
		bootStrapCounter = 30;
		switch (option) {
		case 1:
			bootStrapGestureType = KdTreeManager::SCISSOR;
			break;
		case 2:
			bootStrapGestureType = KdTreeManager::ROCK;
			break;
		case 3:
			bootStrapGestureType = KdTreeManager::PAPER;
			break;
		default:
			bootStrapGestureType = KdTreeManager::INVALID;
			bootStrapCounter = 0;
			break;
		}
	}
	if (bootStrapGestureType != KdTreeManager::INVALID && bootStrapCounter > 0) {
		processPointCloud(pointCloud, bootStrapGestureType);
		bootStrapCounter--;
	}
}

float GestureLearner::calculateRecognitionSuccessRate() {
	return (float) recognizedCorrect / ((float) recognizedCorrect
			+ (float) recognizedWrong);
}

void GestureLearner::saveUpdatedListToFile() {
	ofstream fs;
	fs.open(wronglyClassifiedListFileName.c_str());
	set<string>::iterator it;
	for (it = wronglyClassifiedSet.begin(); it != wronglyClassifiedSet.end(); ++it)
		fs << *it << "\n";
	fs.close();
}

bool GestureLearner::loadWronglyClassifiedList() {
	if (!boost::filesystem::exists(wronglyClassifiedListFileName)) {
		cout << "Could not find " + wronglyClassifiedListFileName
				+ "! Creating empty one" << endl;
		ofstream fs;
		fs.open(wronglyClassifiedListFileName.c_str());
		fs << "\n";
		fs.close();
	} else {
		ifstream fs;
		fs.open(wronglyClassifiedListFileName.c_str());
		if (!fs.is_open() || fs.fail())
			return (false);

		string line;
		while (!fs.eof()) {
			getline(fs, line);
			if (line.empty())
				continue;
			if (boost::filesystem::exists(line)) {
				wronglyClassifiedSet.insert(line);
			}
		}
		fs.close();
	}
	return (true);
}

void GestureLearner::addIncorrectHistogramFileToList(
		const std::string &fileName) {
	if (wronglyClassifiedSet.erase(fileName))//the histogram was already stored in the file, so since it was incorrectly classified twice, we delete it
	{
		kdTreeManager.deleteHistogram(fileName);
	} else { //this histogram was not in the file, so we add it
		wronglyClassifiedSet.insert(fileName);
		cout << "this histogram was not in the file, so we add it" << endl;
	}
	saveUpdatedListToFile();
}

