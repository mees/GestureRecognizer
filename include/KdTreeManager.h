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

#ifndef KDTREEMANAGER_H_
#define KDTREEMANAGER_H_
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <time.h>
#include <boost/filesystem.hpp>
#include <flann/io/hdf5.h>
#include <fstream>

// Important: there should not be more than one KdTreeManager with the same path at the same time,
// because otherwise the second one will overwrite some files the first one generates.
class KdTreeManager {
	typedef std::pair<std::string, std::vector<float> > vfh_model;

public:
	enum GestureType {
		SCISSOR = 0, ROCK = 1, PAPER = 2,

		INVALID = 42
	};

	struct Gesture {
		std::string fileName;
		GestureType gestureType;
		float distance;
	};

	struct compareGesture {
		bool operator()(const Gesture& a, const Gesture& b) const {
			return a.distance < b.distance;
		}
	};

public:
	// KdTreeManager has no standard constructor, as a kdTreeManager without a path doesn't make much sense
	// to initialize it as member in another class write for example
	KdTreeManager(std::string path);
	virtual ~KdTreeManager();
	KdTreeManager::GestureType getNearestGestureType(const pcl::PointCloud<
			pcl::PointXYZ> &pointCloud);

	// returns the k nearest gestures. If there are less than k gestures it returns as many as possible.
	std::set<KdTreeManager::Gesture, KdTreeManager::compareGesture>
			getKNearestGestures(
					const pcl::PointCloud<pcl::PointXYZ> &pointCloud, int k);

	void buildTrainingDataMain();
	void loadTrainingFiles();

	std::string gestureTypeToString(KdTreeManager::GestureType gestureType);

	// saves the histogram as .pcd and the pointcloud as .txt file in this->path.
	// Filename contains gesturename and timestamp
	void saveHistogram(const pcl::PointCloud<pcl::PointXYZ> &pointCloud,
			KdTreeManager::GestureType gestureType);
	// the filename to be deleted without extension
	void deleteHistogram(const std::string &fileName);

private:
	const std::string path;
	std::vector<vfh_model> models;
	flann::Matrix<int> k_indices;
	flann::Matrix<float> k_distances;
	flann::Matrix<float> data;
	const std::string kdtree_idx_file_name;
	const std::string training_data_h5_file_name;
	const std::string training_data_list_file_name;

	KdTreeManager::GestureType getGestureTypeFromFileName(
			const std::string &fileName);

	// saves the histogram as .pcd and the pointcloud as .txt file.
	void saveHistogram(const pcl::PointCloud<pcl::PointXYZ> &pointCloud,
			std::string fileName);
	void calcHistogramFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs);
	std::string getTimeStamp();
	bool loadHist(const boost::filesystem::path &path, vfh_model &vfh);

	bool loadFileList(std::vector<vfh_model> &models,
			const std::string &fileName);

	void loadFeatureModels(const boost::filesystem::path &base_dir,
			const std::string &extension, std::vector<vfh_model> &models);
	inline void nearestKSearch(
			flann::Index<flann::ChiSquareDistance<float> > &index,
			const vfh_model &model, int &k, flann::Matrix<int> &indices,
			flann::Matrix<float> &distances);
};

#endif /* KDTREEMANAGER_H_ */
