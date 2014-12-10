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

#include "KdTreeManager.h"

using namespace std;

KdTreeManager::KdTreeManager(std::string path) :
	path(path), kdtree_idx_file_name("kdtree.idx"), training_data_h5_file_name(
			"training_data.h5"), training_data_list_file_name(
			"training_data.list") {
	buildTrainingDataMain();
	loadTrainingFiles();
}

KdTreeManager::~KdTreeManager() {
	// TODO Auto-generated destructor stub
}

std::set<KdTreeManager::Gesture, KdTreeManager::compareGesture> KdTreeManager::getKNearestGestures(
		const pcl::PointCloud<pcl::PointXYZ> &pointCloud, int k) {

	flann::Index<flann::ChiSquareDistance<float> > index(data,
			flann::SavedIndexParams(kdtree_idx_file_name));
	index.buildIndex();
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<
			pcl::VFHSignature308>());
	calcHistogramFromCloud(pointCloud.makeShared(), vfhs);

	vfh_model histogram;
	histogram.first = "temp";
	histogram.second.resize(308);

	// 308 should maybe become a member variable
	for (size_t i = 0; i < 308; ++i) {
		histogram.second[i] = (*vfhs).points[0].histogram[i];
	}

	nearestKSearch(index, histogram, k, this->k_indices, this->k_distances);
	cout<<k<<endl;
	printf("The closest %d neighbors are:\n", k);
	for (int i = 0; i < k; ++i)
		printf("    %d - %s (%d) with a distance of: %f\n", i, this->models.at(
				this->k_indices[0][i]).first.c_str(), this->k_indices[0][i],
				this->k_distances[0][i]);

	set<KdTreeManager::Gesture, KdTreeManager::compareGesture> result;

	for (int i = 0; i < k; ++i) {
		Gesture gest;
		gest.distance = this->k_distances[0][i];
		string gesturePath = models.at(k_indices[0][i]).first.c_str();
		size_t pos;
		pos = gesturePath.find(this->path);
		string fileName = gesturePath.substr(pos + path.length());
		gest.fileName = fileName.substr(0, fileName.length() - 4);
		gest.gestureType = getGestureTypeFromFileName(gest.fileName);
		result.insert(gest);
	}

	return result;
}

KdTreeManager::GestureType KdTreeManager::getNearestGestureType(
		const pcl::PointCloud<pcl::PointXYZ> &pointCloud) {
	set<KdTreeManager::Gesture, KdTreeManager::compareGesture>
			nearestGestureSet = getKNearestGestures(pointCloud, 1);
	if (!nearestGestureSet.empty()) {
		return nearestGestureSet.begin()->gestureType;
	} else {
		return KdTreeManager::INVALID;
	}
}

KdTreeManager::GestureType KdTreeManager::getGestureTypeFromFileName(
		const std::string &fileName) {

	if (strcmp(fileName.substr(0, 2).c_str(), "Sc") == 0)
		return KdTreeManager::SCISSOR;

	if (strcmp(fileName.substr(0, 2).c_str(), "Ro") == 0)
		return KdTreeManager::ROCK;

	if (strcmp(fileName.substr(0, 2).c_str(), "Pa") == 0)
		return KdTreeManager::PAPER;

	return KdTreeManager::INVALID;
}

std::string KdTreeManager::gestureTypeToString(
		KdTreeManager::GestureType gestureType) {
	switch (gestureType) {
	case KdTreeManager::SCISSOR:
		return "Scissor";
		break;
	case KdTreeManager::ROCK:
		return "Rock";
		break;
	case KdTreeManager::PAPER:
		return "Paper";
		break;
	case KdTreeManager::INVALID:
		return "Invalid";
		break;
	default:
		return "Invalid";
	}
}

void KdTreeManager::saveHistogram(
		const pcl::PointCloud<pcl::PointXYZ> &pointCloud,
		KdTreeManager::GestureType gestureType) {
	if (gestureType != KdTreeManager::INVALID) {
		string timeStamp = getTimeStamp();
		saveHistogram(pointCloud, (this->path
				+ gestureTypeToString(gestureType) + "_" + timeStamp));
	}
}

void KdTreeManager::saveHistogram(
		const pcl::PointCloud<pcl::PointXYZ> &pointCloud, std::string fileName) {
	pcl::io::savePCDFileASCII(fileName + ".txt", pointCloud);
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<
			pcl::VFHSignature308>());
	calcHistogramFromCloud(pointCloud.makeShared(), vfhs);
	pcl::io::savePCDFile(fileName + ".pcd", *vfhs);
}

// Calculates the viewpoint feature histogram of the pointcloud
// cloud: Inputcloud, vfhs: Outputhistogram
void KdTreeManager::calcHistogramFromCloud(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<
				pcl::VFHSignature308>::Ptr vfhs) {
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(kdtree);
	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch(0.03);
	// Compute the features
	ne.compute(*normals);
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud(cloud);
	vfh.setInputNormals(normals);
	// Create an empty kdtree representation, and pass it to the FPFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>());
	vfh.setSearchMethod(tree2);
	// Compute the features
	vfh.compute(*vfhs);
}

std::string KdTreeManager::getTimeStamp() {
	char timeStamp[30];
	time_t now;
	now = time(NULL);
	if (now != -1) {
		strftime(timeStamp, 30, "%d.%m.%Y_%H:%M:%S", gmtime(&now));
	}
	return timeStamp;
}

bool KdTreeManager::loadHist(const boost::filesystem::path &path,
		vfh_model &vfh) {
	int vfh_idx;
	// Load the file as a PCD
	try {
		pcl::PCLPointCloud2 cloud;
		int version;
		Eigen::Vector4f origin;
		Eigen::Quaternionf orientation;
		pcl::PCDReader r;
		int type; unsigned int idx;
		r.readHeader(path.string(), cloud, origin, orientation, version, type, idx);

		vfh_idx = pcl::getFieldIndex(cloud, "vfh");
		if (vfh_idx == -1)
			return (false);
		if ((int) cloud.width * cloud.height != 1)
			return (false);
	} catch (pcl::InvalidConversionException e) {
		return (false);
	}

	// Treat the VFH signature as a single Point Cloud
	pcl::PointCloud<pcl::VFHSignature308> point;
	pcl::io::loadPCDFile(path.string(), point);
	vfh.second.resize(308);

	vector<pcl::PCLPointField> fields;
	pcl::getFieldIndex(point, "vfh", fields);

	for (size_t i = 0; i < fields[vfh_idx].count; ++i) {
		vfh.second[i] = point.points[0].histogram[i];
	}
	vfh.first = path.string();
	return (true);
}

void KdTreeManager::loadTrainingFiles() {
	// Check if the data has already been saved to disk
	if (!boost::filesystem::exists(training_data_h5_file_name)
			|| !boost::filesystem::exists(training_data_list_file_name)) {
		printf("Could not find training data models files %s and %s!\n",
				training_data_h5_file_name.c_str(),
				training_data_list_file_name.c_str());
	} else {
		loadFileList(models, training_data_list_file_name);
		flann::load_from_file(data, training_data_h5_file_name, "training_data");
		printf("Training data found. Loaded %d VFH models from %s/%s.\n",
				(int) data.rows, training_data_h5_file_name.c_str(),
				training_data_list_file_name.c_str());
	}

	// Check if the tree index has already been saved to disk
	if (!boost::filesystem::exists(kdtree_idx_file_name)) {
		printf("Could not find kd-tree index in file %s!",
				kdtree_idx_file_name.c_str());
	} else {
		cout << "all files loaded correctly" << endl;
	}
}

/** \brief Load the list of file model names from an ASCII file
 * \param models the resultant list of model name
 * \param filename the input file name
 */
bool KdTreeManager::loadFileList(std::vector<vfh_model> &models,
		const std::string &filename) {
	models.clear();
	ifstream fs;
	fs.open(filename.c_str());
	if (!fs.is_open() || fs.fail())
		return (false);

	string line;
	while (!fs.eof()) {
		getline(fs, line);
		if (line.empty())
			continue;
		vfh_model m;
		m.first = line;
		models.push_back(m);
	}
	fs.close();
	return (true);
}

void KdTreeManager::deleteHistogram(const std::string &filename) {
	if (remove((this->path + filename + ".pcd").c_str()) != 0)
		perror("Error deleting pcd file");
	if (remove((this->path + filename + ".txt").c_str()) != 0)
		perror("Error deleting txt file");
	cout << "Deleted following histogram: " << filename << endl;
}

void KdTreeManager::buildTrainingDataMain() {
	string extension(".pcd");
	transform(extension.begin(), extension.end(), extension.begin(), (int(*)(
			int)) tolower);
	vector<vfh_model> models;

	// Load the model histograms
	loadFeatureModels(this->path, extension, models);
	printf("Loaded %d VFH models. Creating training data %s/%s.\n",
			(int) models.size(), training_data_h5_file_name.c_str(),
			training_data_list_file_name.c_str());
	//check if the Training folder is empty
	if (models.size() == 0) {
		printf(
				"The Training folder is empty, please create some training files\n");
		return;
	}
	// Convert data into FLANN format
	//data.rows = models.size();
	//data.cols = models[0].second.size(); // number of histogram bins
	//data.ptr() = (float*) malloc(data.rows * data.cols * sizeof(float));
	float *rawdata = new float[models.size() * models[0].second.size()];
	flann::Matrix<float> data(rawdata, models.size(), models[0].second.size());

	for (size_t i = 0; i < data.rows; ++i)
		for (size_t j = 0; j < data.cols; ++j)
			rawdata[i * data.cols + j] = models[i].second[j];

	// Save data to disk (list of models)
	save_to_file(data, training_data_h5_file_name, "training_data");
	ofstream fs;
	fs.open(training_data_list_file_name.c_str());
	for (size_t i = 0; i < models.size(); ++i)
		fs << models[i].first << "\n";
	fs.close();

	// Build the tree index and save it to disk
	printf("Building the kdtree index (%s) for %d elements...\n",
			kdtree_idx_file_name.c_str(), (int) data.rows);
	flann::Index<flann::ChiSquareDistance<float> > index(data,
			flann::LinearIndexParams());
	index.buildIndex();
	index.save(kdtree_idx_file_name);
	cout << "finished building tree" << endl;
}

/** \brief Load a set of VFH features that will act as the model (training data)
 * \param base_dir the path of the directory
 * \param extension the file extension containing the VFH features
 * \param models the resultant vector of histogram models
 */
void KdTreeManager::loadFeatureModels(const boost::filesystem::path &base_dir,
		const std::string &extension, std::vector<vfh_model> &models) {
	if (!boost::filesystem::exists(base_dir)
			&& !boost::filesystem::is_directory(base_dir)) {
		boost::filesystem::create_directory("Training");
		return;
	}

	//directory_iterator traverses through the files and directories
	for (boost::filesystem::directory_iterator it(base_dir); it
			!= boost::filesystem::directory_iterator(); ++it) {
		//Checks if the iterator points to a directory. In this case shows information about the progress and
		//make a recursive call to the subdirectory
		if (boost::filesystem::is_directory(it->status())) {
			stringstream ss;
			ss << it->path();
			printf("Loading %s .\n", ss.str().c_str());
			loadFeatureModels(it->path(), extension, models);
			cout << models.size() << " models loaded" << endl;
		}
		//If its a file with proper Extension, load it
		if (boost::filesystem::is_regular_file(it->status())
				&& boost::filesystem::extension(it->path()) == extension) {
			vfh_model m;
			if (loadHist(base_dir / it->path().filename(), m))
				models.push_back(m);
		}
	}
}

/** \brief Search for the closest k neighbors
 * \param index the tree
 * \param model the query model
 * \param k the number of neighbors to search for
 * \param indices the resultant neighbor indices
 * \param distances the resultant neighbor distances
 */
inline void KdTreeManager::nearestKSearch(flann::Index<
		flann::ChiSquareDistance<float> > &index, const vfh_model &model,
		int &k, flann::Matrix<int> &indices, flann::Matrix<float> &distances) {
	// Query point
	flann::Matrix<float> p = flann::Matrix<float>(
			new float[model.second.size()], 1, model.second.size());

	memcpy(p.ptr(), &model.second[0], p.cols * p.rows * sizeof(float));

	indices = flann::Matrix<int>(new int[k], 1, k);
	distances = flann::Matrix<float>(new float[k], 1, k);

	if (models.size() < (uint) k) {
		k = models.size();
	}
	if (k != 0) {
		index.knnSearch(p, indices, distances, k, flann::SearchParams(512));
	}
	delete[] p.ptr();
}
