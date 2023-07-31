/*********************************************************************************

					  Point Cloud Denoising Object

						Programing in 2020/08/11

						   By Dr. Chenlei Lv

			The functions includes:
			1. Add Gaussian noise into a point cloud;
			2. Deoising by Gaussian kernel function for a point cloud;
			2. Deoising by Fast BilateralFilter function for a point cloud;

*********************************************************************************/

#pragma once
#include <iostream> 
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h> 
#include <pcl/point_cloud.h>
#include <boost/random.hpp>
#include <pcl/console/time.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/bilateral.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointXYZI PointT;

using namespace std;

class DenoisingPD {

public:

	vector<Point3f> pointCloudOriginal;//point cloud
	vector<Point3f> pointCloudONormal;//normal vectors	
	vector<Point3f> pCDenosing;//remove noise from a point cloud
	vector<Point3f> pCDenosingNormal;//new normal
	double K = 12;//K-neighbor
	double segma;
	double radius;//searching K-neighbor radius
	int iter_Global = 10;//point update iter
	string fileName;
	vector<int> pointOutlierLabel;

public:

	void DenoisingPD_init(vector<Point3f> p, vector<Point3f> n, string fileNameStore) {

		fileName = fileNameStore;
		cout << "Denoising init start:" << endl;
		pointCloudOriginal = p;
		pointCloudONormal = n;
		cout << "Searching the radius:" << endl;
		DenoisingPD_SearchingRadiusByKneighboor();

		//label outlier
		pointOutlierLabel.resize(p.size(), -1);

		//initial remove outlier
		DenoisingPD_SearchingOutlierByKneighboor();

		cout << "Denoising init finished:" << endl;
	}	

	void DenoisingPD_AddNoise(double maxDis) {
		cout << "Add noising start:" << endl;
		boost::mt19937 rng;
		rng.seed(static_cast<unsigned int>(time(0)));
		boost::normal_distribution<> nd(0, radius * maxDis);
		boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> var_nor(rng, nd);
		for (int i = 0; i < pointCloudOriginal.size(); i++) {
			double randomi = static_cast<float> (var_nor());
			pointCloudOriginal[i][0] = pointCloudOriginal[i][0] + randomi * pointCloudONormal[i][0];
			pointCloudOriginal[i][1] = pointCloudOriginal[i][1] + randomi * pointCloudONormal[i][1];
			pointCloudOriginal[i][2] = pointCloudOriginal[i][2] + randomi * pointCloudONormal[i][2];
		}
		DenoisingPD_SavePLY_Point(pointCloudOriginal, fileName);
		cout << "Add noising finished!" << endl;
	}

	void DenoisingPD_AddOutlier(vector<bool> outlier, int lower = 3, int higher = 4) {

		vector<Point3f> pointCloudOriginal_store = pointCloudOriginal;

		cout << "Add Outlier start:" << endl;
		boost::mt19937 rng;
		rng.seed(static_cast<unsigned int>(time(0)));
		boost::normal_distribution<> nd(radius * lower, radius * higher);
		boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> var_nor(rng, nd);
		for (int i = 0; i < outlier.size(); i++) {
			if (!outlier[i]) {
				continue;
			}
			else {
				double randomi = static_cast<float> (var_nor());
				pointCloudOriginal[i][0] = pointCloudOriginal[i][0] + randomi * pointCloudONormal[i][0];
				pointCloudOriginal[i][1] = pointCloudOriginal[i][1] + randomi * pointCloudONormal[i][1];
				pointCloudOriginal[i][2] = pointCloudOriginal[i][2] + randomi * pointCloudONormal[i][2];
			}
		}	
		
		DenoisingPD_SavePLY_Point(pointCloudOriginal, fileName);
		cout << "Add noising finished!" << endl;

	}
	
private:

	void DenoisingPD_SearchingRadiusByKneighboor() {

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;//kdtree
		std::cout << "Init kdtree" << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = pointCloudOriginal.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		// fills a PointCloud with random data
		for (int i = 0; i < pointCloudOriginal.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = pointCloudOriginal[i][0];
			cloud->points[i].y = pointCloudOriginal[i][1];
			cloud->points[i].z = pointCloudOriginal[i][2];

		}
		kdtree.setInputCloud(cloud);

		double rMax = -1;

		int step = pointCloudOriginal.size() / 1000;
		if (step < 1) {
			for (int i = 0; i < pointCloudOriginal.size(); i++) {
				std::vector<int> pointIdxNKNSearch(K);
				std::vector<float> pointNKNSquaredDistance(K);
				std::vector<int> pointNeibor_i;
				pcl::PointXYZ searchPoint;
				searchPoint.x = pointCloudOriginal[i][0];
				searchPoint.y = pointCloudOriginal[i][1];
				searchPoint.z = pointCloudOriginal[i][2];
				kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
				double dis = sqrt(pointNKNSquaredDistance[pointNKNSquaredDistance.size() - 1]);
				if (dis > rMax) {
					rMax = dis;
				}
			}
		}
		else {
			for (int i = 0; i < step; i++) {
				int index = i * 1000;
				std::vector<int> pointIdxNKNSearch(K);
				std::vector<float> pointNKNSquaredDistance(K);
				std::vector<int> pointNeibor_i;
				pcl::PointXYZ searchPoint;
				searchPoint.x = pointCloudOriginal[index][0];
				searchPoint.y = pointCloudOriginal[index][1];
				searchPoint.z = pointCloudOriginal[index][2];
				kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
				double dis = sqrt(pointNKNSquaredDistance[pointNKNSquaredDistance.size() - 1]);
				if (dis > rMax) {
					rMax = dis;
				}
			}
		}
		radius = rMax;
	}

	void DenoisingPD_SearchingOutlierByKneighboor() {

		//pointOutlierLabel;

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;//kdtree
		std::cout << "Init kdtree" << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = pointCloudOriginal.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		// fills a PointCloud with random data
		for (int i = 0; i < pointCloudOriginal.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = pointCloudOriginal[i][0];
			cloud->points[i].y = pointCloudOriginal[i][1];
			cloud->points[i].z = pointCloudOriginal[i][2];

		}
		kdtree.setInputCloud(cloud);


		double rAverage = 0;
		int K_Out = 8;
		for (int i = 0; i < pointCloudOriginal.size(); i++) {
			std::vector<int> pointIdxNKNSearch(K_Out);
			std::vector<float> pointNKNSquaredDistance(K_Out);
			std::vector<int> pointNeibor_i;
			pcl::PointXYZ searchPoint;
			searchPoint.x = pointCloudOriginal[i][0];
			searchPoint.y = pointCloudOriginal[i][1];
			searchPoint.z = pointCloudOriginal[i][2];
			kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			double dis = sqrt(pointNKNSquaredDistance[pointNKNSquaredDistance.size() - 1]);
			rAverage = rAverage + dis;
			pointOutlierLabel[i] = dis;
		}

		rAverage = rAverage / pointCloudOriginal.size();
		double r_therold = rAverage * 3;

		for (int i = 0; i < pointOutlierLabel.size(); i++) {
			if (pointOutlierLabel[i] > r_therold) {
				pointOutlierLabel[i] = -1;
			}
		}

	}
	
	void DenoisingPD_SavePLY_Point(vector<Point3f> points, string fileNameStore) {

		ofstream f1(fileNameStore);

		//f1 << points.size() << " " << facet.size() << " " << 0 << endl;
		f1 << "ply" << endl;
		f1 << "format ascii 1.0" << endl;
		f1 << "comment VCGLIB generated" << endl;
		f1 << "element vertex " << points.size() << endl;
		f1 << "property float x" << endl;
		f1 << "property float y" << endl;
		f1 << "property float z" << endl;
		f1 << "element face 0" << endl;
		f1 << "property list uchar int vertex_indices" << endl;
		f1 << "end_header" << endl;

		for (int i = 0; i < points.size(); i++) {
			f1 << points[i][0] << " " << points[i][1] << " " << points[i][2] << " " << endl;
		}

		f1.close();

	}
};


