/*********************************************************************************

					     Point Cloud Rendering

						Programing in 2022/07/13

						   By Dr. Chenlei Lv

			The functions includes:
			1. Transfer a point cloud into a set of ball;
			2. Generate normal-based color for the point cloud;
			3. Estimate the radius between points;
			4. Output the point cloud.

*********************************************************************************/

#pragma once
#include "PcLoad.hpp"
#include "denoising.hpp"

using namespace std;

class RenderPC {

private:

	vector<Point3f> point;
	vector<Point3f> pointBall;
	vector<vector<int>> faceball;

	vector<Point3f> pointBall_Global;
	vector<vector<int>> faceball_Global;
	float radus_B;
	float radus_G;

public:

	void RenderPC_init(vector<Point3f> point_input) {

		point = point_input;	
		string ballPath = "ball.obj";
		Load_Resampling lr;
		lr.Load_Resampling_init(ballPath);
		pointBall = lr.pointOriginal;
		faceball = lr.faceOriginal;

		radus_B = sqrt(pointBall[0][0] * pointBall[0][0]
			+ pointBall[0][1] * pointBall[0][1]
			+ pointBall[0][2] * pointBall[0][2]);

		RenderPC_Estimate_Radius();
		float multipule = radus_G / radus_B;
		for (int i = 0; i < pointBall.size(); i++) {
			pointBall[i][0] = pointBall[i][0] * multipule;
			pointBall[i][1] = pointBall[i][1] * multipule;
			pointBall[i][2] = pointBall[i][2] * multipule;		
		}
	
	}

	void RenderPC_Draw(string Path) {

		//pointBall_Global
		//faceball_Global
		int pointIndex = 0;
		for (int i = 0; i < point.size(); i++) {			

			//draw point
			vector<Point3f> point_i = pointBall;
			for (int j = 0; j < pointBall.size(); j++) {

				point_i[j][0] = point_i[j][0] + point[i][0];
				point_i[j][1] = point_i[j][1] + point[i][1];
				point_i[j][2] = point_i[j][2] + point[i][2];	

			}
			pointBall_Global.insert(pointBall_Global.end(), point_i.begin(), point_i.end());			
			
			//draw face			
			vector<vector<int>> faceball_i = faceball;
			for (int j = 0; j < faceball_i.size(); j++) {

				for (int k = 0; k < faceball_i[j].size(); k++) {
					faceball_i[j][k] = faceball_i[j][k] + pointIndex + 1;				
				}

			}

			faceball_Global.insert(faceball_Global.end(), faceball_i.begin(), faceball_i.end());

			pointIndex = pointIndex + pointBall.size();
		
		}

		RenderPC_StoreObj(Path);
	
	
	}

	void RenderPC_Draw_Normal(string Path, vector<Point3f> normal) {

		//pointBall_Global
		//faceball_Global
		int pointIndex = 0;
		for (int i = 0; i < point.size(); i++) {

			//draw point
			vector<Point3f> point_i = pointBall;
			for (int j = 0; j < pointBall.size(); j++) {

				point_i[j][0] = point_i[j][0] + point[i][0];
				point_i[j][1] = point_i[j][1] + point[i][1];
				point_i[j][2] = point_i[j][2] + point[i][2];

			}
			pointBall_Global.insert(pointBall_Global.end(), point_i.begin(), point_i.end());

			//draw face			
			vector<vector<int>> faceball_i = faceball;
			for (int j = 0; j < faceball_i.size(); j++) {

				for (int k = 0; k < faceball_i[j].size(); k++) {
					faceball_i[j][k] = faceball_i[j][k] + pointIndex + 1;
				}

			}

			faceball_Global.insert(faceball_Global.end(), faceball_i.begin(), faceball_i.end());

			pointIndex = pointIndex + pointBall.size();

		}

		//transfer normal to color map

		for (int i = 0; i < normal.size(); i++) {
			
			float dis = sqrt(normal[i][0] * normal[i][0] + normal[i][1] * normal[i][1] + normal[i][2] * normal[i][2]);
			normal[i][0] = abs(normal[i][0] / dis);
			normal[i][1] = abs(normal[i][1] / dis);
			normal[i][2] = abs(normal[i][2] / dis);
		
		}

		RenderPC_StoreObj(Path, normal);

	}

	void RenderPC_Draw_RGBCube(string Path) {

		vector<Point3f> rgbCube = point;

		//pointBall_Global
		//faceball_Global
		int pointIndex = 0;
		for (int i = 0; i < point.size(); i++) {

			//draw point
			vector<Point3f> point_i = pointBall;
			for (int j = 0; j < pointBall.size(); j++) {

				point_i[j][0] = point_i[j][0] + point[i][0];
				point_i[j][1] = point_i[j][1] + point[i][1];
				point_i[j][2] = point_i[j][2] + point[i][2];

			}
			pointBall_Global.insert(pointBall_Global.end(), point_i.begin(), point_i.end());

			//draw face			
			vector<vector<int>> faceball_i = faceball;
			for (int j = 0; j < faceball_i.size(); j++) {

				for (int k = 0; k < faceball_i[j].size(); k++) {
					faceball_i[j][k] = faceball_i[j][k] + pointIndex + 1;
				}

			}

			faceball_Global.insert(faceball_Global.end(), faceball_i.begin(), faceball_i.end());

			pointIndex = pointIndex + pointBall.size();

		}

		//map point cloud into a cube

		//compute the center
		Point3f p_Center;
		p_Center[0] = 0;
		p_Center[1] = 0;
		p_Center[2] = 0;

		for (int i = 0; i < point.size(); i++) {

			p_Center[0] = p_Center[0] + point[i][0];
			p_Center[1] = p_Center[1] + point[i][1];
			p_Center[2] = p_Center[2] + point[i][2];

		}

		p_Center[0] = p_Center[0] / point.size();
		p_Center[1] = p_Center[1] / point.size();
		p_Center[2] = p_Center[2] / point.size();

		vector<Point3f> pointStore = point;
		float maxRadius = 0;

		for (int i = 0; i < pointStore.size(); i++) {

			pointStore[i][0] = pointStore[i][0] - p_Center[0];
			pointStore[i][1] = pointStore[i][1] - p_Center[1];
			pointStore[i][2] = pointStore[i][2] - p_Center[2];	

			if (abs(pointStore[i][0]) > maxRadius) {
				maxRadius = abs(pointStore[i][0]);			
			}
			if (abs(pointStore[i][1]) > maxRadius) {
				maxRadius = abs(pointStore[i][1]);
			}
			if (abs(pointStore[i][2]) > maxRadius) {
				maxRadius = abs(pointStore[i][2]);
			}
		
		}

		for (int i = 0; i < pointStore.size(); i++) {

			pointStore[i][0] = pointStore[i][0] * 0.5 / maxRadius;
			pointStore[i][1] = pointStore[i][1] * 0.5 / maxRadius;
			pointStore[i][2] = pointStore[i][2] * 0.5 / maxRadius;	

			rgbCube[i][0] = pointStore[i][0] + 0.5;
			rgbCube[i][1] = pointStore[i][1] + 0.5;
			rgbCube[i][2] = pointStore[i][2] + 0.5;
		
		}

		RenderPC_StorePLY(Path, rgbCube);
		
	
	}

private:



	void RenderPC_Estimate_Radius() {

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		std::cout << "Init kdtree" << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = point.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		// fills a PointCloud with random data
		for (int i = 0; i < point.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = point[i][0];
			cloud->points[i].y = point[i][1];
			cloud->points[i].z = point[i][2];

		}
		kdtree.setInputCloud(cloud);

		float sumdis = 0;
		for (int i = 0; i < point.size();i++) {

			std::vector<int> pointIdxNKNSearch(2);
			std::vector<float> pointNKNSquaredDistance(2);
			std::vector<int> result;
			double r_i = pointNKNSquaredDistance[pointNKNSquaredDistance.size() - 1];
			pcl::PointXYZ searchPoint;
			searchPoint.x = point[i][0];
			searchPoint.y = point[i][1];
			searchPoint.z = point[i][2];
			kdtree.nearestKSearch(searchPoint, 2, pointIdxNKNSearch, pointNKNSquaredDistance);
			sumdis = sumdis + sqrt(pointNKNSquaredDistance[1]);
		
		}
		sumdis = sumdis / point.size();
		radus_G =  sumdis/2;
	
	}

	void RenderPC_StoreObj(string Path) {

		ofstream f1(Path);

		for (int i = 0; i < pointBall_Global.size(); i++) {

			f1 << "v"<<" "<<pointBall_Global[i][0] << " " << pointBall_Global[i][1] << " " << pointBall_Global[i][2] << endl;

		}

		for (int i = 0; i < faceball_Global.size(); i++) {

			f1 << "f" << " " << faceball_Global[i][0] << " " << faceball_Global[i][1] << " " << faceball_Global[i][2] << endl;

		}

		f1.close();
	
	
	}

	void RenderPC_StoreObj(string Path, vector<Point3f> normalColor) {

		ofstream f1(Path);

		

		for (int i = 0; i < pointBall_Global.size(); i++) {

			int index = i/ pointBall.size();

			f1 << "v" << " " << pointBall_Global[i][0] << " " << pointBall_Global[i][1] << " " << pointBall_Global[i][2] <<" "<<
				(int)(normalColor[index][0] * 255) << " "<< (int)(normalColor[index][1] * 255)<<" "<< (int)(normalColor[index][2] * 255) << endl;

		}

		for (int i = 0; i < faceball_Global.size(); i++) {

			f1 << "f" << " " << faceball_Global[i][0] << " " << faceball_Global[i][1] << " " << faceball_Global[i][2] << endl;

		}

		f1.close();


	}

	void RenderPC_StorePLY(string Path, vector<Point3f> normalColor) {

		ofstream f1(Path);
		f1 << "ply" << endl;
		f1 << "format ascii 1.0" << endl;
		f1 << "comment VCGLIB generated" << endl;
		f1 << "element vertex " << pointBall_Global.size() << endl;
		f1 << "property float x" << endl;
		f1 << "property float y" << endl;
		f1 << "property float z" << endl;
		f1 << "property uchar red " << endl;
		f1 << "property uchar green" << endl;
		f1 << "property uchar blue" << endl;
		f1 << "element face " << faceball_Global.size() << endl;
		f1 << "property list uchar int vertex_indices" << endl;
		f1 << "property uchar red " << endl;
		f1 << "property uchar green" << endl;
		f1 << "property uchar blue" << endl;
		f1 << "end_header" << endl;		

		for (int i = 0; i < pointBall_Global.size(); i++) {

			int index = i / pointBall.size();

			f1 << pointBall_Global[i][0] << " " << pointBall_Global[i][1] << " " << pointBall_Global[i][2] << " " <<
				(int)(normalColor[index][0] * 255) << " " << (int)(normalColor[index][1] * 255) << " " << (int)(normalColor[index][2] * 255) << endl;

		}

		int faceNum = 3;

		for (int i = 0; i < faceball_Global.size(); i++) {

			int index = i / faceball.size();

			f1 << faceNum <<" "<< faceball_Global[i][0] - 1 << " " << faceball_Global[i][1] - 1 << " " << faceball_Global[i][2] - 1 << " " <<
				(int)(normalColor[index][0] * 255) << " " << (int)(normalColor[index][1] * 255) << " " << (int)(normalColor[index][2] * 255) << endl;

		}

		f1.close();


	}

	


};
