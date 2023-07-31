/*********************************************************************************

						 GlobalEigen for Point Cloud

							Updating in 2022/08/04

							  By Dr. Chenlei Lv

				The functions includes:
				1. Using PCA to analysis a point cloud
				2. Using eigenvectors(smallest and largest eigenvalues)
				to construct a local coordinate system
				3. Transfer point cloud by the system

*********************************************************************************/

#pragma once
#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <GL/freeglut.h> 
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h> 
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <Eigen/Dense>
#include "PcLoad.hpp"
#include "normalCompute.hpp"

using namespace std;

class GlobalEigen_F {

private:

	vector<Point3f> point;
	vector<Point3f> point_Transfer;
	vector<Point3f> localCoordinate;
	Point3f v_s;//vector with smallest eigen value
	Point3f v_l;//vector with largest eigen value	
	Point3f v_sl;//vector with largest eigen value
	float scale_estimate;


public:

	void GlobalEigen_init(vector<Point3f> point_Input) {

		point = point_Input;

		//align center
		Point3f point_center;
		point_center[0] = 0;
		point_center[1] = 0;
		point_center[2] = 0;

		Point3f point_random = point[point.size() / 2];
		float dismax = 0;

		for (int i = 0; i < point.size(); i++) {

			point_center[0] = point_center[0] + point[i][0];
			point_center[1] = point_center[1] + point[i][1];
			point_center[2] = point_center[2] + point[i][2];
			float dis_i = GlobalEigen_Dis(point_random, point[i]);
			if (dis_i > dismax) {
				dismax = dis_i;
			}
		}

		scale_estimate = dismax;

		point_center[0] = point_center[0] / point.size();
		point_center[1] = point_center[1] / point.size();
		point_center[2] = point_center[2] / point.size();

		for (int i = 0; i < point.size(); i++) {

			point[i][0] = point[i][0] - point_center[0];
			point[i][1] = point[i][1] - point_center[1];
			point[i][2] = point[i][2] - point_center[2];

		}

		point_Transfer = point;

	}

	void GlobalEigen_Start() {

		Eigen::MatrixXf pointMatrix(point.size(), 3);

		for (int i = 0; i < point.size(); i++) {

			pointMatrix(i, 0) = point[i][0];
			pointMatrix(i, 1) = point[i][1];
			pointMatrix(i, 2) = point[i][2];

		}

		Eigen::MatrixXf pointMTM = pointMatrix.transpose() * pointMatrix;
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigen_solver(pointMTM);

		Eigen::VectorXf eigenvalues = eigen_solver.eigenvalues();
		Eigen::MatrixXf eigenvectors = eigen_solver.eigenvectors();

		cout << "eigenvalues:" << endl;
		cout << eigenvalues << endl;

		cout << "eigenvectors:" << endl;
		cout << eigenvectors << endl;

		Eigen::VectorXf vs = eigenvectors.col(0);
		Eigen::VectorXf vl = eigenvectors.col(eigenvalues.size() - 1);

		//cout << "vs:" << endl;
		//cout << vs << endl;

		v_s[0] = vs(0);
		v_s[1] = vs(1);
		v_s[2] = vs(2);

		v_l[0] = vl(0);
		v_l[1] = vl(1);
		v_l[2] = vl(2);

		//compute local coordinate system;
		vs.normalize();
		vl.normalize();

		Eigen::VectorXf vz = vs;
		Eigen::VectorXf vy = GlobalEigen_Cross(vs, vl);
		Eigen::VectorXf vx = GlobalEigen_Cross(vz, vy);

		Point3f pvx;
		pvx[0] = vx(0);
		pvx[1] = vx(1);
		pvx[2] = vx(2);

		Point3f pvy;
		pvy[0] = vy(0);
		pvy[1] = vy(1);
		pvy[2] = vy(2);

		Point3f pvz;
		pvz[0] = vz(0);
		pvz[1] = vz(1);
		pvz[2] = vz(2);

		localCoordinate.push_back(pvx);
		localCoordinate.push_back(pvy);
		localCoordinate.push_back(pvz);

		GlobalEigen_Transfer();

		v_sl = GlobalEigen_Cross(v_s, v_l);//vector with largest eigen value		

	}

	void GlobalEigen_Start_F(string fin) {

		vector<vector<Point3f>> pointTransfer_list;

		Eigen::MatrixXf pointMatrix(point.size(), 3);

		for (int i = 0; i < point.size(); i++) {

			pointMatrix(i, 0) = point[i][0];
			pointMatrix(i, 1) = point[i][1];
			pointMatrix(i, 2) = point[i][2];

		}

		Eigen::MatrixXf pointMTM = pointMatrix.transpose() * pointMatrix;
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigen_solver(pointMTM);

		Eigen::VectorXf eigenvalues = eigen_solver.eigenvalues();
		Eigen::MatrixXf eigenvectors = eigen_solver.eigenvectors();

		cout << "eigenvalues:" << endl;
		cout << eigenvalues << endl;

		cout << "eigenvectors:" << endl;
		cout << eigenvectors << endl;

		Eigen::VectorXf vs = eigenvectors.col(0);
		Eigen::VectorXf vl = eigenvectors.col(eigenvalues.size() - 1);

		//cout << "vs:" << endl;
		//cout << vs << endl;

		v_s[0] = vs(0);
		v_s[1] = vs(1);
		v_s[2] = vs(2);

		v_l[0] = vl(0);
		v_l[1] = vl(1);
		v_l[2] = vl(2);

		//compute local coordinate system;
		//pose1
		vs.normalize();
		vl.normalize();
		Eigen::VectorXf vz_1 = vs;
		Eigen::VectorXf vy_1 = GlobalEigen_Cross(vs, vl);
		Eigen::VectorXf vx_1 = GlobalEigen_Cross(vz_1, vy_1);
		Point3f pvx;
		pvx[0] = vx_1(0);
		pvx[1] = vx_1(1);
		pvx[2] = vx_1(2);
		Point3f pvy;
		pvy[0] = vy_1(0);
		pvy[1] = vy_1(1);
		pvy[2] = vy_1(2);
		Point3f pvz;
		pvz[0] = vz_1(0);
		pvz[1] = vz_1(1);
		pvz[2] = vz_1(2);
		vector<Point3f> localCoordinate_1;
		localCoordinate_1.push_back(pvx);
		localCoordinate_1.push_back(pvy);
		localCoordinate_1.push_back(pvz);
		vector<Point3f> p1 = GlobalEigen_Transfer_F(localCoordinate_1);

		//pose2
		Eigen::VectorXf vs_2 = vs;
		Eigen::VectorXf vl_2 = vl;
		vs_2[0] = -vs(0);
		vs_2[1] = -vs(1);
		vs_2[2] = -vs(2);
		vl_2[0] = vl(0);
		vl_2[1] = vl(1);
		vl_2[2] = vl(2);
		Eigen::VectorXf vz_2 = vs_2;
		Eigen::VectorXf vy_2 = GlobalEigen_Cross(vs_2, vl_2);
		Eigen::VectorXf vx_2 = GlobalEigen_Cross(vz_2, vy_2);
		//Point3f pvx;
		pvx[0] = vx_2(0);
		pvx[1] = vx_2(1);
		pvx[2] = vx_2(2);
		//Point3f pvy;
		pvy[0] = vy_2(0);
		pvy[1] = vy_2(1);
		pvy[2] = vy_2(2);
		//Point3f pvz;
		pvz[0] = vz_2(0);
		pvz[1] = vz_2(1);
		pvz[2] = vz_2(2);
		vector<Point3f> localCoordinate_2;
		localCoordinate_2.push_back(pvx);
		localCoordinate_2.push_back(pvy);
		localCoordinate_2.push_back(pvz);
		vector<Point3f> p2 = GlobalEigen_Transfer_F(localCoordinate_2);

		//pose3
		Eigen::VectorXf vs_3 = vs;
		Eigen::VectorXf vl_3 = vl;
		vs_3[0] = -vs(0);
		vs_3[1] = -vs(1);
		vs_3[2] = -vs(2);
		vl_3[0] = -vl(0);
		vl_3[1] = -vl(1);
		vl_3[2] = -vl(2);
		Eigen::VectorXf vz_3 = vs_3;
		Eigen::VectorXf vy_3 = GlobalEigen_Cross(vs_3, vl_3);
		Eigen::VectorXf vx_3 = GlobalEigen_Cross(vz_3, vy_3);
		//Point3f pvx;
		pvx[0] = vx_3(0);
		pvx[1] = vx_3(1);
		pvx[2] = vx_3(2);
		//Point3f pvy;
		pvy[0] = vy_3(0);
		pvy[1] = vy_3(1);
		pvy[2] = vy_3(2);
		//Point3f pvz;
		pvz[0] = vz_3(0);
		pvz[1] = vz_3(1);
		pvz[2] = vz_3(2);
		vector<Point3f> localCoordinate_3;
		localCoordinate_3.push_back(pvx);
		localCoordinate_3.push_back(pvy);
		localCoordinate_3.push_back(pvz);
		vector<Point3f> p3 = GlobalEigen_Transfer_F(localCoordinate_3);

		//pose4
		Eigen::VectorXf vs_4 = vs;
		Eigen::VectorXf vl_4 = vl;
		vs_4[0] = vs(0);
		vs_4[1] = vs(1);
		vs_4[2] = vs(2);
		vl_4[0] = -vl(0);
		vl_4[1] = -vl(1);
		vl_4[2] = -vl(2);
		Eigen::VectorXf vz_4 = vs_4;
		Eigen::VectorXf vy_4 = GlobalEigen_Cross(vs_4, vl_4);
		Eigen::VectorXf vx_4 = GlobalEigen_Cross(vz_4, vy_4);
		//Point3f pvx;
		pvx[0] = vx_4(0);
		pvx[1] = vx_4(1);
		pvx[2] = vx_4(2);
		//Point3f pvy;
		pvy[0] = vy_4(0);
		pvy[1] = vy_4(1);
		pvy[2] = vy_4(2);
		//Point3f pvz;
		pvz[0] = vz_4(0);
		pvz[1] = vz_4(1);
		pvz[2] = vz_4(2);
		vector<Point3f> localCoordinate_4;
		localCoordinate_4.push_back(pvx);
		localCoordinate_4.push_back(pvy);
		localCoordinate_4.push_back(pvz);
		vector<Point3f> p4 = GlobalEigen_Transfer_F(localCoordinate_4);

		pointTransfer_list.push_back(p1);
		pointTransfer_list.push_back(p2);
		pointTransfer_list.push_back(p3);
		pointTransfer_list.push_back(p4);

		//GlobalEigen_Save_Void(fin, pointTransfer_list);
		GlobalEigen_Save_Void_text(fin, pointTransfer_list);
	}

	void GlobalEigen_Save_Cord(string fin) {

		ofstream f1(fin);
		f1 << "ply" << endl;
		f1 << "format ascii 1.0" << endl;
		f1 << "comment VCGLIB generated" << endl;
		f1 << "element vertex " << point_Transfer.size() + 750 << endl;
		f1 << "property float x" << endl;
		f1 << "property float y" << endl;
		f1 << "property float z" << endl;
		f1 << "property uchar red " << endl;
		f1 << "property uchar green" << endl;
		f1 << "property uchar blue" << endl;
		f1 << "element face 0" << endl;
		f1 << "property list uchar int vertex_indices" << endl;
		f1 << "end_header" << endl;

		for (int i = 0; i < point_Transfer.size(); i++) {

			f1 << point_Transfer[i][0] << " " << point_Transfer[i][1] << " " << point_Transfer[i][2] << " " <<
				128 << " " << 0 << " " << 128 << endl;

		}

		//1500 point into two vectors		

		float step_s = scale_estimate / 500;
		float step_l = scale_estimate / 250;
		for (int i = 0; i < 250; i++) {

			f1 << i * step_s * v_s[0] << " " << i * step_s * v_s[1] << " " << i * step_s * v_s[2] << " " <<
				255 << " " << 0 << " " << 0 << endl;

		}
		//for (int i = 0; i < 250; i++) {

			//f1 << -i * step_s * v_s[0] << " " << -i * step_s * v_s[1] << " " << -i * step_s * v_s[2] << " " <<
				//255 << " " << 0 << " " << 0 << endl;

		//}

		for (int i = 0; i < 250; i++) {

			f1 << i * step_l * v_l[0] << " " << i * step_l * v_l[1] << " " << i * step_l * v_l[2] << " " <<
				0 << " " << 255 << " " << 0 << endl;

		}
		//for (int i = 0; i < 250; i++) {

			//f1 << -i * step_l * v_l[0] << " " << -i * step_l * v_l[1] << " " << -i * step_l * v_l[2] << " " <<
				//0 << " " << 255 << " " << 0 << endl;

		//}	

		for (int i = 0; i < 250; i++) {

			f1 << i * step_s * v_sl[0] << " " << i * step_s * v_sl[1] << " " << i * step_s * v_sl[2] << " " <<
				0 << " " << 0 << " " << 255 << endl;

		}

		f1.close();

	}

	vector<Point3f> GlobalEigen_Save(string fin) {

		ofstream f1(fin);
		f1 << "ply" << endl;
		f1 << "format ascii 1.0" << endl;
		f1 << "comment VCGLIB generated" << endl;
		f1 << "element vertex " << point_Transfer.size() << endl;
		f1 << "property float x" << endl;
		f1 << "property float y" << endl;
		f1 << "property float z" << endl;
		f1 << "property uchar red " << endl;
		f1 << "property uchar green" << endl;
		f1 << "property uchar blue" << endl;
		f1 << "element face 0" << endl;
		f1 << "property list uchar int vertex_indices" << endl;
		f1 << "end_header" << endl;

		for (int i = 0; i < point_Transfer.size(); i++) {

			f1 << point_Transfer[i][0] << " " << point_Transfer[i][1] << " " << point_Transfer[i][2] << " " <<
				128 << " " << 0 << " " << 128 << endl;

		}

		f1.close();

		return point_Transfer;

	}

	void GlobalEigen_Save_Void(string fin, vector<vector<Point3f>> point_Transfer_list) {

		string s_1 = fin + "_1.ply";
		string s_2 = fin + "_2.ply";
		string s_3 = fin + "_3.ply";
		string s_4 = fin + "_4.ply";

		vector<string> path_list;
		path_list.push_back(s_1);
		path_list.push_back(s_2);
		path_list.push_back(s_3);
		path_list.push_back(s_4);

		for (int k = 0; k < path_list.size(); k++) {

			ofstream f1(path_list[k]);
			f1 << "ply" << endl;
			f1 << "format ascii 1.0" << endl;
			f1 << "comment VCGLIB generated" << endl;
			f1 << "element vertex " << point_Transfer.size() << endl;
			f1 << "property float x" << endl;
			f1 << "property float y" << endl;
			f1 << "property float z" << endl;
			f1 << "property uchar red " << endl;
			f1 << "property uchar green" << endl;
			f1 << "property uchar blue" << endl;
			f1 << "element face 0" << endl;
			f1 << "property list uchar int vertex_indices" << endl;
			f1 << "end_header" << endl;

			for (int i = 0; i < point_Transfer_list[k].size(); i++) {

				f1 << point_Transfer_list[k][i][0] << " " << point_Transfer_list[k][i][1] << " " << point_Transfer_list[k][i][2] << " " <<
					128 << " " << 0 << " " << 128 << endl;

			}

			f1.close();		
		
		}		

	}

	void GlobalEigen_Save_Void_text(string fin, vector<vector<Point3f>> point_Transfer_list) {

		string s_1 = fin + "_1.txt";
		string s_2 = fin + "_2.txt";
		string s_3 = fin + "_3.txt";
		string s_4 = fin + "_4.txt";

		vector<string> path_list;
		path_list.push_back(s_1);
		path_list.push_back(s_2);
		path_list.push_back(s_3);
		path_list.push_back(s_4);

		vector<vector<Point3f>> pointNormal;
		NormalEstimation ne;

		for (int i = 0; i < point_Transfer_list.size(); i++) {

			vector<Point3f> point_i = point_Transfer_list[i];
			vector<Point3f> pointnormal_i = ne.estimateNormal_PCL_MP(point_i);
			pointNormal.push_back(pointnormal_i);
		
		}		

		for (int k = 0; k < path_list.size(); k++) {

			ofstream f1(path_list[k]);			

			for (int i = 0; i < point_Transfer_list[k].size(); i++) {

				f1 << point_Transfer_list[k][i][0] << " " << point_Transfer_list[k][i][1] << " " << point_Transfer_list[k][i][2] << " " <<
					pointNormal[k][i][0] << " " << pointNormal[k][i][1] << " " << pointNormal[k][i][2] << endl;

			}

			f1.close();

		}

	}

private:

	vector<Point3f> GlobalEigen_LocalTransfer(Eigen::VectorXf vs, Eigen::VectorXf vl) {

		Eigen::VectorXf vz = vs;
		Eigen::VectorXf vy = GlobalEigen_Cross(vs, vl);
		Eigen::VectorXf vx = GlobalEigen_Cross(vz, vy);

		Point3f pvx;
		pvx[0] = vx(0);
		pvx[1] = vx(1);
		pvx[2] = vx(2);

		Point3f pvy;
		pvy[0] = vy(0);
		pvy[1] = vy(1);
		pvy[2] = vy(2);

		Point3f pvz;
		pvz[0] = vz(0);
		pvz[1] = vz(1);
		pvz[2] = vz(2);

		vector<Point3f> localCoordinate_temp;

		localCoordinate_temp.push_back(pvx);
		localCoordinate_temp.push_back(pvy);
		localCoordinate_temp.push_back(pvz);

		return localCoordinate_temp;

	}

	float GlobalEigen_Dis(Point3f p1, Point3f p2) {

		float dis = 0;

		dis = sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) +
			(p1[1] - p2[1]) * (p1[1] - p2[1]) +
			(p1[2] - p2[2]) * (p1[2] - p2[2]));

		return dis;

	}

	void GlobalEigen_Transfer() {

		//point_Transfer;
		//localCoordinate;
		Point3f localCenter;
		localCenter[0] = 0;
		localCenter[1] = 0;
		localCenter[2] = 0;
		point_Transfer = GlobalEigen_Global2local(localCoordinate, localCenter, point_Transfer);

		//vector<Point3f> v_sl;
		//v_sl.push_back(v_s);
		//v_sl.push_back(v_l);
		//v_sl = GlobalEigen_Global2local(localCoordinate, localCenter, v_sl);
		//v_s = v_sl[0];
		//v_l = v_sl[1];

		//check direction
		int left_sum_x = 0;
		int right_sum_x = 0;
		int left_sum_z = 0;
		int right_sum_z = 0;

		for (int i = 0; i < point_Transfer.size(); i++) {

			if (point_Transfer[i][0] > 0) {
				right_sum_x++;
			}
			else if (point_Transfer[i][0] < 0) {
				left_sum_x++;
			}
			else {
				continue;
			}

			if (point_Transfer[i][2] > 0) {
				right_sum_z++;
			}
			else if (point_Transfer[i][2] < 0) {
				left_sum_z++;
			}
			else {
				continue;
			}

		}

		Eigen::VectorXf vs_temp(3);
		Eigen::VectorXf vl_temp(3);
		vs_temp(0) = v_s[0];
		vs_temp(1) = v_s[1];
		vs_temp(2) = v_s[2];
		vl_temp(0) = v_l[0];
		vl_temp(1) = v_l[1];
		vl_temp(2) = v_l[2];

		//cout << "vs_temp" << endl;
		//cout << vs_temp << endl;

		if (left_sum_x > right_sum_x) {

			vl_temp(0) = -vl_temp(0);
			vl_temp(1) = -vl_temp(1);
			vl_temp(2) = -vl_temp(2);

		}

		if (left_sum_z < right_sum_z) {

			vs_temp(0) = -vs_temp(0);
			vs_temp(1) = -vs_temp(1);
			vs_temp(2) = -vs_temp(2);

		}

		vector<Point3f> localCoordinate_temp = GlobalEigen_LocalTransfer(vs_temp, vl_temp);
		point_Transfer = GlobalEigen_Global2local(localCoordinate_temp, localCenter, point);
		v_s[0] = vs_temp(0);
		v_s[1] = vs_temp(1);
		v_s[2] = vs_temp(2);
		v_l[0] = vl_temp(0);
		v_l[1] = vl_temp(1);
		v_l[2] = vl_temp(2);

		vector<Point3f> v_sl;
		v_sl.push_back(v_s);
		v_sl.push_back(v_l);
		v_sl = GlobalEigen_Global2local(localCoordinate_temp, localCenter, v_sl);
		v_s = v_sl[0];
		v_l = v_sl[1];

	}

	vector<Point3f> GlobalEigen_Transfer_F(vector<Point3f> localCoordinate_input) {

		//point_Transfer;
		//localCoordinate;
		Point3f localCenter;
		localCenter[0] = 0;
		localCenter[1] = 0;
		localCenter[2] = 0;
		vector<Point3f> point_Transfer_i = GlobalEigen_Global2local(localCoordinate_input, localCenter, point);		
		return point_Transfer_i;

	}


	Eigen::VectorXf GlobalEigen_Cross(Eigen::VectorXf v1, Eigen::VectorXf v2) {

		Eigen::VectorXf p_i(3);
		p_i(0) = v1[1] * v2[2] - v1[2] * v2[1];
		p_i(1) = -(v1[0] * v2[2] - v1[2] * v2[0]);
		p_i(2) = v1[0] * v2[1] - v1[1] * v2[0];
		return p_i;

	}

	Point3f GlobalEigen_Cross(Point3f v1, Point3f v2) {

		Point3f p_i;
		p_i[0] = v1[1] * v2[2] - v1[2] * v2[1];
		p_i[1] = -(v1[0] * v2[2] - v1[2] * v2[0]);
		p_i[2] = v1[0] * v2[1] - v1[1] * v2[0];
		return p_i;

	}

	vector<Point3f> GlobalEigen_Global2local(vector<Point3f> localAxis,
		Point3f localCenter, vector<Point3f> pG) {

		Point3f xAxis = localAxis[0];
		Point3f yAxis = localAxis[1];
		Point3f zAxis = localAxis[2];

		Point3f o;
		o[0] = -localCenter[0] * xAxis[0] - localCenter[1] * xAxis[1] - localCenter[2] * xAxis[2];
		o[1] = -localCenter[0] * yAxis[0] - localCenter[1] * yAxis[1] - localCenter[2] * yAxis[2];
		o[2] = -localCenter[0] * zAxis[0] - localCenter[1] * zAxis[1] - localCenter[2] * zAxis[2];

		for (int i = 0; i < pG.size(); i++) {
			float plx = pG[i][0] * xAxis[0] + pG[i][1] * xAxis[1] + pG[i][2] * xAxis[2] + o[0];
			float ply = pG[i][0] * yAxis[0] + pG[i][1] * yAxis[1] + pG[i][2] * yAxis[2] + o[1];
			float plz = pG[i][0] * zAxis[0] + pG[i][1] * zAxis[1] + pG[i][2] * zAxis[2] + o[2];
			pG[i][0] = plx;
			pG[i][1] = ply;
			pG[i][2] = plz;
		}

		return pG;

	}

	vector<Point3f> GlobalEigen_Local2Global(vector<Point3f> localAxis,
		Point3f localCenter, vector<Point3f> pl) {

		Point3f xAxis = localAxis[0];
		Point3f yAxis = localAxis[1];
		Point3f zAxis = localAxis[2];

		for (int i = 0; i < pl.size(); i++) {

			float pGx = pl[i][0] * xAxis[0] + pl[i][1] * yAxis[0] + pl[i][2] * zAxis[0] + localCenter[0];
			float pGy = pl[i][0] * xAxis[1] + pl[i][1] * yAxis[1] + pl[i][2] * zAxis[1] + localCenter[1];
			float pGz = pl[i][0] * xAxis[2] + pl[i][1] * yAxis[2] + pl[i][2] * zAxis[2] + localCenter[2];
			pl[i][0] = pGx;
			pl[i][1] = pGy;
			pl[i][2] = pGz;

		}

		return pl;

	}

};


