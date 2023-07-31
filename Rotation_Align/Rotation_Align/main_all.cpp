/*********************************************************************************

					 Main for Point Cloud Denoising

						Updating in 2022/07/11

						   By Dr. Chenlei Lv

			The functions includes:
			1. load a point cloud (obj/off/ply)

*********************************************************************************/


#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "PcLoad.hpp"
#include "denoising.hpp"
#include "GlobalEigen_F.hpp"
#include "RenderPC.hpp"
#include "normalCompute.hpp"

using namespace std;

int main(int argc, char** argv) {

	cout << "start!" << endl;
	char* pfileIn;
	char* pfileOut;
	if (argc != 3) {
		cout << "parameter error!" << endl;
		return 0;
	}
	else {
		pfileIn = argv[1];
		pfileOut = argv[2];
	}

	int simplification = 5000;
	vector<vcg::Point3f> point;
	double r = 0.001;

	if (1) {
		int t0 = clock();
		Load_Resampling lr;
		lr.Load_Resampling_init(pfileIn);
		point = lr.pointOriginal;
		int t1 = clock();
		cout << "Rre-processing time:" << float(t1 - t0) / CLOCKS_PER_SEC << "s" << endl;
		//point = lr.pointOriginal;
		//face = lr.faceOriginal;
		r = lr.radiusGlobal;
	}

	//string fileNameStore(pfileIn);
	//int index = fileNameStore.find_last_of(".");
	//string fileName_Save = fileNameStore.substr(0, index);	

	string fileName_Save(pfileOut);

	GlobalEigen_F ge;
	ge.GlobalEigen_init(point);
	//ge.GlobalEigen_Start();
	ge.GlobalEigen_Start_F(fileName_Save);	

}


