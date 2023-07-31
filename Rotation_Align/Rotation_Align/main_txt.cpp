/*********************************************************************************

					 Main for Point Cloud Denoising

						Updating in 2022/09/20

						   By Dr. Chenlei Lv

			The functions includes:
			1. generate txt for pc-align

*********************************************************************************/
/*
#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "FileProcess/CBrowseDir.h"
#include "FileProcess/CStatDir.hpp"
#include "normalCompute.hpp"

void Load_Save_Txt(string pathIn, string pathOut) {

	fstream out2;
	out2.open(pathIn, ios::in);
	int lineNum = 0;
	while (1) {
		char line[255];
		out2.getline(line, sizeof(line));
		lineNum++;
		if (out2.eof())
		{
			break;
		}
	}

	out2.clear();
	out2.seekg(0);

	vector<string> pointString;

	for (int i = 0; i < lineNum; i++) {		

		string st;
		getline(out2, st);
		pointString.push_back(st);

	}

	out2.close();

	ofstream f1(pathOut);

	for (int i = 0; i < pointString.size(); i++) {
				
		f1 << pointString[i] << "_1" << endl;
		f1 << pointString[i] << "_2" << endl;
		f1 << pointString[i] << "_3" << endl;
		f1 << pointString[i] << "_4" << endl;

	}	
	f1.close();

}

int main(int argc, char** argv) {

	cout << "start!" << endl;
	char* pfileRootIn;
	char* pfileRootOut;

	if (argc != 3) {
		cout << "parameter error!" << endl;
		return 0;
	}
	else {
		pfileRootIn = argv[1];
		pfileRootOut = argv[2];
	}

	string pathIn(pfileRootIn);
	string pathOut(pfileRootOut);
	Load_Save_Txt(pathIn, pathOut);		

}
*/
