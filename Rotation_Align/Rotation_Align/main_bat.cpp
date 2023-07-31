/*********************************************************************************

					 Main for Point Cloud Denoising

						Updating in 2022/09/19

						   By Dr. Chenlei Lv

			The functions includes:
			1. generate bat for pc-align

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

void output_bat(vector<string> fileNameList, string fatherPathOut, string fileNameStore){
	
	ofstream f1(fileNameStore);

	for (int i = 0; i < fileNameList.size(); i++) {

		string fileNameList_i = fileNameList[i];
		int find_1 = fileNameList_i.find_last_of("\\");

		string fileNameList_i2 = fileNameList_i.substr(0, find_1-1);
		int find_2 = fileNameList_i2.find_last_of("\\");

		int nameobj = fileNameList_i.find_last_of(".txt");
		string basicPath = fileNameList_i.substr(find_2, nameobj - find_2 - 3);

		string fileNameList_new_i = fatherPathOut + basicPath;

		f1 << "PCA_Align_txt.exe" << " " << fileNameList_i << " " << fileNameList_new_i << endl;		
	
	}
	f1 << "pause()" << endl;
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

	char* buffer;
	string filePathRoot;
	
	if ((buffer = _getcwd(NULL, 0)) == NULL)
	{
		perror("getcwd error");
	}
	else
	{
		printf("%s\n", buffer);
		filePathRoot = buffer;
		free(buffer);
	}
	

	string fatherPathIn(pfileRootIn);
	string fatherPathOut(pfileRootOut);
	string fatherPathBat = filePathRoot + "\\start.bat";

	string fileStyle = "*.txt";
	char filePath[256];
	strcpy_s(filePath, fatherPathIn.c_str());
	CStatDir statdir;

	if (!statdir.SetInitDir(filePath))
	{
		cout << "error! file path wrong!" << endl;
	}
	statdir.BeginBrowse(fileStyle.c_str());
	printf("OutPut:%d\nOutPutSecond:%d\n", statdir.GetFileCount(), statdir.GetSubdirCount());
	printf("CountNumber:%d\n", statdir.lifp.size());

	vector<string> fileNameList = statdir.lifp;	
	output_bat(fileNameList, fatherPathOut, fatherPathBat);

}
*/
