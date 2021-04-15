#include <iostream>
#include <stdio.h>
#include <time.h>
#include <direct.h>
#include <io.h>
#include <algorithm>
#include <stdlib.h>
#include "fileio.h"
#include <windows.h>

using namespace std;

static string getFileSuffix(string filename)
{
	string suffix;
	suffix = filename.substr(filename.find_last_of('.') + 1);//获取文件后缀  
	return suffix;
}

/*
* @function: 获取cate_dir目录下的所有文件名
* @param: cate_dir - string类型
* @result：vector<string>类型
*/
static  vector<string> getFiles(std::string cate_dir)
{
	vector<string> files;//存放文件名  

#ifdef _WIN32  
	_finddata_t file;
	long lf;
	//输入文件夹路径  
	if ((lf = _findfirst(cate_dir.c_str(), &file)) == -1) {
		std::cout << cate_dir << " not found!!!" << endl;
	}
	else {
		while (_findnext(lf, &file) == 0) {
			//输出文件名  
			//cout<<file.name<<endl;  
			if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0)
				continue;
			files.push_back(file.name);
		}
	}
	_findclose(lf);
#endif  

#ifdef linux  
	DIR *dir;
	struct dirent *ptr;
	char base[1000];

	if ((dir = opendir(cate_dir.c_str())) == NULL)
	{
		perror("Open dir error...");
		exit(1);
	}

	while ((ptr = readdir(dir)) != NULL)
	{
		if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)    ///current dir OR parrent dir  
			continue;
		else if (ptr->d_type == 8)    ///file  
									  //printf("d_name:%s/%s\n",basePath,ptr->d_name);  
			files.push_back(ptr->d_name);
		else if (ptr->d_type == 10)    ///link file  
									   //printf("d_name:%s/%s\n",basePath,ptr->d_name);  
			continue;
		else if (ptr->d_type == 4)    ///dir  
		{
			files.push_back(ptr->d_name);
			/*
			memset(base,'\0',sizeof(base));
			strcpy(base,basePath);
			strcat(base,"/");
			strcat(base,ptr->d_nSame);
			readFileList(base);
			*/
		}
	}
	closedir(dir);
#endif  

	//排序，按从小到大排序  
	sort(files.begin(), files.end());
	return files;
}

bool findCorrespondingFiles(const std::string &rootDir,
	std::vector<std::string > &filelists, const std::string &_suffix)
{
	string searchDir;
	if (rootDir.back() != '\\' || rootDir.back() != '/')
	{
		searchDir = rootDir + "\\";
		searchDir = searchDir + "*";
	}
	else
	{
		searchDir = rootDir + "*";
	}

	filelists.clear();
	vector<string> files;
	files = getFiles(searchDir);

	for (vector<string>::iterator it = files.begin(); it != files.end(); it++)
	{
		string suffix = getFileSuffix(*it);
		if (suffix == _suffix)
		{
			std::cout << rootDir +*it << std::endl;
			filelists.push_back(rootDir + *it);
		}
	}

	return true;
}

string getFileName(const string& s) {
	char sep = '/';
#ifdef _WIN32
	sep = '\\';
#endif

	size_t i = s.rfind(sep, s.length());
	size_t j = s.find_last_of('.');

	if (i != string::npos) {
		return(s.substr(i + 1, j-i-1));
	}

	return("");
}