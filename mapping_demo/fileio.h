#pragma once

#include <string>
#include <vector>
#include <string.h>

bool findCorrespondingFiles(const std::string &rootDir,
	std::vector<std::string > &filelists, const std::string &_suffix = "png");

std::string getFileName(const std::string& s);