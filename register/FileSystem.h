#ifndef FILESYSTEM_H
#define FILESYSTEM_H

#include <string>
#include <iostream>
#include <vector>
#include <direct.h>

bool isExists(const std::string& path);

bool makeDir(const std::string &dp);

void getFiles(std::string path, std::vector<std::string>& files);

bool selectFile(std::string filepath, std::string& filename, std::string& imgname);

std::string int2str(int &i);

#endif