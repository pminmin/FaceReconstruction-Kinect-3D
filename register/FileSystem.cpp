#include <io.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include "FileSystem.h"

using namespace std;

bool isExists(const string& path)
{
	if (access(path.c_str(), 0) != -1)
	{
		return true;
	}
	else{
		return false;
	}
}

bool makeDir(const string &dp)
{
	if (isExists(dp))
	{
		return false;
	}
	else
	{
		int ret_code;
		ret_code = mkdir(dp.c_str());
		if (ret_code != 0)
		{
			std::cout << "Couldn't create " << dp << std::endl;
		}
		return true;
	}
}

void getFiles(string path, vector<string>& files)
{
	//文件句柄  
	long   hFile = 0;
	//文件信息  
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之  
			//如果不是,加入列表  
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

bool selectFile(string filepath, string& filename, string& imgname)
{
	vector<string> files;
	vector<string> pcdFiles;
	vector<string> imgFiles;

	if (!isExists(filepath))
	{
		cout << filepath << "doesn't exist!" << endl;
		return false;
	}
	else
	{
		getFiles(filepath, files);
		for (int i = 0; i < files.size(); i++){
			int dotIndex = files[i].find_last_of('.');
			if (dotIndex != -1 && files[i].substr(dotIndex + 1) == "pcd"){
				pcdFiles.push_back(files[i]);
			}
			else{
				imgFiles.push_back(files[i]);
			}
		}

		srand((unsigned)time(0));
		int size = pcdFiles.size();
		int id = rand() % size;
		filename = pcdFiles[id];
		imgname = imgFiles[id];
		return true;
	}
}

string int2str(int &i)
{
	string s;
	stringstream ss(s);
	ss << i;
	return ss.str();
}