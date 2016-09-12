#include <direct.h>
#include <fstream>
#include <iostream>
#include <io.h>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <gl/GLU.h>
#include "utils.h"

using namespace std;
using namespace cv;
using namespace kn;

int fnameIdx = 0;
int fnameIdx0 = 0;

void quaternion_to_euler(const Vector4& quaternion, Euler& euler) {
	float x = quaternion.x;
	float y = quaternion.y;
	float z = quaternion.z;
	float w = quaternion.w;
	
	euler.pitch = atan2(2 * y*w - 2 * x*z, 1 - 2 * y*y - 2 * z*z);
	euler.roll = -atan2(2 * x*w - 2 * y*z, 1 - 2 * x*x - 2 * z*z);
	euler.yaw = -asin(2 * x*y + 2 * z*w);
}

void draw(vector<kn::Point>& cloud) {
	string fname;
	if (fnameIdx == 0) {
		fname = "0.obj";
		fnameIdx = 1;
	}
	else {
		fname = "1.obj";
		fnameIdx = 0;
	}
	FILE* fd = fopen(fname.c_str(), "w");

	int counter = 0;
	int g_idx = 1;
	for (int i = 0; i < cloud.size(); i++) {
		kn::Point& p = cloud[i];
		if (isinf(p.x) || isinf(p.y) || isinf(p.z) || isnan(p.x) || isnan(p.y) || isnan(p.z)) {
			counter++;
			continue;
		}
		fprintf(fd, "v %lf %lf %lf\n", p.x, p.y, p.z);
	}

	//  for (int i = 1; i < height; i++) {
	//    for (int j = 0; i < width; j++) {
	//#define INDEX(x, y) (((x)-1)*width+(y))
	//      fprintf(fd, "f %d %d %d", INDEX(i - 1, j + 1), INDEX(i, j), INDEX(i, j + 1));
	//      fprintf(fd, "f %d %d %d", INDEX(i, j), INDEX(i, j - 1), INDEX(i + 1, j));
	//    }
	//  }

	fclose(fd);
}

void draw_o(vector<kn::Point>& cloud) {
	string fname;
	if (fnameIdx0 == 0) {
		fname = "3.obj";
		fnameIdx0 = 1;
	}
	else {
		fname = "4.obj";
		fnameIdx0 = 0;
	}
	FILE* fd = fopen(fname.c_str(), "w");

	for (int i = 0; i < cloud.size(); i++) {
		kn::Point& p = cloud[i];
		if (isinf(p.x) || isinf(p.y) || isinf(p.z) || isnan(p.x) || isnan(p.y) || isnan(p.z)) {
			continue;
		}
		fprintf(fd, "v %lf %lf %lf\n", p.x, p.y, p.z);
	}

	fclose(fd);
}

void draw_on_window(vector<kn::Point>& cloud) {
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glPushMatrix();
	glClear(GL_COLOR_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0., 0., 0.5, 0., 0., 1., 0., 1., 0.);
	//gluLookAt(0, 0, 0, 0, 0, 0, 0, 0, 1);
	glPointSize(2.);
	glBegin(GL_POINTS);
	for (int i = 0; i < cloud.size(); i++) {
		kn::Point p = cloud[i];
		if (isinf(p.x) || isinf(p.y) || isinf(p.z) || isnan(p.x) || isnan(p.y) || isnan(p.z)) continue;
		glColor3f(cloud[i].r / 255., cloud[i].g / 255., cloud[i].b / 255.);
		//glColor3f(1., 1., 1.);
		glVertex3f(cloud[i].x, cloud[i].y, cloud[i].z);
	}
	glEnd();
	glFlush();
	glPopMatrix();
	glPopAttrib();
}

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
		cout << "文件夹已存在" << endl;
		return false;
	}
	else
	{
		int ret_code;
		ret_code = mkdir(dp.c_str());
		if (ret_code != 0)
		{
			cout << "Couldn't create " << dp << endl;
		}
		return true;
	}
}

void saveData(cv::Rect& r, kn::FaceData& f, string filename, Euler& euler)
{
	ofstream file(filename);
	file << euler.roll << " " << euler.yaw << " " << euler.pitch << "\n";
	for (int i = 0; i < kn::FaceData::kLandmarkN; i++){
		file << f.landmarks[i].X - r.x << " " << f.landmarks[i].Y - r.y << "\n";
	}
}

void saveDepthData(cv::Rect& r, kn::FaceData& face, vector<UINT16>& depth, string filename)
{
	ofstream file(filename);
	for (int i = 0; i < kn::FaceData::kLandmarkN; i++){
		file << face.landmarks[i].X - r.x << " " << face.landmarks[i].Y - r.y << "\n";
	}
	file << r.width << " " << r.height << "\n";
	for (int y = r.y; y < r.y + r.height; y++){
		for (int x = r.x; x < r.x + r.width; x++){
			size_t idx = y * kn::kDepthFrameWidth + x;
			file << depth[idx] << "\n";
		}
	}
	file.close();
}

void findColorOfDepthPoint(Mat& img, vector<ColorSpacePoint>& cop, vector<RGBQUAD>& color)
{
	for (int y = 0; y < kDepthFrameHeight; y++)
	{
		for (int x = 0; x < kDepthFrameWidth; x++)
		{
			Vec3b p;
			int depthIndex = y*kDepthFrameWidth + x;
			if (isinf(cop[depthIndex].X) || cop[depthIndex].Y < 0) continue;
			int xx = cop[depthIndex].X;
			int yy = cop[depthIndex].Y;
			int colorIndex = yy*kColorFrameWidth + xx;
			if (yy >= kColorFrameHeight || xx >= kColorFrameWidth) continue;
			p[0] = color[colorIndex].rgbBlue;
			p[1] = color[colorIndex].rgbGreen;
			p[2] = color[colorIndex].rgbRed;
			img.at<Vec3b>(y, x) = p;
		}
	}
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertToPCDdata(kn::PointCloud cloud)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGBA>(0, 0));
	pointcloud->is_dense = false;
	pointcloud->clear();
	pointcloud->reserve(kn::kDepthPointN);
	for (int i = 0; i < cloud.size(); i++)
	{
		pcl::PointXYZRGBA point;
		point.x = cloud[i].x;
		point.y = cloud[i].y;
		point.z = cloud[i].z;
		point.r = cloud[i].r;
		point.g = cloud[i].g;
		point.b = cloud[i].b;
		pointcloud->push_back(point);
	}
	return pointcloud;
}

