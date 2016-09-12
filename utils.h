#pragma once
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include "kninja.hpp"

struct Euler{
	float roll;
	float yaw;
	float pitch;
};

void quaternion_to_euler(const Vector4& quaternion, Euler& euler);

void draw(std::vector<kn::Point>& cloud);

void draw_o(std::vector<kn::Point>& cloud);

void draw_on_window(std::vector<kn::Point>& cloud);

bool isExists(const std::string& path);

bool makeDir(const std::string& dp);

void saveData(cv::Rect& r, kn::FaceData& f, std::string filename, Euler& euler);

void saveDepthData(cv::Rect& r, kn::FaceData& face, std::vector<UINT16>& depth, std::string filename);

// some convert functions

// get color information of depth point(x, y), then assign to depthImage
void findColorOfDepthPoint(cv::Mat& img, std::vector<ColorSpacePoint>& cop, std::vector<RGBQUAD>& color);

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertToPCDdata(kn::PointCloud cloud);