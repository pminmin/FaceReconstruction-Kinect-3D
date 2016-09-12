#pragma once

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kninja.hpp"

void expandFace(cv::Rect& r, kn::FaceData& f, float xSize, float ySize);

bool isValidFace(cv::Rect& r, cv::Mat& img);

void drawLandmarks(cv::Mat& img, kn::FaceData& f);

kn::PointCloud getFacePointCloud(kn::FaceData& f, std::vector<UINT16> depth, \
	std::vector<RGBQUAD> color, std::vector<ColorSpacePoint> cop);