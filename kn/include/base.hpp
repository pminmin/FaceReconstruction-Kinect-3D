#pragma once
#include <memory>
#include <vector>
#include <Kinect.h>
#include <opencv2/core/core.hpp>

// disable the copy and assignment operator for a class.
#define DISABLE_COPY_AND_ASSIGN(classname) \
private:\
  classname(const classname&) = delete;\
  classname& operator=(const classname&) = delete

namespace kn {

using std::vector;
using std::shared_ptr;

// const variables
const int kDepthFrameWidth = 512;
const int kDepthFrameHeight = 424;
const int kColorFrameWidth = 1920;
const int kColorFrameHeight = 1080;
const int kDepthPointN = kDepthFrameWidth*kDepthFrameHeight;
const int kColorPointN = kColorFrameWidth*kColorFrameHeight;
const int kBodyCount = BODY_COUNT;

// convert all types
void Convert(shared_ptr<IColorFrame>& color_frame, vector<RGBQUAD>& rgb);
void Convert(shared_ptr<IColorFrame>& color_frame, cv::Mat& rgb);
void Convert(shared_ptr<IDepthFrame>& depth_frame, vector<UINT16>& depth);
void Map(vector<UINT16>& depth, vector<CameraSpacePoint>& cameraspacepoint);
void Map(vector<UINT16>& depth, vector<DepthSpacePoint>& depthspacepoint);
void Map(vector<UINT16>& depth, vector<ColorSpacePoint>& colorspacepoint);
void MapDepthFrameToDepthSpace(vector<UINT16>& depth, vector<DepthSpacePoint>& dsp);
void MapDepthFrameToColorSpace(vector<UINT16>& depth, vector<ColorSpacePoint>& csp);
void MapDepthFrameToCameraSpace(vector<UINT16>& depth, vector<CameraSpacePoint>& csp);

// safe release
template<typename Interface>
void SafeRelease(Interface* ptr) {
  if (ptr != nullptr) ptr->Release();
}

} // namespace kinect
