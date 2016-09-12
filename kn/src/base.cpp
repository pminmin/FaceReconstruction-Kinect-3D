#include <ctime>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include "base.hpp"
#include "device.hpp"

namespace kn {

// Convert IcolorFrame to color data
// Input: color_frame
// Output: vector<RGBQUAD> rgb
void Convert(shared_ptr<IColorFrame>& color_frame, vector<RGBQUAD>& rgb) {
  const int len = kColorFrameHeight*kColorFrameWidth;
  const int buff_size = len*sizeof(RGBQUAD);
  rgb.resize(len);
  color_frame->CopyConvertedFrameDataToArray(buff_size, reinterpret_cast<BYTE*>(&rgb[0]), \
    ColorImageFormat::ColorImageFormat_Bgra);
}

// Convert IcolorFrame to Mat in OpenCV
// Input: color_frame
// Output: cv::Mat rgb
void Convert(shared_ptr<IColorFrame>& color_frame, cv::Mat& rgb) {
  const int len = kColorFrameHeight*kColorFrameWidth;
  const int buff_size = len*sizeof(RGBQUAD);
  rgb.create(cv::Size(kColorFrameWidth, kColorFrameHeight), CV_8UC4); // CV_8UC4表示4通道阵列，8 bit无符号整数
  color_frame->CopyConvertedFrameDataToArray(buff_size, reinterpret_cast<BYTE*>(rgb.data), \
    ColorImageFormat::ColorImageFormat_Bgra);
}

// Convert IDepthFrame to depth data
// Input: depth_frame
// Output: vector<UINT16> depth
void Convert(shared_ptr<IDepthFrame>& depth_frame, vector<UINT16>& depth) {
  const int len = kDepthFrameHeight*kDepthFrameWidth;
  const int buff_size = len*sizeof(UINT16);
  depth.resize(len);
  depth_frame->CopyFrameDataToArray(len, reinterpret_cast<UINT16*>(&depth[0]));
}

// Map color frame to depth space, (x,y) in color frame = dsp in depth frame
// Input: vector<UINT16> depth
// Output: vector<DepthSpacePoint> dsp (1920*1080)
void MapColorFrameToDepthSpace(vector<UINT16>& depth, vector<DepthSpacePoint>& dsp) {
  Kinect& device = Kinect::Device();
  dsp.resize(kColorPointN);
  HRESULT hr = device.coordinate_mapper()->MapColorFrameToDepthSpace(kDepthPointN, &depth[0], \
																	kColorPointN, &dsp[0]);
}

// Map depth frame to color space, each (x,y) in depth frame = cop in color frame
// Input: vector<UINT16> depth
// Output: vector<ColorSpacePoint> cop (512*484)
void MapDepthFrameToColorSpace(vector<UINT16>& depth, vector<ColorSpacePoint>& cop) {
  Kinect& device = Kinect::Device();
  cop.resize(kDepthPointN);
  HRESULT hr = device.coordinate_mapper()->MapDepthFrameToColorSpace(kDepthPointN, &depth[0], \
                                                                     kDepthPointN, &cop[0]);
}

// Map depth frame to camera space, each (x,y) in depth frame = cap in camera space
// Input: vector<UINT16> depth
// Output: vector<CameraSpacePoint> csp (512*484)
void MapDepthFrameToCameraSpace(vector<UINT16>& depth, vector<CameraSpacePoint>& cap) {
  Kinect& device = Kinect::Device();
  cap.resize(kDepthPointN);
  HRESULT hr = device.coordinate_mapper()->MapDepthFrameToCameraSpace(kDepthPointN, &depth[0], \
                                                                      kDepthPointN, &cap[0]);
}

} // namespace kinect
