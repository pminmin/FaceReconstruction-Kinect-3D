#include "device.hpp"
#include <iostream>

// register a deleter for Kinect Interface
#define MakeShared(classname, pointer) \
  std::shared_ptr<classname>(pointer, [](classname* obj) { \
    SafeRelease(obj); \
  })

namespace kn {

Kinect::Kinect() {
  GetDefaultKinectSensor(&sensor_);
  HRESULT hr = sensor_->Open();
  
  IColorFrameSource* color_source;
  hr = sensor_->get_ColorFrameSource(&color_source);
  hr = color_source->OpenReader(&color_reader_);
  color_source->Release();

  IDepthFrameSource* depth_source;
  hr = sensor_->get_DepthFrameSource(&depth_source);
  hr = depth_source->OpenReader(&depth_reader_);
  depth_source->Release();

  IBodyFrameSource* body_source;
  hr = sensor_->get_BodyFrameSource(&body_source);

  hr = body_source->OpenReader(&body_reader_);
  body_source->Release();

  hr = sensor_->get_CoordinateMapper(&coordinate_mapper_);
}

Kinect::~Kinect() {
  color_reader_->Release();
  depth_reader_->Release();
  body_reader_->Release();
  coordinate_mapper_->Release();
  sensor_->Close();
  sensor_->Release();
}

shared_ptr<IColorFrame> Kinect::AcquireLatestColorFrame() {
  IColorFrame* color_frame = nullptr;
  HRESULT hr = color_reader_->AcquireLatestFrame(&color_frame);

  return MakeShared(IColorFrame, color_frame);
}

shared_ptr<IDepthFrame> Kinect::AcquireLatestDepthFrame() {
  IDepthFrame* depth_frame = nullptr;
  HRESULT hr = depth_reader_->AcquireLatestFrame(&depth_frame);

  return MakeShared(IDepthFrame, depth_frame);
}

shared_ptr<IBodyFrame> Kinect::AcquireLatestBodyFrame() {
  IBodyFrame* body_frame = nullptr;
  HRESULT hr = body_reader_->AcquireLatestFrame(&body_frame);

  return MakeShared(IBodyFrame, body_frame);
}

} // namespace kinect
