#pragma once
#include <vector>
#include <memory>
#include <Kinect.h>
#include "base.hpp"

namespace kn {

/*!
 * \breif global Kinect singleton
 */
class Kinect {
public:
  /*! \breif get device instance */
  static inline Kinect& Device() {
    static Kinect kinect_;
    return kinect_;
  }
  /*! \breif get original kinect sensor */
  inline IKinectSensor* Sensor() {
    return sensor_;
  }
  /*!
   * \breif acqurie latest frames
   * \note  acqurie frame may return nullptr even if the Kinect device has been stable
   *        you should always checkout the result with nullptr
   */
  shared_ptr<IColorFrame> AcquireLatestColorFrame();
  shared_ptr<IDepthFrame> AcquireLatestDepthFrame();
  shared_ptr<IBodyFrame> AcquireLatestBodyFrame();
  /*! \breif getters */
  inline IColorFrameReader* color_reader() {
    return color_reader_;
  }
  inline IDepthFrameReader* depth_reader() {
    return depth_reader_;
  }
  inline IBodyFrameReader* body_reader() {
    return body_reader_;
  }
  inline ICoordinateMapper* coordinate_mapper() {
    return coordinate_mapper_;
  }

private:
  Kinect();
  ~Kinect();
  /*! \breif kinect sensor */
  IKinectSensor* sensor_;
  /*! \breif color frame reader */
  IColorFrameReader* color_reader_;
  /*! \breif depth frame reader */
  IDepthFrameReader* depth_reader_;
  /*! \breif body frame reader */
  IBodyFrameReader* body_reader_;
  /*! \breif coordiante mapper */
  ICoordinateMapper* coordinate_mapper_;

  DISABLE_COPY_AND_ASSIGN(Kinect);
};

} // namespace kinect
