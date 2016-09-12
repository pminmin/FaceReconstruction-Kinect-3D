#pragma once
#include "Kinect.h"
#include "Kinect.Face.h"
#include <opencv2/core/core.hpp>
#include "device.hpp"

namespace kn {

class Point {
public:
  Point()
    : x(0.), y(0.), z(0.), r(0), g(0), b(0) {
  }
  Point(float x, float y, float z, uchar r, uchar g, uchar b)
    : x(x), y(y), z(z), r(r), g(g), b(b) {
  }

public:
  float x, y, z;
  uchar r, g, b;
};

typedef vector<Point> PointCloud;

/*! \breif normalized quaternion */
class Quaternion {
public:
  Quaternion()
    : x(0.), y(0.), z(0.), w(0.) {
  }
  Quaternion(float x, float y, float z, float w)
    : x(x), y(y), z(z), w(w) {
  }
  Quaternion(const Vector4& v)
    : x(v.x), y(v.y), z(v.z), w(v.w) {
  }

  void operator=(const Quaternion& other) {
	  x = other.x;
	  y = other.y;
	  z = other.z;
	  w = other.w;
  }

  Quaternion Conjugate() const;
  Quaternion Multiply(const Quaternion& q) const;
  Quaternion Inverse() const {
    return Conjugate();
  }
  Point Apply(const Point& p) const;
  PointCloud Apply(const PointCloud& cloud) const;
  void ToEuler(float& roll, float& yaw, float& pitch);

public:
  float x, y, z, w;
  const float PI_F = 3.1415926f;
};

class FaceData {
public:
  static const int kLandmarkN = FacePointType_Count;
  /*! \breif is tracked */
  BOOLEAN tracked;
  int id;
  RectI bbox;
  PointF landmarks[kLandmarkN];
  Vector4 rotation;

  FaceData() : tracked(false) {
  }

  inline cv::Rect rect() const {
    return cv::Rect(bbox.Left, bbox.Top, bbox.Right - bbox.Left, bbox.Bottom - bbox.Top);
  }
};

class FaceDriver {
public:
  /*! \breif driver instance */
  static FaceDriver& Driver() {
    static FaceDriver driver_;
    return driver_;
  }
  /*! \breif get face*/
  vector<FaceData> AcquireFaceList(shared_ptr<IBodyFrame>& body_frame);

private:
  FaceDriver();
  ~FaceDriver();

  IFaceFrameSource* face_sources[kBodyCount];
  IFaceFrameReader* face_readers[kBodyCount];

  DISABLE_COPY_AND_ASSIGN(FaceDriver);
};

vector<FaceData> findFaceInDepthSpace(const vector<ColorSpacePoint>& csp, \
                                      const vector<FaceData>& faces);

} // namespace kinect
