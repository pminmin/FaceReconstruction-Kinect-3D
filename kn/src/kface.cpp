#include <cmath>
#include <iostream>
#include "kface.hpp"

namespace kn {

Quaternion Quaternion::Conjugate() const {
  float x_ = -x;
  float y_ = -y;
  float z_ = -z;
  float w_ = w;
  return Quaternion(x_, y_, z_, w_);
}

Quaternion Quaternion::Multiply(const Quaternion& q) const {
  float x_ = w*q.x + x*q.w + y*q.z - z*q.y;
  float y_ = w*q.y + y*q.w + z*q.x - x*q.z;
  float z_ = w*q.z + z*q.w + x*q.y - y*q.x;
  float w_ = w*q.w - x*q.x - y*q.y - z*q.z;
  return Quaternion(x_, y_, z_, w_);
}

Point Quaternion::Apply(const Point& p) const  {
  const Quaternion qc = Conjugate();
  Quaternion q1(p.x, p.y, p.z, 0.);
  Quaternion q2 = q1.Multiply(qc);
  q1 = Multiply(q2);
  Point rp;
  rp.x = q1.x;
  rp.y = q1.y;
  rp.z = q1.z;
  rp.r = p.r;
  rp.g = p.g;
  rp.b = p.b;
  return rp;
}

PointCloud Quaternion::Apply(const PointCloud& cloud) const {
  PointCloud cloud_(cloud.size());
  for (int i = 0; i < cloud.size(); i++) {
    cloud_[i] = Apply(cloud[i]);
  }
  return cloud_;
}

void Quaternion::ToEuler(float& roll, float& yaw, float& pitch)
{
	roll = atan2f(2.f * (x * y + w * z), w * w + x * x - y * y - z * z) / PI_F * 180.F;
	yaw = asinf(2.f * (w * y - x * z)) / PI_F * 180.f;
	pitch = atan2f(2.f * (y * z + w * z), w * w - x * x - y * y + z * z) / PI_F * 180.f;
}

// define the face frame features required to be computed by this application
static const DWORD kFaceFrameFeatures =
    FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
  | FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
  | FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
  | FaceFrameFeatures::FaceFrameFeatures_Happy
  | FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
  | FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
  | FaceFrameFeatures::FaceFrameFeatures_MouthOpen
  | FaceFrameFeatures::FaceFrameFeatures_MouthMoved
  | FaceFrameFeatures::FaceFrameFeatures_LookingAway
  | FaceFrameFeatures::FaceFrameFeatures_Glasses
  | FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;

FaceDriver::FaceDriver() {
  Kinect& device = Kinect::Device();
  HRESULT hr;
  for (int i = 0; i < kBodyCount; i++) {
    hr = CreateFaceFrameSource(device.Sensor(), 0, kFaceFrameFeatures, &face_sources[i]);
    hr = face_sources[i]->OpenReader(&face_readers[i]);
  }
}

FaceDriver::~FaceDriver() {
  for (int i = 0; i < kBodyCount; i++) {
    face_sources[i]->Release();
    face_readers[i]->Release();
  }
}

vector<FaceData> FaceDriver::AcquireFaceList(shared_ptr<IBodyFrame>& body_frame) {
  HRESULT hr;
  vector<FaceData> faces;
  faces.reserve(kBodyCount);

  IBody* bodies[kBodyCount] = { 0 };
  hr = body_frame->GetAndRefreshBodyData(kBodyCount, bodies);
  if (!SUCCEEDED(hr)) {
    for (int i = 0; i < kBodyCount; i++) {
      SafeRelease(bodies[i]);
    }
    return faces;
  }

  for (int i = 0; i < kBodyCount; i++) {
    if (!bodies[i]) continue; // no body

    UINT64 id = 0;
    bodies[i]->get_TrackingId(&id);
    face_sources[i]->put_TrackingId(id); // get and put id

    IFaceFrame* face_frame = nullptr;
    hr = face_readers[i]->AcquireLatestFrame(&face_frame); // get face frame

    BOOLEAN tracked = false;
    if (face_frame != nullptr && SUCCEEDED(hr)) {
      hr = face_frame->get_IsTrackingIdValid(&tracked);
      if (SUCCEEDED(hr) && tracked) {
        FaceData face;
        face.tracked = tracked;

        IFaceFrameResult* face_result = nullptr;
        hr = face_frame->get_FaceFrameResult(&face_result);

        if (face_result != nullptr && SUCCEEDED(hr)) {
          hr = face_result->get_FaceBoundingBoxInColorSpace(&face.bbox);
          if (SUCCEEDED(hr)) {
            hr = face_result->GetFacePointsInColorSpace(FacePointType_Count, face.landmarks); 
          }
          if (SUCCEEDED(hr)) {
            hr = face_result->get_FaceRotationQuaternion(&face.rotation);
          }
          // final filter
          bool flag = true;
          if (face.bbox.Left == 0 && face.bbox.Right == 0 && face.bbox.Top == 0 && face.bbox.Bottom == 0) {
            flag = false;
          }
          if (!flag) continue;
          for (int j = 0; j < FaceData::kLandmarkN; j++) {
            if (face.landmarks[j].X<face.bbox.Left || face.landmarks[j].X>face.bbox.Right) {
              flag = false;
              break;
            }
            if (face.landmarks[j].Y<face.bbox.Top || face.landmarks[j].Y>face.bbox.Bottom) {
              flag = false;
              break;
            }
          }
          if (!flag) continue;
          face.id = id;
          faces.push_back(face);
        }

        SafeRelease(face_result);
      }
    }
    SafeRelease(face_frame);
  }

  for (int i = 0; i < kBodyCount; i++) {
    SafeRelease(bodies[i]);
  }
  return faces;
}

static inline float distance(float x, float y, PointF p) {
  float d = sqrt(pow(x - p.X, 2) + pow(y - p.Y, 2));
  return d;
}

// Find faces in depth space, cause face detection bases on color space in SDK
// Input: vector<ColorSpacePoint> cop (512*484)
// Input: vector<FaceData> faces in color space
// Return: vector<FaceData> in depth space
vector<FaceData> findFaceInDepthSpace(const vector<ColorSpacePoint>& cop, \
                                      const vector<FaceData>& faces) {
  vector<FaceData> faces_(faces); // copy data
  for (int i = 0; i < faces_.size(); i++) {
    FaceData& f = faces_[i];
    int left, right, top, bottom;
    left = kDepthFrameWidth; right = 0;
    top = kDepthFrameHeight; bottom = 0;
    PointF landmarks[FaceData::kLandmarkN];
    vector<float> dist(FaceData::kLandmarkN, std::numeric_limits<float>::max()); // 5 float values, each initialized with max(float)

    for (int y = 0; y < kDepthFrameHeight; y++) {
      for (int x = 0; x < kDepthFrameWidth; x++) {
        int depthIndex = y*kDepthFrameWidth + x;
		if (isinf(cop[depthIndex].X) || isinf(cop[depthIndex].Y)) continue;
		float xx = cop[depthIndex].X;
		float yy = cop[depthIndex].Y;
        if (xx >= f.bbox.Left && xx <= f.bbox.Right && yy >= f.bbox.Top && yy <= f.bbox.Bottom) {
          left = std::min(left, x); right = std::max(right, x); // find face bounding box in depth space
          top = std::min(top, y); bottom = std::max(bottom, y);
          for (int j = 0; j < FaceData::kLandmarkN; j++) {
            if (distance(xx, yy, f.landmarks[j]) < dist[j]) {
              dist[j] = distance(xx, yy, f.landmarks[j]);
              landmarks[j].X = x; // find landmarks in depth space
              landmarks[j].Y = y;
            }
          }
        }
      }
    }

    f.bbox.Left = left; f.bbox.Right = right;
    f.bbox.Top = top; f.bbox.Bottom = bottom;
    for (int j = 0; j < FaceData::kLandmarkN; j++) {
      f.landmarks[j] = landmarks[j];
    }
  }

  return faces_;
}

} // namespace kinect
