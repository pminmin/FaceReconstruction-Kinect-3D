#include "faceUtils.h"

using namespace kn;
using namespace cv;

void expandFace(cv::Rect& r, kn::FaceData& f, float xSize, float ySize)
{
	r.width = f.bbox.Right - f.bbox.Left;
	r.height = f.bbox.Bottom - f.bbox.Top;
	int center_x = f.bbox.Left + r.width / 2;
	int center_y = f.bbox.Top + r.height / 2;
	r.width *= xSize;
	r.height *= ySize;
	r.x = center_x - r.width / 2;
	r.y = center_y - r.height * 3 / 5;

	f.bbox.Left = r.x; f.bbox.Right = r.x + r.width;
	f.bbox.Top = r.y; f.bbox.Bottom = r.y + r.height;
}

bool isValidFace(cv::Rect& r, cv::Mat& img)
{
	if (r.x < 0 || (r.x + r.width) > img.cols \
		|| r.y < 0 || (r.y + r.height) > img.rows)
	{
		return false;
	}else{
		return true;
	}
}

void drawLandmarks(Mat& img, FaceData& f)
{
	for (int i = 0; i < FaceData::kLandmarkN; i++)
	{
		circle(img, cv::Point(f.landmarks[i].X, f.landmarks[i].Y), \
			2, Scalar(0, 255, 0), -1);
	}
}

kn::PointCloud getFacePointCloud(kn::FaceData& f, vector<UINT16> depth, \
	vector<RGBQUAD> color, vector<ColorSpacePoint> cop)
{
	kn::PointCloud cloud;
	cloud.reserve(kDepthPointN);
	vector<CameraSpacePoint> cap;
	MapDepthFrameToCameraSpace(depth, cap);

	int depthIndexOfNose = f.landmarks[FacePointType_Nose].Y*kDepthFrameWidth + \
		f.landmarks[FacePointType_Nose].X;
	float nose_x = cap[depthIndexOfNose].X;
	float nose_y = cap[depthIndexOfNose].Y;
	float nose_z = cap[depthIndexOfNose].Z;

	for (int y = f.bbox.Top; y <= f.bbox.Bottom; y++) {
		for (int x = f.bbox.Left; x <= f.bbox.Right; x++) {
			kn::Point p;
			int depthIndex = y*kDepthFrameWidth + x;
			p.x = cap[depthIndex].X - nose_x;
			p.y = cap[depthIndex].Y - nose_y;
			p.z = cap[depthIndex].Z - nose_z;

			int xx = cop[depthIndex].X;
			int yy = cop[depthIndex].Y;
			if (isinf(cop[depthIndex].X) || cop[depthIndex].Y < 0) continue;
			if (yy >= kColorFrameHeight || xx >= kColorFrameWidth) continue;
			int colorIndex = yy*kColorFrameWidth + xx;

			p.r = color[colorIndex].rgbRed;
			p.g = color[colorIndex].rgbGreen;
			p.b = color[colorIndex].rgbBlue;

			cloud.push_back(p);
		}
	}
	return cloud;
}