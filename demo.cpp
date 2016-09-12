#include <cmath>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define NOMINMAX // Visual C++ defines min and max as macros somewhere in windows.h
#include <windows.h>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <gl/GLU.h>

#include "kninja.hpp"
#include "utils.h"
#include "faceUtils.h"
#include "LBF.hpp"

using namespace cv;
using namespace std;
using namespace kn;

#define TIMER_BEGIN { double __time__ = cv::getTickCount();
#define TIMER_NOW   ((double(cv::getTickCount()) - __time__) / cv::getTickFrequency())
#define TIMER_END   }

int gC = 0;
double gA = 0.;
double gB = 0.;

int main(int argc, char* argv[]) {
	// display rotated face
	sf::Window window(sf::VideoMode(640, 480), "OpenGL", sf::Style::Default, sf::ContextSettings(24));
	window.setVerticalSyncEnabled(true);

	glClearColor(0., 0., 0., 1.);
	glClear(GL_COLOR_BUFFER_BIT);
	window.display();

	Kinect& device = Kinect::Device();
	
	FaceDriver& fdriver = FaceDriver::Driver();

	char chari[100];
	int counter = 1;
	
	while (true)
	{
		// 1.get color frame & depth frame & body frame
		shared_ptr<IColorFrame> color_frame = device.AcquireLatestColorFrame();
		shared_ptr<IDepthFrame> depth_frame = device.AcquireLatestDepthFrame();
		shared_ptr<IBodyFrame> body_frame = device.AcquireLatestBodyFrame();
		if (color_frame != nullptr && body_frame != nullptr && depth_frame != nullptr)
		{
			// 2.convert frame to data
			vector<RGBQUAD> color;
			cv::Mat img;
			vector<UINT16> depth;

			Convert(color_frame, color);
			Convert(color_frame, img);
			Convert(depth_frame, depth);

			// 3.map depth frame to color space, each (x,y) in depth frame = cop in color frame
			vector<ColorSpacePoint> cop;
			MapDepthFrameToColorSpace(depth, cop);

			// 4.get color information of depth point (x,y), then assign to depthImage
			Mat depthImage(kDepthFrameHeight, kDepthFrameWidth, CV_8UC3);
			depthImage.setTo(0);
			findColorOfDepthPoint(depthImage, cop, color);

			// 5.get all faces in a color frame
			vector<FaceData> color_faces = fdriver.AcquireFaceList(body_frame);

			Euler euler;
			if (color_faces.size() > 0)
			{
				// 6.expand rect size of faces and then filter faces
				vector<FaceData> face_filtered;
				Rect r_big;
				FaceData colorFace;

				TIMER_BEGIN;
				for (int i = 0; i < color_faces.size(); i++)
				{
					colorFace = color_faces[i];
					expandFace(colorFace.rect(), colorFace, 1.2f, 1.2f);

					if (isValidFace(colorFace.rect(), img))
					{
						rectangle(img, colorFace.rect(), Scalar(0, 0, 255), 3); // draw rect on color image
						drawLandmarks(img, colorFace);
						face_filtered.push_back(colorFace);
					}
					expandFace(r_big, colorFace, 2.0f, 2.0f);
				}

				TIMER_END;
				//printf("Time of acquiring faces is: %.4lf\n", TIMER_NOW);

				// 7.find faces in depth space
				vector<FaceData> depth_faces;
				depth_faces = findFaceInDepthSpace(cop, face_filtered);
				FaceData depthFace;
				for (int i = 0; i < depth_faces.size(); i++)
				{
					depthFace = depth_faces[i];
					if (isValidFace(depthFace.rect(), depthImage))
					{
						rectangle(depthImage, depthFace.rect(), Scalar(0, 0, 255), 2); // draw rect on depth image
						drawLandmarks(depthImage, depthFace);
					}

					// 8.get face point cloud
					kn::PointCloud cloud = getFacePointCloud(depthFace, depth, color, cop);
					//draw_o(cloud);

					// 9.possible rotation
					Quaternion quaternion(depthFace.rotation);
					quaternion.ToEuler(euler.roll, euler.yaw, euler.pitch);
					cloud = quaternion.Inverse().Apply(cloud);
					//draw(cloud);
					draw_on_window(cloud);
					window.display();
				}
				Size si(1920 * 480 / 1080., 480);
				cv::resize(img, img, si);
				imshow("dep", depthImage);
				cv::waitKey(1);
				cv::imshow("kinect", img);
				cv::waitKey(1);
			}
			else {
				cout << "failed" << endl;
			}
		}
		else {
			cout << "get frame failed" << endl;
		}
	}
	return 0;
}