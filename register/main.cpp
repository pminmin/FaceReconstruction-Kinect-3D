#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "FileSystem.h"
#include "parameters.h"

using namespace std;
using namespace pcl;
using pcl::visualization::PointCloudColorHandlerCustom;

#define TIMER_BEGIN { double __time__ = cv::getTickCount();
#define TIMER_NOW   ((double(cv::getTickCount()) - __time__) / cv::getTickFrequency())
#define TIMER_END   }

pcl::visualization::PCLVisualizer *p;
int vp(0);

void showClouds(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out)
{
	p->removePointCloud("cloud");
	PointCloudColorHandlerCustom<pcl::PointXYZRGBA> out(cloud_out, 0, 255, 0);
	p->addPointCloud(cloud_out, out, "cloud", vp);
	p->spin();
}

void reg(string source_file, string target_file, string out_file, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr full_cloud)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGBA>());

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

	bool loadFail = true;
	if (pcl::io::loadPCDFile(source_file, *source) == 0 && pcl::io::loadPCDFile(target_file, *target) == 0)
	{
		loadFail = false;
	}
	if (loadFail)
	{
		printf("Could not read pcd files!\n");
		return;
	}

	//移除离群点
	int nr_k = 50;
	//int stddev_mult = 0.8;
	int stddev_mult = 1.0;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;//统计离群点方法
	sor.setInputCloud(source);
	sor.setMeanK(nr_k);
	sor.setStddevMulThresh((double)stddev_mult);
	sor.filter(*source_cloud);
	sor.setInputCloud(target);
	sor.filter(*target_cloud);

	printf("*************** Point clouds registration ***************\n");

	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

	//ICP精配准
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_source_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	IterativeClosestPoint<PointXYZRGBA, PointXYZRGBA> icp;
	icp.setInputCloud(source_cloud);//需要匹配的点云
	icp.setInputTarget(target_cloud);//基准点云

	// 设置ICP参数
	icp.setMaxCorrespondenceDistance(ICP_MAX_DIST);//忽略在此距离之外的点
	icp.setMaximumIterations(ICP_MAX_ITERATIONS);//第1个约束，迭代次数
	icp.setTransformationEpsilon(ICP_TRANSFORMATION_EPSILON);//第2个约束，这个值一般设为1e-6或者更小
	icp.setEuclideanFitnessEpsilon(ICP_EUCLIDEAN_FITNESS_EPSILON);//第3个约束，前后两次迭代误差的差值

	icp.align(*new_source_cloud);//输出配准后点云	

	transformation = icp.getFinalTransformation();
	printf("\n");
	std::cout << "has converged: " << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	printf("\n");
	pcl::transformPointCloud(*source_cloud, *new_source_cloud, transformation);

	//合成点云
	*full_cloud += *target_cloud;
	*full_cloud += *new_source_cloud;

	//在右侧显示配准点云
	pcl::io::savePCDFile(out_file, *full_cloud);
}

int main(int argc, char* argv[])
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr lf(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rf(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>());
	
	string personName = argv[1];
	cout << personName << endl;

	if (!isExists(personName)){
		cout << "No such person!" << endl;
		return -1;
	}

	string dirName = personName + "//";
	vector<string> files;
	vector<string> pcdFiles;

	getFiles(personName, files);
	for (int i = 0; i < files.size(); i++){
		int dotIndex = files[i].find_last_of('.');
		if (dotIndex != -1 && files[i].substr(dotIndex + 1) == "pcd"){
			pcdFiles.push_back(files[i]);  // get l, f, r pcd files
		}
	}

	char chari[100];
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	sprintf_s(chari, "%02d_%02d", sys.wMonth, sys.wDay);

	string file_1 = dirName + personName + "_lf_" + chari + ".pcd";
	string file_2 = dirName + personName + "_rf_" + chari + ".pcd";
	string out = dirName + personName + "_out_" + chari + ".pcd";
	
	reg(pcdFiles[0], pcdFiles[1], file_1, lf); // get lf.pcd file
	reg(pcdFiles[2], pcdFiles[1], file_2, rf); // get rf.pcd file
	reg(file_1, file_2, out, cloud_out); // get final pcd file

	printf("cloud_out size: %d\n", cloud_out->size());

	p = new pcl::visualization::PCLVisualizer("Cloud Viewer");
	p->createViewPort(0.0, 0, 1.0, 1.0, vp);
	showClouds(cloud_out);

	// Whether show RGB imformation
	/*pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> color(cloud_out);
	viewer.addPointCloud(cloud_out, color);
	while (!viewer.wasStopped()){
		viewer.spinOnce();
	}*/
	return 0;
}
