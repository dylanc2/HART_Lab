#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <math.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <eigen_conversions/eigen_msg.h>
#include <cloud_filtering/Float64MultiArrayStamped.h>
#include <cloud_filtering/PrincipalComponentStamped.h>
#include <cloud_filtering/ContactStateEstimationStamped.h>
#include <iostream>
#include <string>
#include <pcl_ros/point_cloud.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/common/pca.h>

ros::Publisher filtered_membrane_pub;
ros::Publisher deformed_cloud_pub;
ros::Publisher first_pc_pub;
ros::Publisher state_estimation_pub;
ros::Publisher centroid_pub;
ros::Publisher pca_evals_pub;
ros::Publisher pca_evecs_custom_pub;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudColor;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

int width;
int height;
const float max_depth = 0.20;
const float min_depth = 0.11;

bool SET_PARAMS = true;
bool first_loop = true;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr undeformed_filtered_membrane(new pcl::PointCloud<pcl::PointXYZRGB>);
const double deformation_threshold = 0.015;

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(255, 255, 255);

	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "centroid");
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "deformed");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "centroid");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "deformed");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.4, 0.6, "deformed");

	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->removeCoordinateSystem();
	return (viewer);
}

float *getColorBar(float s)
{
	static float rgb_vals[3];
	uint8_t r = 0;
	uint8_t g = 0;
	uint8_t b = 0;

	float a = (1 - s) / 0.25;
	int X = floor(a);
	float Y = floor(255 * (a - X));

	switch (X)
	{
	case 0:
		r = 255;
		g = Y;
		b = 0;
		break;
	case 1:
		r = 255 - Y;
		g = 255;
		b = 0;
		break;
	case 2:
		r = 0;
		g = 255;
		b = Y;
		break;
	case 3:
		r = 0;
		g = 255 - Y;
		b = 255;
		break;
	case 4:
		r = 0;
		g = 0;
		b = 255;
		break;
	}

	rgb_vals[0] = r;
	rgb_vals[1] = g;
	rgb_vals[2] = b;
	return rgb_vals;
}

void setColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_membrane)
{
	for (int c = 0; c < width; c++)
	{ //224, width of point cloud
		for (int r = 0; r < height; r++)
		{ //171, height
			if (pcl::isFinite(filtered_membrane->at(c, r)) && !std::isnan(filtered_membrane->at(c, r).z))
			{
				float cbar_val = (filtered_membrane->at(c, r).z - min_depth) / (max_depth - min_depth);
				float *rgb_vals = getColorBar(cbar_val);
				filtered_membrane->at(c, r).r = rgb_vals[0];
				filtered_membrane->at(c, r).g = rgb_vals[1];
				filtered_membrane->at(c, r).b = rgb_vals[2];
			}
		}
	}
}

void cloud_cb(const PointCloud::ConstPtr &cloud_msg)
{

	// Get width and height of point cloud
	if (first_loop)
	{
		std::cout << "Getting dimensions of point cloud" << std::endl;
		width = cloud_msg->width;
		height = cloud_msg->height;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_membrane(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr deformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::copyPointCloud(*cloud_msg, *filtered_membrane);
	pcl::copyPointCloud(*cloud_msg, *deformed_cloud);

	setColor(filtered_membrane);

	// Filtering object
	pcl::ExtractIndices<pcl::PointXYZRGB> eifilter(true);
	pcl::PointIndices::Ptr membrane_indices(new pcl::PointIndices());
	pcl::PointIndices::Ptr deformed_indices(new pcl::PointIndices());

	// Get membrane indices and deformed indices
	for (size_t i = 0; i < cloud_msg->points.size(); i++)
	{
		if ((pow(cloud_msg->points[i].x, 2) + pow(cloud_msg->points[i].y, 2)) < pow(0.05, 2))
		{
			membrane_indices->indices.push_back(i);
			if (!first_loop &&
				fabs(undeformed_filtered_membrane->points[i].z - cloud_msg->points[i].z) > deformation_threshold)
			{
				deformed_indices->indices.push_back(i);
			}
		}
	}

	eifilter.setInputCloud(filtered_membrane);	// cloud to filter
	eifilter.setIndices(membrane_indices);		// indices to include
	eifilter.filterDirectly(filtered_membrane); // modify filtered_membrane to only include membrane_indices

	if (first_loop)
	{
		std::cout << "Saving undeformed filtered membrane point cloud" << std::endl;
		pcl::copyPointCloud(*filtered_membrane, *undeformed_filtered_membrane);
		first_loop = false;
	}

	eifilter.setInputCloud(deformed_cloud);
	eifilter.setIndices(deformed_indices);
	eifilter.filterDirectly(deformed_cloud);

	pcl::PointXYZRGB centroid;
	pcl::computeCentroid(*deformed_cloud, centroid);
	centroid_cloud->push_back(centroid);

	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*deformed_cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	computeCovarianceMatrixNormalized(*deformed_cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

	Eigen::Vector3f e1, e2, e3, evals;
	e1 = eigenVectorsPCA.col(0);
	e2 = eigenVectorsPCA.col(1);
	e3 = eigenVectorsPCA.col(2);
	evals = eigen_solver.eigenvalues();

	Eigen::Vector3f evecs[3] = {e1, e2, e3};

	float max_eval = -INFINITY;
	int max_index = 0;

	for (int i = 0; i < 3; i++)
	{
		if (evals(i) > max_eval)
		{
			max_eval = evals(i);
			max_index = i;
		}
	}

	Eigen::Vector3f pc = evecs[max_index];

	// Display on viewer
	viewer->updatePointCloud(filtered_membrane, "cloud");
	viewer->updatePointCloud(centroid_cloud, "centroid");
	viewer->updatePointCloud(deformed_cloud, "deformed");

	float k = 100; // Scale vectors to make them visible on viewer
	pcl::PointXYZ p1, p2, p3;
	p1.x = centroid.x + 2.5 * k * evals(0) * e1(0);
	p1.y = centroid.y + 2.5 * k * evals(0) * e1(1);
	p1.z = centroid.z + 2.5 * k * evals(0) * e1(2);
	p2.x = centroid.x + k * evals(1) * e2(0);
	p2.y = centroid.y + k * evals(1) * e2(1);
	p2.z = centroid.z + k * evals(1) * e2(2);
	p3.x = centroid.x + k * evals(2) * e3(0);
	p3.y = centroid.y + k * evals(2) * e3(1);
	p3.z = centroid.z + k * evals(2) * e3(2);

	viewer->removeShape("e1_line");
	viewer->addLine<pcl::PointXYZRGB, pcl::PointXYZ>(centroid, p1, 1, 0, 0, "e1_line");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 25, "e1_line");

	viewer->removeShape("e2_line");
	viewer->addLine<pcl::PointXYZRGB, pcl::PointXYZ>(centroid, p2, 1, 0, 0, "e2_line");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 25, "e2_line");

	viewer->removeShape("e3_line");
	viewer->addLine<pcl::PointXYZRGB, pcl::PointXYZ>(centroid, p3, 1, 0, 0, "e3_line");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 25, "e3_line");

	// Publish topics
	filtered_membrane_pub.publish(filtered_membrane);
	deformed_cloud_pub.publish(deformed_cloud);

	geometry_msgs::Vector3Stamped pca_evals_msg;
	pca_evals_msg.vector.x = evals(0);
	pca_evals_msg.vector.y = evals(1);
	pca_evals_msg.vector.z = evals(2);
	pca_evals_msg.header.stamp = ros::Time::now();
	pca_evals_pub.publish(pca_evals_msg);

	std_msgs::Float64MultiArray pca_evecs_msg;
	tf::matrixEigenToMsg(eigenVectorsPCA, pca_evecs_msg);

	cloud_filtering::Float64MultiArrayStamped pca_evecs_custom_msg;
	pca_evecs_custom_msg.header.stamp = ros::Time::now();
	pca_evecs_custom_msg.data = pca_evecs_msg.data;
	pca_evecs_custom_pub.publish(pca_evecs_custom_msg);

	geometry_msgs::Vector3Stamped centroid_msg;
	centroid_msg.vector.x = centroid.x;
	centroid_msg.vector.y = centroid.y;
	centroid_msg.vector.z = centroid.z;
	centroid_msg.header.stamp = ros::Time::now();
	centroid_pub.publish(centroid_msg);

	cloud_filtering::PrincipalComponentStamped first_pc_msg;
	first_pc_msg.principal_component[0] = pc(0);
	first_pc_msg.principal_component[1] = pc(1);
	first_pc_msg.principal_component[2] = pc(2);
	first_pc_msg.eigenvalue = max_eval;
	first_pc_msg.header.stamp = ros::Time::now();
	first_pc_pub.publish(first_pc_msg);

	cloud_filtering::ContactStateEstimationStamped state_estimation_msg;
	state_estimation_msg.orientation[0] = pc(0);
	state_estimation_msg.orientation[1] = pc(1);
	state_estimation_msg.orientation[2] = pc(2);
	state_estimation_msg.position[0] = centroid.x;
	state_estimation_msg.position[1] = centroid.y;
	state_estimation_msg.position[2] = centroid.z;
	state_estimation_msg.header.stamp = ros::Time::now();
	state_estimation_pub.publish(state_estimation_msg);
}

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "filtering_node");
	ros::NodeHandle nh;

	if (SET_PARAMS)
	{
		ros::Publisher use_case_pub = nh.advertise<std_msgs::String>("/use_case", 1);
		ros::Publisher max_filter_pub = nh.advertise<std_msgs::Float32>("/max_filter", 1);
		ros::Publisher min_filter_pub = nh.advertise<std_msgs::Float32>("/min_filter", 1);
		ros::Publisher exposure_pub = nh.advertise<std_msgs::UInt32>("/expo_time", 1);

		sleep(5);

		std_msgs::String mode_msg;
		std::stringstream ss;
		ss << "MODE_5_45FPS_500";
		mode_msg.data = ss.str();
		use_case_pub.publish(mode_msg);

		std_msgs::Float32 max_filter_msg;
		max_filter_msg.data = max_depth;
		max_filter_pub.publish(max_filter_msg);

		std_msgs::Float32 min_filter_msg;
		min_filter_msg.data = min_depth;
		min_filter_pub.publish(min_filter_msg);

		std_msgs::UInt32 expo_time_msg;
		expo_time_msg.data = 50;
		exposure_pub.publish(expo_time_msg);
		sleep(3);

		ros::spinOnce();
	}

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe<PointCloud>("/royale_camera_driver/point_cloud", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	filtered_membrane_pub = nh.advertise<PointCloud>("/filtered_membrane", 1);
	deformed_cloud_pub = nh.advertise<PointCloud>("/deformed_cloud", 1);
	pca_evals_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/pca_evals", 1);
	pca_evecs_custom_pub = nh.advertise<cloud_filtering::Float64MultiArrayStamped>("/pca_evecs", 1);
	centroid_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/centroid", 1);
	first_pc_pub = nh.advertise<cloud_filtering::PrincipalComponentStamped>("/first_pc", 1);
	state_estimation_pub = nh.advertise<cloud_filtering::ContactStateEstimationStamped>("/contact_state_estimation", 1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::cout << "Loading live stream\n\n";

	basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;

	viewer = simpleVis(point_cloud_ptr);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		ros::spinOnce();
	}
}

// dsalkfjdsaf