#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <mutex>
#include <chrono>

std::mutex cloud_mutex;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
bool new_cloud_available = false;

void generatePointCloud() {
	while (true) {
		// Generate random points
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (int i = 0; i < 1600; ++i) {
			pcl::PointXYZ point;
			point.x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
			point.y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
			point.z = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
			new_cloud->points.push_back(point);
		}

		// Lock the mutex before modifying the shared resource
		std::lock_guard<std::mutex> lock(cloud_mutex);
		*cloud += *new_cloud;

		// Set the cloud as new data available
		new_cloud_available = true;

		std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for 1 second
	}
}

void displayPointCloud() {
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	// ���忪ʼʱ���ͽ���ʱ���
	std::chrono::time_point<std::chrono::steady_clock> start, end;
	// ��ʼ��ʱ
	start = std::chrono::steady_clock::now();
	while (!viewer->wasStopped()) {
		if (new_cloud_available) {
			// Lock the mutex before accessing the shared resource
			std::lock_guard<std::mutex> lock(cloud_mutex);
			std::time_t now = std::time(nullptr);
			std::string cloud_id = "cloud_" + std::to_string(now);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
			viewer->removePointCloud(cloud_id); // Remove existing cloud with unique ID
			viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, cloud_id);
			new_cloud_available = false;

			// ������ʱ
			end = std::chrono::steady_clock::now();
			// ����ѭ��ִ��ʱ��
			std::chrono::duration<double> elapsed_seconds = end - start;
			std::cout << "���Ƹ���" << cloud->size() << "   ��ʱ��" << elapsed_seconds.count() << std::endl;
			start = end; //��ȡϵͳ��ǰʱ��
		}

		viewer->spinOnce();
		std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Yield control to other threads
	}
}

int main_() {
	std::thread cloud_generator(generatePointCloud);
	std::thread cloud_viewer(displayPointCloud);

	cloud_generator.join();
	cloud_viewer.join();

	return 0;
}
