#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <mutex>

std::mutex cloud_mutex;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
bool new_cloud_available = false;

void generatePointCloud() {
	while (true) {
		// Generate random points
		for (int i = 0; i < 100; ++i) {
			pcl::PointXYZ point;
			point.x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
			point.y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
			point.z = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

			// Lock the mutex before modifying the shared resource
			std::lock_guard<std::mutex> lock(cloud_mutex);
			cloud->points.push_back(point);
		}

		// Set the cloud as new data available
		{
			std::lock_guard<std::mutex> lock(cloud_mutex);
			new_cloud_available = true;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for 1 second
	}
}

void displayPointCloud() {
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);

	while (!viewer->wasStopped()) {
		if (new_cloud_available) {
			// Lock the mutex before accessing the shared resource
			std::lock_guard<std::mutex> lock(cloud_mutex);

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
			viewer->removePointCloud("cloud");
			viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "cloud");

			new_cloud_available = false;
		}

		viewer->spinOnce();
		std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Yield control to other threads
	}
}

int main() {
	std::thread cloud_generator(generatePointCloud);
	std::thread cloud_viewer(displayPointCloud);

	cloud_generator.join();
	cloud_viewer.join();

	return 0;
}