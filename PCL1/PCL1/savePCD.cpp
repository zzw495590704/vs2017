#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main() {
	// 生成点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = 2400000; // 设置点云宽度为 2400000
	cloud->height = 1; // 单帧点云
	cloud->is_dense = false; // 设置为非稠密点云

	cloud->points.resize(cloud->width * cloud->height); // 调整点云大小

	// 随机生成点云数据
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	// 保存点云数据到文件
	pcl::PCDWriter writer;
	std::cout << "PointCloud start saved." << std::endl;
	writer.write<pcl::PointXYZ>("output_cloud.pcd", *cloud, false); // 将点云保存为 PCD 文件

	std::cout << "PointCloud saved." << std::endl;

	return 0;
}
