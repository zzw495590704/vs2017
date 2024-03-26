#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

int main_() {
	// 创建点云对象
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 设定点云大小
	cloud->width = 2400000;
	cloud->height = 1;  // 单个无序点云
	cloud->is_dense = false;  // 设为非稠密

	// 生成随机点云数据
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f); // 随机生成x坐标
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f); // 随机生成y坐标
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f); // 随机生成z坐标
	}

	// 保存点云到PLY文件
	pcl::io::savePLYFile("point_cloud.ply", *cloud);

	std::cout << "点云数据已保存为 point_cloud.ply" << std::endl;

	return 0;
}
