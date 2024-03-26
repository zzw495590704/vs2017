#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

int main_() {
	// �������ƶ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// �趨���ƴ�С
	cloud->width = 2400000;
	cloud->height = 1;  // �����������
	cloud->is_dense = false;  // ��Ϊ�ǳ���

	// ���������������
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f); // �������x����
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f); // �������y����
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f); // �������z����
	}

	// ������Ƶ�PLY�ļ�
	pcl::io::savePLYFile("point_cloud.ply", *cloud);

	std::cout << "���������ѱ���Ϊ point_cloud.ply" << std::endl;

	return 0;
}
