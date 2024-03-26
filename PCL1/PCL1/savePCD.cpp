#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main() {
	// ���ɵ�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = 2400000; // ���õ��ƿ��Ϊ 2400000
	cloud->height = 1; // ��֡����
	cloud->is_dense = false; // ����Ϊ�ǳ��ܵ���

	cloud->points.resize(cloud->width * cloud->height); // �������ƴ�С

	// ������ɵ�������
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	// ����������ݵ��ļ�
	pcl::PCDWriter writer;
	std::cout << "PointCloud start saved." << std::endl;
	writer.write<pcl::PointXYZ>("output_cloud.pcd", *cloud, false); // �����Ʊ���Ϊ PCD �ļ�

	std::cout << "PointCloud saved." << std::endl;

	return 0;
}
