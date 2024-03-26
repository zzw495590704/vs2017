#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <thread> // ���������̵߳�ͷ�ļ�
int main_()
{
	// �������ƶ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// ��ȡPLY�ļ�
	if (pcl::io::loadPLYFile<pcl::PointXYZ>("./data/loop-path-fix.ply", *cloud) == -1) //* ����PLY��ʽ�ĵ����ļ������ʧ�ܣ�����-1
	{
		PCL_ERROR("�޷�����PLY�ļ�.\n");
		return (-1);
	}
	std::cout << "���صĵ��ư��� " << cloud->size() << " ���㡣" << std::endl;

	// �������������²����˲���
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);

	// ������������ĳߴ磬����ֻ�� Y ������г���
	sor.setLeafSize(0.1f, 0.1f, 0.1f); // ����Y��������سߴ�Ϊ0.1�ף���������Ϊ0.01��

	// ִ���˲��������ƽ��г���
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
	sor.filter(*filtered);
	std::cout << "�����еĵ������Ϊ��" << filtered->size() << std::endl;
	// ���ӻ���ʾ������ĵ���
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	viewer->setBackgroundColor(0, 0, 0); // ���ñ�����ɫΪ��ɫ
	viewer->addPointCloud<pcl::PointXYZ>(filtered, "cloud"); // ��ӵ��Ƶ����ӻ�����
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud"); // ���õ�����ʾ��С

	// ��ʾ���ӻ����ڣ�ֱ�����ڹر�
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100); // ʹ���ӻ����ڱ��ֻ�Ծ״̬
		std::this_thread::sleep_for(std::chrono::milliseconds(100)); // ����һ��ʱ�䣬����CPUռ�ù���
	}

	return (0);
}
