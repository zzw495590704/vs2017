#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <thread> // 包含用于线程的头文件
int main_()
{
	// 创建点云对象
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 读取PLY文件
	if (pcl::io::loadPLYFile<pcl::PointXYZ>("./data/loop-path-fix.ply", *cloud) == -1) //* 读入PLY格式的点云文件，如果失败，返回-1
	{
		PCL_ERROR("无法加载PLY文件.\n");
		return (-1);
	}
	std::cout << "加载的点云包含 " << cloud->size() << " 个点。" << std::endl;

	// 创建体素网格下采样滤波器
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);

	// 设置体素网格的尺寸，这里只在 Y 方向进行抽样
	sor.setLeafSize(0.1f, 0.1f, 0.1f); // 设置Y方向的体素尺寸为0.1米，其他方向为0.01米

	// 执行滤波，将点云进行抽样
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
	sor.filter(*filtered);
	std::cout << "点云中的点的数量为：" << filtered->size() << std::endl;
	// 可视化显示抽样后的点云
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	viewer->setBackgroundColor(0, 0, 0); // 设置背景颜色为黑色
	viewer->addPointCloud<pcl::PointXYZ>(filtered, "cloud"); // 添加点云到可视化窗口
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud"); // 设置点云显示大小

	// 显示可视化窗口，直到窗口关闭
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100); // 使可视化窗口保持活跃状态
		std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 休眠一段时间，避免CPU占用过高
	}

	return (0);
}
