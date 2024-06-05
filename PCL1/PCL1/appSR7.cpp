#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <mutex>
#include "SR7Link.h"
#include "mathApp.h"
#include "appSR7.h"
#include "serialAPP.h"


#define OFFSETX 24
#define B_POSITION 24
#define	A_POSITION -300

std::mutex cloud_mutex;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr save(new pcl::PointCloud<pcl::PointXYZ>);
bool new_cloud_available = false;
static int s_lenghtX, s_widthY, s_heightZ;
static double s_heightTotal = 0;
static double s_x_offset = 0;
static int s_scanNum = 0;
static bool s_scanStart = false;
static bool s_killScan = false;
std::vector<double> rowX;

void startScanSR7() {
	s_scanStart = true;
	//printf("start scan");
}
void endScanSR7() {
	s_scanStart = false;
}
void StartMeasureSR7() {
	//相机IP地址
	SR7IF_ETHERNET_CONFIG SREthernetConFig;
	SREthernetConFig.abyIpAddress[0] = 192;
	SREthernetConFig.abyIpAddress[1] = 168;
	SREthernetConFig.abyIpAddress[2] = 31;
	SREthernetConFig.abyIpAddress[3] = 21;

	//设备ID
	const int DEVICE_ID = 0;
	SR7IF_Data DataObject = NULL;
	//连接相机 
	SR7IF_EthernetOpen(DEVICE_ID, &SREthernetConFig);

	int EnStartMearsure = 0x01;
	//批处理是使能
	SR7IF_SetSetting(0, 0x01, 0x10, 0, 0x03, 0, &EnStartMearsure, 1);
	//开始批处理
	SR7IF_StartMeasure(DEVICE_ID, 150000);
	//批处理总行数获取
	int BatchPoint = SR7IF_ProfilePointSetCount(DEVICE_ID, DataObject); //获取总行数
	//获取轮廓宽度
	int m_DataWidth = SR7IF_ProfileDataWidth(DEVICE_ID, DataObject);
	//获取x方向距离
	double m_DataXPitch = SR7IF_ProfileData_XPitch(DEVICE_ID, DataObject);
	double m_DataYPitch = 0.02;
	// 输出参数
	std::cout << "总行数:" << BatchPoint << "   宽度:" << m_DataWidth << "   x方向距离:" << m_DataXPitch << "   y方向距离:" << m_DataYPitch << endl;
	// 循环获取批处理数据
	int lastBatchPoint_CurNo = 0; //上一次接收到的批处理数据行数编号
	int * HeightData = new int[BatchPoint * m_DataWidth]; //高度数据缓存
	
	
	do {
		//获取当前批次的批处理行数编号
		int BatchPoint_CurNo = SR7IF_ProfilePointCount(DEVICE_ID, DataObject);
		//当前批次批处理行数
		int m_curBatchPoint = BatchPoint_CurNo - lastBatchPoint_CurNo;
		//获取当前批次高度数据
		SR7IF_GetProfileContiuneData(DEVICE_ID, DataObject, &HeightData[lastBatchPoint_CurNo * m_DataWidth], m_curBatchPoint);
		
		//std::cout << "当前行：" << BatchPoint_CurNo << "  上次行：" << lastBatchPoint_CurNo << "  处理行：" << m_curBatchPoint << "  耗时：" << elapsed_seconds.count() << std::endl;
		// 循环遍历每一行
		for (int i = lastBatchPoint_CurNo; i < BatchPoint_CurNo; ++i) {
			double temp_max_x = 0;
			double temp_min_x = 99999;
			// 循环遍历每一列
			int j_start,j_end;
			if (s_scanNum) {
				j_start = 500;
				j_end = 1600;
			}
			else {
				j_start = 0;
				j_end = 1000;
				
			}
				
			for (int j = 200; j < 1400; ++j) {
				// 访问 HeightData[i * m_DataWidth + j] 来获取数据
				int data = HeightData[i * m_DataWidth + j];
				if (data != -1000000000) {
					pcl::PointXYZ point;
					point.y = (double)j* m_DataXPitch + s_x_offset;
					if (point.y > temp_max_x)
						temp_max_x = point.x;
					if (point.y < temp_min_x)
						temp_min_x = point.y;
					/*if(s_scanNum%2)
						point.y = (double)(BatchPoint-i)* m_DataYPitch;
					else*/
					point.x = (double)i* m_DataYPitch;
					point.z = (double)data / 100000;
					save->points.push_back(point);
					//插入显示点云		
					if (i % 8 == 0 && j % 8 == 0) {
						// Lock the mutex before modifying the shared resource
						std::lock_guard<std::mutex> lock(cloud_mutex);
						cloud->points.push_back(point);
					}
					//体积计算相关
					if (point.z > 0.2) {
						s_heightTotal = s_heightTotal + point.z;
						s_lenghtX += 1;
						s_heightZ += 1;
					}
					// 在这里进行处理，比如输出到控制台
					//std::cout << "Data at (" << i << ", " << j << "): " << data << std::endl;
				}
			}
			double MaxrowX = temp_max_x - temp_min_x;
			std::cout << i << " x行间距:" << MaxrowX << "mm" << std::endl;
			rowX.push_back(MaxrowX);
		}
		lastBatchPoint_CurNo = BatchPoint_CurNo;

		// Set the cloud as new data available
		{
			std::lock_guard<std::mutex> lock(cloud_mutex);
			new_cloud_available = true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for 1 second

	} while (lastBatchPoint_CurNo < BatchPoint);
	std::cout << "total:" << save->size() << " lenghtX:" << s_lenghtX << "  heightZ:" << s_heightZ << std::endl;
	double volume = s_lenghtX * m_DataXPitch * m_DataYPitch * s_heightTotal / s_heightZ;
	std::cout << "=====================" << std::endl;
	std::cout << "选取点云:" << s_heightZ << std::endl;
	std::cout << "体积:" << volume << "mm^3" << std::endl;
	
	double maxVal = findMax(rowX);
	double minVal = findMin(rowX);
	double average = findAverage(rowX);
	double mode = findMode(rowX);

	cout << "Maximum value: " << maxVal << endl;
	cout << "Minimum value: " << minVal << endl;
	cout << "Average value: " << average << endl;
	cout << "Mode value: " << mode << endl;
	std::ofstream outFile("./data/data.txt");
	// 将 vector 写入文件
	for (double num : rowX) {
		outFile << num << "\n";
	}
	// 关闭文件
	outFile.close();

	std::cout << "Vector 已保存到文件 'data.txt' 中。\n";
	//std::cout << "密度:" << 820/volume << "g/cm^3" << std::endl;
	//内存释放
	delete[] HeightData;
	//停止批处理
	SR7IF_StopMeasure(DEVICE_ID);
	//关闭设备
	SR7IF_CommClose(DEVICE_ID);

	///////////////////////////////保存数据/////////////////////////////
	
	s_scanNum++;
	s_x_offset = s_x_offset + OFFSETX;
	std::chrono::time_point<std::chrono::steady_clock> start, end;
	//if (s_scanNum >= 3) {
	//	std::cout << "开始保存PLY文件" << std::endl;
	//	// 开始计时
	//	start = std::chrono::steady_clock::now();
	//	// 保存点云到PLY文件
	//	pcl::io::savePLYFile("./data/point_cloud.ply", *save);
	//	// 结束计时
	//	end = std::chrono::steady_clock::now();
	//	// 计算循环执行时间
	//	std::chrono::duration<double> elapsed_seconds = end - start;
	//	std::cout << "保存point_cloud.ply 耗时：" << elapsed_seconds.count() << std::endl;
	//	// 结束后添加x偏置
	//}
	
}
void generatePointCloud() {
	while (!s_killScan) {
		if (s_scanStart) {
			StartMeasureSR7();
			endScanSR7();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Yield control to other threads
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
	
	// 定义开始时间点和结束时间点
	std::chrono::time_point<std::chrono::steady_clock> start, end;
	// 开始计时
	start = std::chrono::steady_clock::now();
	std::cout << "开始保存PLY文件" << std::endl;
	// 保存点云到PLY文件
	pcl::io::savePLYFile("./data/point_cloud.ply", *save);
	// 结束计时
	end = std::chrono::steady_clock::now();
	// 计算循环执行时间
	std::chrono::duration<double> elapsed_seconds = end - start;
	std::cout << "保存point_cloud.ply 耗时：" << elapsed_seconds.count() << std::endl;
	s_killScan = true;
	serialAppClose();
}


int appSR7() {
	std::cout << "开始创建扫描线程" << std::endl;
	std::thread cloud_generator(generatePointCloud);
	std::thread cloud_viewer(displayPointCloud);

	cloud_generator.join();
	cloud_viewer.join();
	std::cout << "结束扫描线程" << std::endl;
	return 0;
}