#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <mutex>
#include "SR7Link.h"
std::mutex cloud_mutex;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
bool new_cloud_available = false;

void generatePointCloud() {
	//���IP��ַ
	SR7IF_ETHERNET_CONFIG SREthernetConFig;
	SREthernetConFig.abyIpAddress[0] = 192;
	SREthernetConFig.abyIpAddress[1] = 168;
	SREthernetConFig.abyIpAddress[2] = 31;
	SREthernetConFig.abyIpAddress[3] = 21;

	//�豸ID
	const int DEVICE_ID = 0;
	SR7IF_Data DataObject = NULL;
	//������� 
	SR7IF_EthernetOpen(DEVICE_ID, &SREthernetConFig);
	//��ʼ������
	SR7IF_StartMeasure(DEVICE_ID, 150000);
	//��������������ȡ
	int BatchPoint = SR7IF_ProfilePointSetCount(DEVICE_ID, DataObject); //��ȡ������
	//��ȡ��������
	int m_DataWidth = SR7IF_ProfileDataWidth(DEVICE_ID, DataObject);
	//��ȡx�������
	double m_DataXPitch = SR7IF_ProfileData_XPitch(DEVICE_ID, DataObject);
	// �������
	std::cout << "������:" << BatchPoint << "   ����:" << m_DataWidth << "   x�������:" << m_DataXPitch << endl;
	// ѭ����ȡ����������
	int lastBatchPoint_CurNo = 0; //��һ�ν��յ��������������������
	int * HeightData = new int[BatchPoint * m_DataWidth]; //�߶����ݻ���
	 // ���忪ʼʱ���ͽ���ʱ���
	std::chrono::time_point<std::chrono::steady_clock> start, end;
	// ��ʼ��ʱ
	start = std::chrono::steady_clock::now();
	do {
		//��ȡ��ǰ���ε��������������
		int BatchPoint_CurNo = SR7IF_ProfilePointCount(DEVICE_ID, DataObject);
		//��ǰ��������������
		int m_curBatchPoint = BatchPoint_CurNo - lastBatchPoint_CurNo;
		//��ȡ��ǰ���θ߶�����
		SR7IF_GetProfileContiuneData(DEVICE_ID, DataObject, &HeightData[lastBatchPoint_CurNo * m_DataWidth], m_curBatchPoint);
		// ������ʱ
		end = std::chrono::steady_clock::now();
		// ����ѭ��ִ��ʱ��
		std::chrono::duration<double> elapsed_seconds = end - start;
		std::cout << "��ǰ�У�" << BatchPoint_CurNo << "  �ϴ��У�" << lastBatchPoint_CurNo << "  �����У�" << m_curBatchPoint << "  ��ʱ��" << elapsed_seconds.count() << std::endl;
		start = end;
		// ѭ������ÿһ��
		for (int i = lastBatchPoint_CurNo; i < BatchPoint_CurNo; ++i) {
			// ѭ������ÿһ��
			for (int j = 0; j < m_DataWidth; ++j) {
				// ���� HeightData[i * m_DataWidth + j] ����ȡ����
				int data = HeightData[i * m_DataWidth + j];
				if (data != -1000000000) {
					pcl::PointXYZ point;
					point.x = (double)i*0.01;
					point.y = (double)j*0.02;
					point.z = (double)data/100000;

					// Lock the mutex before modifying the shared resource
					std::lock_guard<std::mutex> lock(cloud_mutex);
					cloud->points.push_back(point);
					// ��������д������������������̨
					//std::cout << "Data at (" << i << ", " << j << "): " << data << std::endl;
				}
				
			}
		}
		lastBatchPoint_CurNo = BatchPoint_CurNo;

		// Set the cloud as new data available
		{
			std::lock_guard<std::mutex> lock(cloud_mutex);
			new_cloud_available = true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for 1 second

	} while (lastBatchPoint_CurNo < BatchPoint);

	//�ڴ��ͷ�
	delete[] HeightData;
	//ֹͣ������
	SR7IF_StopMeasure(DEVICE_ID);
	//�ر��豸
	SR7IF_CommClose(DEVICE_ID);

	///////////////////////////////��������/////////////////////////////
	std::cout << "��ʼ����PLY�ļ�" << std::endl;
	// ��ʼ��ʱ
	start = std::chrono::steady_clock::now();
	// ������ʱ
	end = std::chrono::steady_clock::now();
	// ����ѭ��ִ��ʱ��
	std::chrono::duration<double> elapsed_seconds = end - start;
	// ������Ƶ�PLY�ļ�
	pcl::io::savePLYFile("./data/point_cloud.ply", *cloud);
	std::cout << "����point_cloud.ply ��ʱ��"<< elapsed_seconds.count() << std::endl;
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