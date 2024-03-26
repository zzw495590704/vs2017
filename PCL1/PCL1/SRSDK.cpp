#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <vtkAutoInit.h>
#include "SR7Link.h"
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingOpenGL);
using namespace std;

int main()
{
	cout << "test" << endl;
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
	//开始批处理
	SR7IF_StartMeasure(DEVICE_ID, 120000);
	//批处理总行数获取
	int BatchPoint = SR7IF_ProfilePointSetCount(DEVICE_ID, DataObject); //获取总行数
	//获取轮廓宽度
	int m_DataWidth = SR7IF_ProfileDataWidth(DEVICE_ID, DataObject);
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
		// 循环遍历每一行
		for (int i = lastBatchPoint_CurNo; i < m_curBatchPoint; ++i) {
			// 循环遍历每一列
			for (int j = 0; j < m_DataWidth; ++j) {
				// 访问 HeightData[i * m_DataWidth + j] 来获取数据
				int data = HeightData[i * m_DataWidth + j];
				// 在这里进行处理，比如输出到控制台
				std::cout << "Data at (" << i << ", " << j << "): " << data << std::endl;
			}
		}

	} while (lastBatchPoint_CurNo < BatchPoint);

	//内存释放
	delete[] HeightData;
	//停止批处理
	SR7IF_StopMeasure(DEVICE_ID);
	//关闭设备
	SR7IF_CommClose(DEVICE_ID);
	return 0;
}