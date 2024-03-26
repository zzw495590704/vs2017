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
	SR7IF_StartMeasure(DEVICE_ID, 120000);
	//��������������ȡ
	int BatchPoint = SR7IF_ProfilePointSetCount(DEVICE_ID, DataObject); //��ȡ������
	//��ȡ�������
	int m_DataWidth = SR7IF_ProfileDataWidth(DEVICE_ID, DataObject);
	// ѭ����ȡ����������
	int lastBatchPoint_CurNo = 0; //��һ�ν��յ��������������������
	int * HeightData = new int[BatchPoint * m_DataWidth]; //�߶����ݻ���

	do {
		//��ȡ��ǰ���ε��������������
		int BatchPoint_CurNo = SR7IF_ProfilePointCount(DEVICE_ID, DataObject);
		//��ǰ��������������
		int m_curBatchPoint = BatchPoint_CurNo - lastBatchPoint_CurNo;
		//��ȡ��ǰ���θ߶�����
		SR7IF_GetProfileContiuneData(DEVICE_ID, DataObject, &HeightData[lastBatchPoint_CurNo * m_DataWidth], m_curBatchPoint);
		// ѭ������ÿһ��
		for (int i = lastBatchPoint_CurNo; i < m_curBatchPoint; ++i) {
			// ѭ������ÿһ��
			for (int j = 0; j < m_DataWidth; ++j) {
				// ���� HeightData[i * m_DataWidth + j] ����ȡ����
				int data = HeightData[i * m_DataWidth + j];
				// ��������д����������������̨
				std::cout << "Data at (" << i << ", " << j << "): " << data << std::endl;
			}
		}

	} while (lastBatchPoint_CurNo < BatchPoint);

	//�ڴ��ͷ�
	delete[] HeightData;
	//ֹͣ������
	SR7IF_StopMeasure(DEVICE_ID);
	//�ر��豸
	SR7IF_CommClose(DEVICE_ID);
	return 0;
}