
#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include "serialAPP.h"
#include "appSR7.h"
using namespace boost::asio;
bool g_bExitThread = true;

//�������
double serialWeight;
std::string serialCommand;

double serialAppGetWeight() {
	printf("serialAppGetWeight:%lf\n", serialAppGetWeight);
	return serialWeight;
}

std::string serialAppGetCommand() {
	return serialCommand;
}

void serialAppClose() {
	std::cout << "Thread start kill." << std::endl;
	// �ر��̺߳ʹ���
	g_bExitThread = false;
	std::cout << "Thread stopped." << std::endl;
	
}

// �������ݽ��պ���
void serialReceive(serial_port& serial) {
	// ��ȡ��������
	while (g_bExitThread) {
		std::string line;
		// ���ַ���ȡֱ���������з�
		char c;
		while (true) {
			read(serial, buffer(&c, 1));
			if (c == '\n') {
				break;
			}
			line += c;
		}
		// ��ӡ���յ����ַ���
		//std::cout << "Received: " << line << std::endl;
		size_t commaPos = line.find(':');
		if (commaPos != std::string::npos) {
			serialCommand = line.substr(0, commaPos);
			std::string valueStr = line.substr(commaPos + 1);
			// ת��Ϊfloat����
			serialWeight = std::stof(valueStr);
			// ��ӡ���
			//std::cout << "Command: " << serialCommand << ", Value: " << serialWeight << std::endl;
			if (serialCommand == "SCAN") {
				startScanSR7();
			}
		}
	}
}

int serialApp() {
	// ���ô��ڶ˿ںźͲ�����
	io_service io;
	serial_port serial(io, "COM7"); // ���Ĵ���·����ƥ����Ĵ���
	serial.set_option(serial_port_base::baud_rate(115200)); // ���Ĳ�������ƥ���������

	// ����һ�����߳������մ�������
	std::thread receiveThread(serialReceive, std::ref(serial));

	// ���̼߳���ִ����������
	// ������Է�������������룬����ֱ�ӽ�������״̬�ȴ��������

	// �ȴ������߳̽���
	receiveThread.join();
	return 0;
}
