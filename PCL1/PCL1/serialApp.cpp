
#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include "serialAPP.h"
#include "appSR7.h"
using namespace boost::asio;
bool g_bExitThread = true;

//重量监测
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
	// 关闭线程和串口
	g_bExitThread = false;
	std::cout << "Thread stopped." << std::endl;
	
}

// 串口数据接收函数
void serialReceive(serial_port& serial) {
	// 读取串口数据
	while (g_bExitThread) {
		std::string line;
		// 逐字符读取直到遇到换行符
		char c;
		while (true) {
			read(serial, buffer(&c, 1));
			if (c == '\n') {
				break;
			}
			line += c;
		}
		// 打印接收到的字符串
		//std::cout << "Received: " << line << std::endl;
		size_t commaPos = line.find(':');
		if (commaPos != std::string::npos) {
			serialCommand = line.substr(0, commaPos);
			std::string valueStr = line.substr(commaPos + 1);
			// 转换为float类型
			serialWeight = std::stof(valueStr);
			// 打印结果
			//std::cout << "Command: " << serialCommand << ", Value: " << serialWeight << std::endl;
			if (serialCommand == "SCAN") {
				startScanSR7();
			}
		}
	}
}

int serialApp() {
	// 设置串口端口号和波特率
	io_service io;
	serial_port serial(io, "COM7"); // 更改串口路径以匹配你的串口
	serial.set_option(serial_port_base::baud_rate(115200)); // 更改波特率以匹配你的设置

	// 创建一个新线程来接收串口数据
	std::thread receiveThread(serialReceive, std::ref(serial));

	// 主线程继续执行其他操作
	// 这里可以放置你的其他代码，或者直接进入休眠状态等待程序结束

	// 等待接收线程结束
	receiveThread.join();
	return 0;
}
