#include <iostream>
#include <windows.h>
#include <string>
#include "serialApp.h"

// 串口参数
const char* portName = "COM8";
const DWORD baudRate = CBR_115200; // 波特率9600

// 串口句柄
HANDLE serialHandle;

DWORD WINAPI SerialReaderThread(LPVOID lpParam) {
	char buffer[255];
	DWORD bytesRead;
	std::string receivedData; // 用于保存当前接收到的完整数据

	while (true) {
		if (!ReadFile(serialHandle, buffer, sizeof(buffer), &bytesRead, NULL)) {
			std::cerr << "Error reading from serial port." << std::endl;
			CloseHandle(serialHandle);
			return 1;
		}

		if (bytesRead > 0) {
			// 将读取的数据添加到已接收数据中
			receivedData.append(buffer, bytesRead);

			// 检查是否接收到完整一行数据（包含换行符）
			size_t newlinePos;
			while ((newlinePos = receivedData.find('\n')) != std::string::npos) {
				// 提取完整一行数据
				std::string line = receivedData.substr(0, newlinePos);
				// 打印读取到的数据
				std::cout << "Received: " << line << std::endl;
				// 划分数据
				size_t commaPos = line.find(',');
				if (commaPos != std::string::npos) {
					std::string command = line.substr(0, commaPos);
					std::string valueStr = line.substr(commaPos + 1);
					// 转换为float类型
					float value = std::stof(valueStr);
					// 打印结果
					std::cout << "Command: " << command << ", Value: " << value << std::endl;
				}
				// 删除已处理的数据
				receivedData.erase(0, newlinePos + 1);
			}
		}
	}

	return 0;
}

int serialApp() {
	// 打开串口
	serialHandle = CreateFile(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (serialHandle == INVALID_HANDLE_VALUE) {
		std::cerr << "Failed to open serial port." << std::endl;
		return 1;
	}

	// 配置串口
	DCB serialParams = { 0 };
	serialParams.DCBlength = sizeof(serialParams);
	if (!GetCommState(serialHandle, &serialParams)) {
		std::cerr << "Failed to get serial port parameters." << std::endl;
		CloseHandle(serialHandle);
		return 1;
	}
	serialParams.BaudRate = baudRate;
	serialParams.ByteSize = 8;
	serialParams.StopBits = ONESTOPBIT;
	serialParams.Parity = NOPARITY;
	if (!SetCommState(serialHandle, &serialParams)) {
		std::cerr << "Failed to set serial port parameters." << std::endl;
		CloseHandle(serialHandle);
		return 1;
	}

	// 创建线程
	HANDLE threadHandle = CreateThread(NULL, 0, SerialReaderThread, NULL, 0, NULL);
	if (threadHandle == NULL) {
		std::cerr << "Failed to create serial reader thread." << std::endl;
		CloseHandle(serialHandle);
		return 1;
	}

	// 主线程继续执行其他任务
	std::string userInput;
	std::cout << "Enter 'quit' to exit." << std::endl;
	while (userInput != "quit") {
		std::cin >> userInput;
	}

	// 关闭线程和串口
	CloseHandle(threadHandle);
	CloseHandle(serialHandle);
	return 0;
}
