#include <iostream>
#include <windows.h>
#include "serialApp.h"

// 串口参数
const char* portName = "COM8";
const DWORD baudRate = CBR_115200; // 波特率9600
bool g_bExitThread = true;
// 串口句柄
HANDLE serialHandle;
HANDLE threadHandle;
//重量监测
double serialWeight;
std::string serialCommand;

DWORD WINAPI SerialReaderThread(LPVOID lpParam) {
	char buffer[255];
	DWORD bytesRead;
	std::string receivedData; // 用于保存当前接收到的完整数据

	while (g_bExitThread) {
		if (!ReadFile(serialHandle, buffer, sizeof(buffer), &bytesRead, NULL)) {
			std::cerr << "Error reading from serial port." << std::endl;
			CloseHandle(serialHandle);
			return 1;
		}

		if (bytesRead > 0 && bytesRead < 255) {
			// 将读取的数据添加到已接收数据中
			receivedData.append(buffer, bytesRead);
			std::cout << "数据传输 " << receivedData << std::endl;
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
					serialCommand = line.substr(0, commaPos);
					std::string valueStr = line.substr(commaPos + 1);
					// 转换为float类型
					serialWeight = std::stof(valueStr);
					// 打印结果
					std::cout << "Command: " << serialCommand << ", Value: " << serialWeight << std::endl;
				}
				// 删除已处理的数据
				receivedData.erase(0, newlinePos + 1);
			}
		}
	}

	return 0;
}

double serialAppGetWeight() {
	return serialWeight;
}

std::string serialAppGetCommand() {
	return serialCommand;
}

void serialAppClose() {
	std::cout << "Thread start kill." << std::endl;
	// 关闭线程和串口
	g_bExitThread = false;
	WaitForSingleObject(threadHandle, INFINITE);
	CloseHandle(threadHandle);
	WaitForSingleObject(serialHandle, INFINITE);
	CloseHandle(serialHandle);
	std::cout << "Thread stopped." << std::endl;
}

int serialApp() {
	// 打开串口
	serialHandle = CreateFile(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (serialHandle == INVALID_HANDLE_VALUE) {
		std::cerr << "Failed to open serial port." << std::endl;
		return 1;
	}
	// 设置串口超时时间
	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 50; // 读间隔超时，单位为毫秒，例如50ms
	timeouts.ReadTotalTimeoutConstant = 50; // 总超时时间，单位为毫秒，例如50ms
	timeouts.ReadTotalTimeoutMultiplier = 10; // 每个字符的超时时间，单位为毫秒，例如10ms
	timeouts.WriteTotalTimeoutConstant = 50; // 写入总超时时间，单位为毫秒，例如50ms
	timeouts.WriteTotalTimeoutMultiplier = 10; // 每个字符的写入超时时间，单位为毫秒，例如10ms
	if (!SetCommTimeouts(serialHandle, &timeouts)) {
		std::cerr << "Failed to set serial port timeouts." << std::endl;
		CloseHandle(serialHandle);
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
	threadHandle = CreateThread(NULL, 0, SerialReaderThread, NULL, 0, NULL);
	if (threadHandle == NULL) {
		std::cerr << "Failed to create serial reader thread." << std::endl;
		CloseHandle(serialHandle);
		return 1;
	}
	return 0;
}
