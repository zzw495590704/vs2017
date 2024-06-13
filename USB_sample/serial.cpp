#include <windows.h>
#include <iostream>
#include <thread>
#include <sstream>
#include <vector>
#include <string>
#include <mutex>
#include <fstream>
HANDLE hSerial;
bool keepReading = true;
std::vector<int64_t> data;
std::vector<std::vector<int64_t>> csvData;
int64_t timeStamp;
std::mutex dataMutex;
void InitSerialPort(const wchar_t* portName) {
	hSerial = CreateFileW(portName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

	if (hSerial == INVALID_HANDLE_VALUE) {
		if (GetLastError() == ERROR_FILE_NOT_FOUND) {
			std::cerr << "Serial port does not exist.\n";
		}
		std::cerr << "Error opening serial port.\n";
	}

	DCB dcbSerialParams = { 0 };
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (!GetCommState(hSerial, &dcbSerialParams)) {
		std::cerr << "Getting state error.\n";
	}

	dcbSerialParams.BaudRate = CBR_115200;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	if (!SetCommState(hSerial, &dcbSerialParams)) {
		std::cerr << "Error setting serial port state.\n";
	}

	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	if (!SetCommTimeouts(hSerial, &timeouts)) {
		std::cerr << "Error setting timeouts.\n";
	}
}
void WriteCsvDataToFile(const std::string& filename) {
	std::ofstream outFile(filename); // 创建输出文件流

	if (!outFile) {
		std::cerr << "Error opening file for writing: " << filename << std::endl;
		return;
	}

	for (const auto& row : csvData) { // 遍历每一行数据
		for (size_t i = 0; i < row.size(); ++i) {
			outFile << row[i]; // 写入数据
			if (i < row.size() - 1) {
				outFile << ","; // 添加逗号分隔符
			}
		}
		outFile << "\n"; // 写入行结束符
	}

	outFile.close(); // 关闭文件
}

int64_t readTimeStamp() {
	std::lock_guard<std::mutex> lock(dataMutex);  // 使用互斥锁保护对 firstData 的访问
	if (data[0] == NULL)
		timeStamp = 0;
	else
		timeStamp = data[0];
	return timeStamp;  // 返回 firstData 的副本
}

void SerialReadThread() {
	char szBuff[100];
	DWORD dwBytesRead = 0;
	std::string lineBuffer;

	while (keepReading) {
		if (ReadFile(hSerial, szBuff, sizeof(szBuff) - 1, &dwBytesRead, NULL)) {
			szBuff[dwBytesRead] = '\0';  // 添加字符串结束符

			for (DWORD i = 0; i < dwBytesRead; ++i) {
				if (szBuff[i] == '\n') {  // 遇到换行符
					lineBuffer += szBuff[i];
					//std::cout << "Received: " << lineBuffer << std::endl;  // 输出一行数据
					// 处理接收到的行数据

					std::vector<int64_t> tempData;
					std::stringstream ss(lineBuffer);
					std::string item;
					while (std::getline(ss, item, ',')) {
						tempData.push_back(std::stoi(item));
					}
					//std::cout << "Received: " << data[0] << std::endl;
					{
						std::lock_guard<std::mutex> lock(dataMutex);
						data = tempData;  // 更新全局 data 数组
					}
					csvData.push_back(tempData);
					std::cout << "data[0]" << data[0] << " data[1]" << data[1] << " data[2]" << data[2] << std::endl;
					lineBuffer.clear();  // 清空缓冲区
				}

				else {
					lineBuffer += szBuff[i];  // 累积到缓冲区
				}
			}
		}
		else {
			std::cerr << "Error reading from serial port.\n";
		}
	}
}
void serialAppInit() {
	const wchar_t* portName = L"COM3";  // 替换为实际的串口名称
	InitSerialPort(portName);

	std::thread readThread(SerialReadThread);
	readThread.detach(); // 将线程设置为后台运行

	std::cout << "serialAppInit" << std::endl;
}

int serialAppClose() {
	keepReading = false;
	std::cout << "serialAppClose" << std::endl;
	CloseHandle(hSerial); 
	WriteCsvDataToFile("output.csv");
	return 0;
}

void main() {
	serialAppInit();
	std::cin.get();
	serialAppClose();
}
