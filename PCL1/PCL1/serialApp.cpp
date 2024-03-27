#include <iostream>
#include <windows.h>
#include "serialApp.h"

// ���ڲ���
const char* portName = "COM8";
const DWORD baudRate = CBR_115200; // ������9600
bool g_bExitThread = true;
// ���ھ��
HANDLE serialHandle;
HANDLE threadHandle;
//�������
double serialWeight;
std::string serialCommand;

DWORD WINAPI SerialReaderThread(LPVOID lpParam) {
	char buffer[255];
	DWORD bytesRead;
	std::string receivedData; // ���ڱ��浱ǰ���յ�����������

	while (g_bExitThread) {
		if (!ReadFile(serialHandle, buffer, sizeof(buffer), &bytesRead, NULL)) {
			std::cerr << "Error reading from serial port." << std::endl;
			CloseHandle(serialHandle);
			return 1;
		}

		if (bytesRead > 0 && bytesRead < 255) {
			// ����ȡ��������ӵ��ѽ���������
			receivedData.append(buffer, bytesRead);
			std::cout << "���ݴ��� " << receivedData << std::endl;
			// ����Ƿ���յ�����һ�����ݣ��������з���
			size_t newlinePos;
			while ((newlinePos = receivedData.find('\n')) != std::string::npos) {
				// ��ȡ����һ������
				std::string line = receivedData.substr(0, newlinePos);
				// ��ӡ��ȡ��������
				std::cout << "Received: " << line << std::endl;
				// ��������
				size_t commaPos = line.find(',');
				if (commaPos != std::string::npos) {
					serialCommand = line.substr(0, commaPos);
					std::string valueStr = line.substr(commaPos + 1);
					// ת��Ϊfloat����
					serialWeight = std::stof(valueStr);
					// ��ӡ���
					std::cout << "Command: " << serialCommand << ", Value: " << serialWeight << std::endl;
				}
				// ɾ���Ѵ��������
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
	// �ر��̺߳ʹ���
	g_bExitThread = false;
	WaitForSingleObject(threadHandle, INFINITE);
	CloseHandle(threadHandle);
	WaitForSingleObject(serialHandle, INFINITE);
	CloseHandle(serialHandle);
	std::cout << "Thread stopped." << std::endl;
}

int serialApp() {
	// �򿪴���
	serialHandle = CreateFile(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (serialHandle == INVALID_HANDLE_VALUE) {
		std::cerr << "Failed to open serial port." << std::endl;
		return 1;
	}
	// ���ô��ڳ�ʱʱ��
	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 50; // �������ʱ����λΪ���룬����50ms
	timeouts.ReadTotalTimeoutConstant = 50; // �ܳ�ʱʱ�䣬��λΪ���룬����50ms
	timeouts.ReadTotalTimeoutMultiplier = 10; // ÿ���ַ��ĳ�ʱʱ�䣬��λΪ���룬����10ms
	timeouts.WriteTotalTimeoutConstant = 50; // д���ܳ�ʱʱ�䣬��λΪ���룬����50ms
	timeouts.WriteTotalTimeoutMultiplier = 10; // ÿ���ַ���д�볬ʱʱ�䣬��λΪ���룬����10ms
	if (!SetCommTimeouts(serialHandle, &timeouts)) {
		std::cerr << "Failed to set serial port timeouts." << std::endl;
		CloseHandle(serialHandle);
		return 1;
	}

	// ���ô���
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

	// �����߳�
	threadHandle = CreateThread(NULL, 0, SerialReaderThread, NULL, 0, NULL);
	if (threadHandle == NULL) {
		std::cerr << "Failed to create serial reader thread." << std::endl;
		CloseHandle(serialHandle);
		return 1;
	}
	return 0;
}
