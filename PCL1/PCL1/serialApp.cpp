#include <iostream>
#include <windows.h>
#include <string>
#include "serialApp.h"

// ���ڲ���
const char* portName = "COM8";
const DWORD baudRate = CBR_115200; // ������9600

// ���ھ��
HANDLE serialHandle;

DWORD WINAPI SerialReaderThread(LPVOID lpParam) {
	char buffer[255];
	DWORD bytesRead;
	std::string receivedData; // ���ڱ��浱ǰ���յ�����������

	while (true) {
		if (!ReadFile(serialHandle, buffer, sizeof(buffer), &bytesRead, NULL)) {
			std::cerr << "Error reading from serial port." << std::endl;
			CloseHandle(serialHandle);
			return 1;
		}

		if (bytesRead > 0) {
			// ����ȡ��������ӵ��ѽ���������
			receivedData.append(buffer, bytesRead);

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
					std::string command = line.substr(0, commaPos);
					std::string valueStr = line.substr(commaPos + 1);
					// ת��Ϊfloat����
					float value = std::stof(valueStr);
					// ��ӡ���
					std::cout << "Command: " << command << ", Value: " << value << std::endl;
				}
				// ɾ���Ѵ��������
				receivedData.erase(0, newlinePos + 1);
			}
		}
	}

	return 0;
}

int serialApp() {
	// �򿪴���
	serialHandle = CreateFile(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (serialHandle == INVALID_HANDLE_VALUE) {
		std::cerr << "Failed to open serial port." << std::endl;
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
	HANDLE threadHandle = CreateThread(NULL, 0, SerialReaderThread, NULL, 0, NULL);
	if (threadHandle == NULL) {
		std::cerr << "Failed to create serial reader thread." << std::endl;
		CloseHandle(serialHandle);
		return 1;
	}

	// ���̼߳���ִ����������
	std::string userInput;
	std::cout << "Enter 'quit' to exit." << std::endl;
	while (userInput != "quit") {
		std::cin >> userInput;
	}

	// �ر��̺߳ʹ���
	CloseHandle(threadHandle);
	CloseHandle(serialHandle);
	return 0;
}
