#include <iostream>
#include <chrono>
#include <thread>
#include "serialAPP.h"
#include "appSR7.h"

std::string scanCommand = "SCAN";

int main() {
	serialApp();
	/*while (true)
	{*/
		double weight = serialAppGetWeight();
		std::string command = serialAppGetCommand();

		std::cout << "Command:" << command << "  Weight:" << weight << std::endl;
		/*if (command == scanCommand) {
			std::cout << "����ѭ��" << std::endl;
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}*/
	appSR7();
	// �ȴ��û�������ֹͣ�߳�
	std::cout << "Press enter to stop the thread." << std::endl;
	std::cin.get();
	serialAppClose();
	return 0;
}