#include <iostream>
#include <chrono>
#include <thread>
#include "serialAPP.h"
#include "appSR7.h"
#include <cstdlib>
std::string scanCommand = "SCAN";

int main() {
	std::thread serialThread(serialApp);
	std::thread SR7Thread(appSR7);
	serialThread.join();
	SR7Thread.join();
	//serialApp();
	//appSR7();
	// 等待用户输入来停止线程	
	std::cout << "run exe" << std::endl;
	//system("C:/Users/codefab/.conda/envs/zzw/python test.py");
	std::cout << "Press enter to stop the thread." << std::endl;
	std::cin.get();
	//serialAppClose();
	
	return 0;
}