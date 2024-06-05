// array_utils.cpp

#include "mathApp.h"
#include <algorithm>
#include <unordered_map>
#include <numeric> // for std::accumulate

// Function to find the maximum value in the array
double findMax(const std::vector<double>& arr) {
	return *std::max_element(arr.begin(), arr.end());
}

// Function to find the minimum value in the array
double findMin(const std::vector<double>& arr) {
	return *std::min_element(arr.begin(), arr.end());
}

// Function to find the average value of the array
double findAverage(const std::vector<double>& arr) {
	double sum = std::accumulate(arr.begin(), arr.end(), 0.0);
	return sum / arr.size();
}

// Function to find the mode (most frequent value) in the array
double findMode(const std::vector<double>& arr) {
	std::unordered_map<double, int> frequencyMap;
	for (double num : arr) {
		frequencyMap[num]++;
	}

	double mode = arr[0];
	int maxCount = 0;
	for (const auto& pair : frequencyMap) {
		if (pair.second > maxCount) {
			maxCount = pair.second;
			mode = pair.first;
		}
	}
	return mode;
}
