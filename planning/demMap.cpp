#include "demMap.h"
#include<iostream>
#include<string>
#include <opencv2/opencv.hpp>

demMap::demMap(const std::string& imagePath) : imagePath_(imagePath) 
{
    this->elevationMap_ = cv::imread(imagePath, cv::IMREAD_GRAYSCALE); // Ĭ�϶���Ҷ�ͼ
    if (this->elevationMap_.empty()) 
    {
        std::cerr << "Error: Could not open or find the image!" << std::endl;
        exit(EXIT_FAILURE);
    }
}

int demMap::heightPix(int x, int y) const 
{
    // �������ظ߶�
    return this->elevationMap_.at<uchar>(y, x);
}

