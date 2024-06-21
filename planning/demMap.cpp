#include "demMap.h"
#include<iostream>
#include<string>
#include <opencv2/opencv.hpp>

demMap::demMap(const std::string& imagePath) : imagePath_(imagePath) 
{
    this->elevationMap_ = cv::imread(imagePath, cv::IMREAD_GRAYSCALE); // 默认读入灰度图
    if (this->elevationMap_.empty()) 
    {
        std::cerr << "Error: Could not open or find the image!" << std::endl;
        exit(EXIT_FAILURE);
    }
}

int demMap::heightPix(int x, int y) const 
{
    // 返回像素高度
    return this->elevationMap_.at<uchar>(y, x);
}

