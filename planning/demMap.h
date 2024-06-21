#pragma once
#ifndef DEMMAP_H
#define DEMMAP_H

#include <string>
#include <opencv2/opencv.hpp>

class demMap {
public:
    // 构造函数，接受图像路径作为参数
    demMap(const std::string& imagePath);

    // 返回指定像素点的高度
    int heightPix(int x, int y) const;

private:
    std::string imagePath_;
    cv::Mat elevationMap_;
};

#endif // DEMMAP_H

