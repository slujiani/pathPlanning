#pragma once
#ifndef DEMMAP_H
#define DEMMAP_H

#include <string>
#include <opencv2/opencv.hpp>

class demMap {
public:
    // ���캯��������ͼ��·����Ϊ����
    demMap(const std::string& imagePath);

    // ����ָ�����ص�ĸ߶�
    int heightPix(int x, int y) const;

private:
    std::string imagePath_;
    cv::Mat elevationMap_;
};

#endif // DEMMAP_H

