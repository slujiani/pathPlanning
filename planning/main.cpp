#include <iostream>
#include "demMap.h"

int main() {
    // 创建demMap对象
    demMap myMap("dem.jpg");

    // 获取某个像素的高度
    int height = myMap.heightPix(10, 10);

    // 输出高度
    std::cout << "Height at (10, 10): " << height << std::endl;

    return 0;
}
