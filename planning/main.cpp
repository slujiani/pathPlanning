#include <iostream>
#include "demMap.h"

int main() {
    // ����demMap����
    demMap myMap("dem.jpg");

    // ��ȡĳ�����صĸ߶�
    int height = myMap.heightPix(10, 10);

    // ����߶�
    std::cout << "Height at (10, 10): " << height << std::endl;

    return 0;
}
