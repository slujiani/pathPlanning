#define _USE_MATH_DEFINES
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <matplotlibcpp.h>
#include <fstream>
#include<Windows.h>
#include "include/base.h"
#include "include/footDEMPlanning.h"
//using namespace cv;
//using namespace std;
//namespace plt = matplotlibcpp;



float heuristic(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

bool isCollisionFree(const cv::Mat& elevationMap, int x, int y,cv::Point direction) {
    int curHeight = elevationMap.at<uchar>(y, x);
    int halfWidth = (robotWidth / perPixelWidth) / 2;
    int halfLength = (robot_foot_length / perPixelWidth) / 2;

    // Calculate the bounding box based on the direction of movement
    int dirX = direction.x;
    int dirY = direction.y;

    for (int i = -halfWidth; i <= halfWidth; ++i) {
        int newX, newY;

        if (dirX == 0) { // Moving vertically
            newX = x + i;
            newY = y + dirY * halfLength;
        } 
        else { // Moving horizontally or diagonally
            newX = x + dirX * halfLength;
            newY = y + i;
        }

        if (newX < 0 || newX >= elevationMap.cols || newY < 0 || newY >= elevationMap.rows) {
            return false; // Out of bounds
        }

        uchar newHeight = elevationMap.at<uchar>(newY, newX);
        if (newHeight == 255) {
            return false; // 255 indicates no data
        }
        //float heightDiff = abs(curHeight - newHeight) * perPixelHeight;
        //if (heightDiff > maxStepHeight) {
        //    return false;
        //}
        //float hDiff = (curHeight - newHeight) * perPixelHeight;
        //if (hDiff > 0.1) {
        //    return false; // Obstacle detected
        //}
        
    }
    return true;
}

std::vector<pixDir> aStar(const cv::Mat& elevationMap, cv::Point start, cv::Point goal, std::unordered_set<int>& visitedPoints) {
    auto start_time = std::chrono::high_resolution_clock::now();  // Start timing
    std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> openList;
    std::unordered_map<int, pixDir> cameFrom;
    std::unordered_map<int, float> costSoFar;

    auto hashPoint = [&elevationMap](int x, int y) { return y * elevationMap.cols + x; };

    openList.push({ start.x, start.y, 0, heuristic(start.x, start.y, goal.x, goal.y) });
    cameFrom[hashPoint(start.x, start.y)] = { {start.x , start.y }, {0, 0} };
    costSoFar[hashPoint(start.x, start.y)] = 0;

    std::vector<cv::Point> directions = { {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1} };

    while (!openList.empty()) {
        PathNode current = openList.top();
        openList.pop();

        if (current.x == goal.x && current.y == goal.y) {
            auto end_time = std::chrono::high_resolution_clock::now();  // End timing
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            std::cout << "Path found in " << duration << " ms" << std::endl;

            std::vector<pixDir> path;
            int currentHash = hashPoint(goal.x, goal.y);
            pixDir currentPosDir = cameFrom[currentHash];

            // Add the first node
            path.push_back(currentPosDir);
            while (currentPosDir.pos != start) {
                path.push_back(currentPosDir);
                currentHash = hashPoint(currentPosDir.pos.x, currentPosDir.pos.y);
                currentPosDir = cameFrom[currentHash];
            }

            // Add the starting point
            path.push_back(cameFrom[hashPoint(start.x, start.y)]);
            reverse(path.begin(), path.end());
            return path;
        }

        for (const cv::Point& dir : directions) {
            int newX = current.x + dir.x;
            int newY = current.y + dir.y;
            visitedPoints.insert(hashPoint(newX, newY));
            if (newX >= 0 && newX < elevationMap.cols && newY >= 0 && newY < elevationMap.rows) {
                //float heightDiff = abs(elevationMap.at<uchar>(newY, newX) - elevationMap.at<uchar>(current.y, current.x)) * perPixelHeight;
                //if (!isCollisionFree(elevationMap, newX, newY, dir)) {
                //    continue; // 高度差超过最大抬脚高度，跳过
                //}
                if (elevationMap.at<uchar>(newY, newX) == 255) {
                    continue;
                }
                float newCost = costSoFar[hashPoint(current.x, current.y)] +  1; // Reduce the impact of height difference


                int hashNewPoint = hashPoint(newX, newY);
                if (costSoFar.find(hashNewPoint) == costSoFar.end() || newCost < costSoFar[hashNewPoint]) {
                    costSoFar[hashNewPoint] = newCost;
                    float priority = newCost + heuristic(newX, newY, goal.x, goal.y);
                    openList.push({ newX, newY, newCost, priority });

                    cameFrom[hashNewPoint] = { {current.x, current.y}, dir }; // Corrected to store the correct node
                }
            }
        }
    }

    return {};  // Return an empty path if no path found
}
uchar curAreaHeight(const cv::Mat& elevationMap, int width, int length, int x, int y)
{
    uchar all = 0;
    for (int i = -width/2; i <= width/2; i++)
    {
        for (int j = -length/2; j <= length/2; j++)
        {
            all = all + elevationMap.at<uchar>(y+j, x+i);
        }
    }
    all /= (width * length);
    return all;
}
//平滑路径
std::vector<pixDir> smoothPath(const std::vector<pixDir>& path, int numInterpolatedPoints = 10) {
    std::vector<pixDir> newPath;

    for (size_t i = 0; i < path.size() - 1; ++i) {
        cv::Point p0 = (i == 0) ? path[i].pos : path[i - 1].pos;
        cv::Point p1 = path[i].pos;
        cv::Point p2 = path[i + 1].pos;
        cv::Point p3 = (i == path.size() - 2) ? path[i + 1].pos : path[i + 2].pos;

        for (int j = 0; j <= numInterpolatedPoints; ++j) {
            double t = j / static_cast<double>(numInterpolatedPoints);
            double t2 = t * t;
            double t3 = t2 * t;

            double f0 = -0.5 * t3 + t2 - 0.5 * t;
            double f1 = 1.5 * t3 - 2.5 * t2 + 1.0;
            double f2 = -1.5 * t3 + 2.0 * t2 + 0.5 * t;
            double f3 = 0.5 * t3 - 0.5 * t2;

            double x = f0 * p0.x + f1 * p1.x + f2 * p2.x + f3 * p3.x;
            double y = f0 * p0.y + f1 * p1.y + f2 * p2.y + f3 * p3.y;

            newPath.push_back({ cv::Point(static_cast<int>(x), static_cast<int>(y)), cv::Point(p2.x - p1.x, p2.y - p1.y) });
        }
    }

    return newPath;
}

//分割路径
std::vector<std::vector<pixDir>> segmentPath(const std::vector<pixDir>& path, const cv::Mat& elevationMap, const cv::Mat& edgeMap)
{
    std::vector<std::vector<pixDir>> segmentedPaths;
    std::vector<pixDir> currentSegment;
    currentSegment.push_back(path[0]);
    bool isEdge = false;
    for (size_t i = 1; i < path.size(); ++i) {
        uchar currentEdgeValue = edgeMap.at<uchar>(path[i].pos.y, path[i].pos.x);

        // 检查路径点是否经过边缘
        if (isEdge==false && currentEdgeValue > 0 && currentSegment.size()>0) {
            // 高度差超过阈值，分成一个新段
            
            segmentedPaths.push_back(currentSegment);
            currentSegment.clear();
            isEdge = true;
        }
        else if (isEdge == true && currentEdgeValue <= 0)
        {
            isEdge = false;
        }
        else
        {
            currentSegment.push_back(path[i]);
        }
        
    }

    if (!currentSegment.empty()) {
        segmentedPaths.push_back(smoothPath(currentSegment));
    }
    return segmentedPaths;
}

int sgn(double k) {
    if (k > 0) {
        return 1;
    }
    else {
        return -1;
    }
}

int sgnTag(double k) {
    if (k > 1) {
        return 2;
    }
    else if (k > 0) {
        return 1;
    }
    else if (k == 0) {
        return 0;
    }
    else {
        return -1;
    }
}

int sgnOfk(xy v0, xy v1) {
    return sgn(-(v0.x * v1.y - v0.y * v1.x));
}

xy calNextPosition(double x0, double y0, double a, double r) {
    double x, y;
    double abs_a = abs(a);
    if (a <= 0 && a > -M_PI / 2) {
        x = x0 + r * cos(abs_a);
        y = y0 + r * sin(abs_a);
    }
    else if (a <= -M_PI / 2 && a >= -M_PI) {
        x = x0 - r * cos(M_PI - abs_a);
        y = y0 + r * sin(M_PI - abs_a);
    }
    else if (a >= 0 && a <= M_PI / 2) {
        x = x0 + r * cos(abs_a);
        y = y0 - r * sin(abs_a);
    }
    else {
        x = x0 - r * cos(M_PI - abs_a);
        y = y0 - r * sin(M_PI - abs_a);
    }
    xy p1 = { x, y };
    return p1;
}

double cal_da(xy v0, xy v1) {
    double a = sgnOfk(v0, v1) * angleBetweenVector(v0, v1);
    double abs_a = abs(a);
    double da;
    if (deflectionAngleRate * abs_a > maxa) {
        da = maxa;
    }
    else {
        da = deflectionAngleRate * abs_a;
    }
    return da;
}

xy newRotationAngle(double da, xy v0, xy& v1) {
    xy tmpA = { cos(da), -sin(da) };
    xy tmpB = { sin(da), cos(da) };
    xy vetp = { dotOfVec(tmpA, v1), dotOfVec(tmpB, v1) };
    tmpA = { cos(-da), -sin(-da) };
    tmpB = { sin(-da), cos(-da) };
    xy vetn = { dotOfVec(tmpA, v1), dotOfVec(tmpB, v1) };
    v1 = vetp;
    double ap = angleBetweenVector(v0, v1);
    v1 = vetn;
    double an = angleBetweenVector(v0, v1);
    if (ap <= an) {
        return vetp;
    }
    else {
        return vetn;
    }
}
//z的坐标用图周围的平均值计算
void dataToFile(std::vector<posDirect2D> Path, std::string fname, const cv::Mat& demMap) {
    std::ofstream outFile(fname, std::ios::out);
    int peSize = Path.size();

    for (int i = 0; i < peSize; i++) {
        int x = static_cast<int>(Path[i].pos.x / perPixelWidth);
        int y = static_cast<int>(Path[i].pos.y / perPixelWidth);

        // Check bounds
        if (x >= 0 && x < demMap.cols && y >= 0 && y < demMap.rows) {
            // Get the height (z) value from the DEM map
            double z = demMap.at<uchar>(y, x) * perPixelHeight;

            outFile << std::to_string(Path[i].pos.x) << ','
                << std::to_string(Path[i].pos.y) << ','
                << std::to_string(z) << ','
                << std::to_string(Path[i].direct.x) << ','
                << std::to_string(Path[i].direct.y) << ','
                << std::to_string(Path[i].tag) << std::endl;
        }
        else {
            std::cerr << "Warning: Point (" << Path[i].pos.x << ", " << Path[i].pos.y
                << ") is out of DEM map bounds." << std::endl;
        }
    }
}

void tagCurOrLine(std::vector<posDirect2D>& Path, double scTH) {
    int pathSize = Path.size();
    scTH = scTH * M_PI / 180;
    for (int i = 0; i < pathSize - 1; i++) {
        double a = angleBetweenVector(Path[i].direct, Path[i + 1].direct);
        if (a < scTH) {
            if (Path[i].tag != 2) {
                Path[i].tag = 1;
            }
        }
        else {
            Path[i].tag = -1;
        }
    }
    Path[pathSize - 1].tag = 0;
}
void short_planning(posDirect2D start, posDirect2D end, std::vector<posDirect2D>& Path, double r = 0.1)
{
    double D = disBetweenPoints(start.pos, end.pos);
    double ang = angleBetweenVector(start.direct, end.direct);
    xy startToEnd = normalizeVect(vectAtoB(start.pos, end.pos));
    xy v0 = { 1, 0 };
    
    posDirect2D curNode = start;
    if (D < robot_foot_length && abs(ang)<0.1)
    {
        curNode.pos.x = (start.pos.x + end.pos.x) / 2.0;
        curNode.pos.y = (start.pos.y + end.pos.y) / 2.0;
        curNode.direct = start.direct;
        curNode.tag = 3;
        Path.push_back(curNode);
    }
    else
    {
        bool firstPointAdded = false;
        Path.push_back(start);
        while (D > r || abs(ang) > 0.1)
        {
            double da = min(D, max_line_forward);
            xy vst;
            if (abs(ang) > 0.1)
            {
                xy v1 = end.direct;
                vst = newRotationAngle(da, v0, v1);
            }
            else
            {
                vst = curNode.direct;
            }
            posDirect2D newNode;
            newNode.pos.x = curNode.pos.x + da * startToEnd.x;
            newNode.pos.y = curNode.pos.y + da * startToEnd.y;
            newNode.direct = vst;
            newNode.tag = firstPointAdded ? 1 : 3;
            Path.push_back(newNode);
            curNode = newNode;
            D = disBetweenPoints(curNode.pos, end.pos);
            ang = angleBetweenVector(curNode.direct, end.direct);
            firstPointAdded = true;
        }
        Path.push_back(end);
    }
    //每一段的第一点的tag修改为3 <-- 在后五步落足点生成中使用
}
//长距离质心轨迹规划
void planning(posDirect2D start, posDirect2D end, std::vector<posDirect2D>& Path, double r = 0.1) 
{
    double msa = sqrt(dotOfVec(start.direct, start.direct));
    start.direct = { start.direct.x / msa, start.direct.y / msa };
    double mea = sqrt(dotOfVec(end.direct, end.direct));
    end.direct = { end.direct.x / mea, end.direct.y / mea };
    xy v0 = { 1, 0 };
    double a = sgnOfk(v0, end.direct) * angleBetweenVector(v0, end.direct);
    double x, y;
    int ks = 0;
    int ke = 0;
    xy v1 = { -end.direct.x, -end.direct.y };
    xy nea = v1;
    double D = disBetweenPoints(start.pos, end.pos);
    std::vector<posDirect2D> Pe = { {end.pos, nea} };
    xy vet = nea;
    xy epc = end.pos;
    std::vector<posDirect2D> Ps = { start };
    xy vst = start.direct;
    xy spc = start.pos;
    double da;
    double ang = angleBetweenVector(start.direct, end.direct);

    while (D>r)
    {
        //在当前的起点和终点之间的距离很小时结束迭代
        //计算终点反方向矢量与终点到起点的矢量之间的夹角
        v0 = vectAtoB(epc, spc);
        v1 = vet;
        da = cal_da(v0, v1);
        //尝试一下当前的方向时顺时针运动还是逆时针运动。向哪个转向运动后的指向与当前终点到起点的矢量之间的夹角减小就用哪个方向旋转
        //顺时针旋转da角度

        vet = newRotationAngle(da, v0, v1);
        //计算转向后朝向矢量与x轴正方向的夹角
        v0 = { 1,0 };
        v1 = vet;
        a = sgnOfk(v0, v1) * angleBetweenVector(v0, v1);
        //在转向方向上从当前点前进r距离后的位置(x,y)
        //把这个点位置记录下来，记录到终点反向序列上
        epc = calNextPosition(Pe[ke].pos.x, Pe[ke].pos.y, a, r);
        ke++;
        Pe.push_back({ epc,vet });

        ///////////////////////////////////////////////////////
        //计算起点方向矢量与起点到终点的矢量之间的夹角，起点朝向不进行反向处理
        v0 = vectAtoB(spc, epc);
        v1 = vst;
        da = cal_da(v0, v1);
        vst = newRotationAngle(da, v0, v1);
        //计算转向后朝向矢量与x轴正方向的夹角
        v0 = { 1,0 };
        a = sgnOfk(v0, v1) * angleBetweenVector(v0, v1);
        spc = calNextPosition(Ps[ks].pos.x, Ps[ks].pos.y, a, r);
        ks++;
        Ps.push_back({ spc,vst });

        D = sqrt(dotOfVec(vectAtoB(spc, epc), vectAtoB(spc, epc)));
    }
    Path.insert(Path.end(), Ps.begin(), Ps.end());
    int peSize = Pe.size();
    posDirect2D nextNode;
    for (int i = peSize - 2; i >= 0; i--) {
        nextNode.pos = Pe[i].pos;
        nextNode.direct = { -Pe[i].direct.x, -Pe[i].direct.y };
        nextNode.direct = normalizeVect(nextNode.direct);
        nextNode.tag = Pe[i].tag;
        Path.push_back(nextNode);
    }
    
    double scTH = 1;
    tagCurOrLine(Path, scTH);
}

xy oneFootNextPos(xy verVect, xy centerPos, double r) {
    xy v0 = { 1, 0 };
    xy v1 = verVect;
    double a = sgnOfk(v0, v1) * angleBetweenVector(v0, v1);
    xy nextPos = calNextPosition(centerPos.x, centerPos.y, a, r);
    return nextPos;
}

posDirect2D genInterpolationNew(posDirect2D pd1, posDirect2D pd2, int j) {
    double k = (j + 1) / interpolation_multiple;
    posDirect2D res;
    xy tmpPos = vecMulC(vectAtoB(pd1.pos, pd2.pos), k);
    res.pos = vectAaddB(pd1.pos, tmpPos);
    xy tmpDirect = vecMulC(vectAtoB(pd1.direct, pd2.direct), k);
    res.direct = normalizeVect(vectAaddB(pd1.direct, tmpDirect));
    res.tag = sgnTag(pd1.tag + k * (pd2.tag - pd1.tag));
    return res;
}

void feetPlanning(std::vector<posDirect2D> Path, std::vector<posDirect2D>& leftPath, std::vector<posDirect2D>& rightPath, const cv::Mat& elevationMap,const cv::Mat& edges) {
    int pathSize = Path.size();
    std::vector <posDirect2D> LPath, RPath;
    for (int i = 0; i < pathSize; i++) {
        xy VerticalDireleft = { Path[i].direct.y, -Path[i].direct.x };
        xy VerticalDireRight = { -Path[i].direct.y, Path[i].direct.x };
        posDirect2D newLeft, newRight;
        newLeft.pos = oneFootNextPos(VerticalDireleft, Path[i].pos, robot_feet_to_centerL);
        newLeft.direct = Path[i].direct;
        newLeft.tag = Path[i].tag;
        LPath.push_back(newLeft);
        newRight.pos = oneFootNextPos(VerticalDireRight, Path[i].pos, robot_feet_to_centerR);
        newRight.direct = Path[i].direct;
        newRight.tag = Path[i].tag;
        RPath.push_back(newRight);
    }
    //string fnameLeft = "file/leftRawPath.csv";
    //dataToFile(LPath, fnameLeft);
    //string fnameRight = "file/rightRawPath.csv";
    //dataToFile(RPath, fnameRight);
    //int Ls = 0;
    //int Rs = 0;

    for (int i = 0; i < pathSize - 1; i++) {
        //Ls++;
        leftPath.push_back(LPath[i]);
        rightPath.push_back(RPath[i]);
        int edgeCur = edges.at<uchar>(Path[i].pos.y / perPixelWidth, Path[i].pos.x / perPixelWidth);
        int edgeNext = edges.at<uchar>(Path[i + 1].pos.y / perPixelWidth, Path[i + 1].pos.x / perPixelWidth);

        if ((edgeCur == 0 && edgeNext > 0) || (edgeCur > 0 && edgeNext == 0)) 
        {
            for (int j = 0; j < interpolation_multiple - 1; j++) {
                posDirect2D newOne = genInterpolationNew(LPath[i], LPath[i + 1], j);
                leftPath.push_back(newOne);
                newOne = genInterpolationNew(RPath[i], RPath[i + 1], j);
                rightPath.push_back(newOne);
            }
        }
        
    }
    pathSize = leftPath.size();
    for (int i = 0; i < pathSize - 1; i++) {
        if (leftPath[i].tag == 0) {
            leftPath[i].tag = leftPath[i + 1].tag;
        }
        if (rightPath[i].tag == 0) {
            rightPath[i].tag = rightPath[i + 1].tag;
        }
    }
}
std::pair<posDirect2D, posDirect2D>calStartStop(std::vector<pixDir>path)
{
    //离起点 and 终点要有半只脚长度的距离
    int halfFootLen = robot_foot_length / perPixelWidth / 2;
    int curLen = path.size();
    pixDir startPix = path[0];
    pixDir endPix = path[curLen-1];
    bool findStart = false;
    for (int j = 0; j < curLen; j++)
    {
        if (findStart==false && disBetwPix(path[0].pos, path[j].pos) > halfFootLen)
        {
            startPix = path[j];
            findStart = true;
        }
        if (disBetwPix(path[j].pos, path[curLen - 1].pos) <= halfFootLen)
        {
            endPix = path[j];
            break;
        }

    }
    posDirect2D start, end;
    start.pos.x = startPix.pos.x * perPixelWidth;
    start.pos.y = startPix.pos.y * perPixelWidth;
    start.direct.x = startPix.dir.x;
    start.direct.y = startPix.dir.y;
    start.tag = 0;
    end.pos.x = endPix.pos.x * perPixelWidth;
    end.pos.y = endPix.pos.y * perPixelWidth;
    end.direct.x = endPix.dir.x;
    end.direct.y = endPix.dir.y;
    end.tag = 0;
    return std::make_pair(start, end);
}
// Mouse callback variables
cv::Rect zoomRect;
bool dragging = false;
cv::Point origin;

// Callback function for mouse events
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    cv::Mat& img = *(cv::Mat*)userdata;

    if (event == cv::EVENT_LBUTTONDOWN) {
        dragging = true;
        origin = cv::Point(x, y);
        zoomRect = cv::Rect(x, y, 0, 0);
    }
    else if (event == cv::EVENT_MOUSEMOVE && dragging) {
        zoomRect.width = x - zoomRect.x;
        zoomRect.height = y - zoomRect.y;
    }
    else if (event == cv::EVENT_LBUTTONUP) {
        dragging = false;
        if (zoomRect.width > 0 && zoomRect.height > 0) {
            cv::Mat zoomedImg = img(zoomRect).clone();
            cv::resize(zoomedImg, img, img.size());
            cv::imshow("Path Visualization", img);
        }
    }
}
std::vector<pixDir> computePathWithDirections(const std::vector<pixDir>& path) {
    std::vector<pixDir> pathWithDirections = path;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        pixDir& current = pathWithDirections[i];
        pixDir& next = pathWithDirections[i + 1];
        current.dir.x = next.pos.x - current.pos.x;
        current.dir.y = next.pos.y - current.pos.y;
    }
    // For the last point, direction can be set to (0,0) or copied from the previous point
    if (!pathWithDirections.empty()) {
        pathWithDirections.back().dir.x = 0;
        pathWithDirections.back().dir.y = 0;
    }
    return pathWithDirections;
}
int genernateFeetPos(cv::Point start, cv::Point goal,std::string fnameCenterPath,std::string fnameleftPath,std::string fnamerightPath)
{
    std::string imagePath = "img/ProcessedDem.jpg";  // Adjust the path to your image file

    cv::Mat elevationMap = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
    if (elevationMap.empty()) {
        std::cout << "Error: Could not open or find the image!" << std::endl;
        return -1;
    }
    std::unordered_set<int> visitedPoints;
    //在图中找到路径
    std::vector<pixDir> aStarSearchedPath = aStar(elevationMap, start, goal, visitedPoints);
    std::vector<pixDir> pathPoints = computePathWithDirections(aStarSearchedPath);
    if (pathPoints.empty())
    {
        std::cout << "No path found!" << std::endl;
        return false;
    }
    else
    {
        // 计算梯度进行边缘检测
        cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y, grad;
        cv::Sobel(elevationMap, grad_x, CV_16S, 1, 0, 3);
        cv::Sobel(elevationMap, grad_y, CV_16S, 0, 1, 3);
        cv::convertScaleAbs(grad_x, abs_grad_x);
        cv::convertScaleAbs(grad_y, abs_grad_y);
        cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

        // 使用Canny进行边缘检测
        cv::Mat edges;
        cv::Canny(grad, edges, 50, 150);
        //分割A*返回的路径，规划质心轨迹
        std::vector<std::vector<pixDir>> segPaths = segmentPath( pathPoints, elevationMap, edges);
        std::vector<posDirect2D> centerPath;
        posDirect2D curStart = { {segPaths[0].front().pos.x,segPaths[0].front().pos.y},{segPaths[0].front().dir.x,segPaths[0].front().dir.y} };
        int halfFootLen = robot_foot_length / perPixelWidth / 2;
        std::vector<posDirect2D>  leftPath, rightPath;
        for (int i = 0; i < segPaths.size(); i++)
        {
            int curSegLen = segPaths[i].size();
            if (disBetwPix(segPaths[i].front().pos, segPaths[i].back().pos) * perPixelWidth < 5.0)
            {
                //小于5米用短距离规划，离起点 and 终点要有半只脚长度的距离 
                //直接走，直接转圈
                posDirect2D start_short, end_short;
                std::tie(start_short, end_short) = calStartStop(segPaths[i]);
                short_planning(start_short, end_short, centerPath);
            }
            else
            {
                //用长距离规划  ，离起点 and 终点要有半只脚长度的距离
                posDirect2D start_long, end_long;
                std::tie(start_long, end_long) = calStartStop(segPaths[i]);
                planning(start_long, end_long, centerPath);

            }

        }
        //规划双足的落脚点
        feetPlanning(centerPath, leftPath, rightPath, elevationMap,edges);
        std::string fnameCenter = "file/"+ fnameCenterPath;
        dataToFile(centerPath, fnameCenter, elevationMap);
        std::string fnameLeft = "file/"+ fnameleftPath;
        dataToFile(leftPath, fnameLeft, elevationMap);
        std::string fnameRight = "file/"+ fnamerightPath;
        dataToFile(rightPath, fnameRight, elevationMap);
        cv::Mat visImage;
        cv::cvtColor(elevationMap, visImage, cv::COLOR_GRAY2BGR);

        //for (const auto& p : centerPath)
        //{
        //    // Draw center point
        //    Point center(p.pos.x / perPixelWidth, p.pos.y / perPixelWidth);
        //    circle(visImage, center, 2, Scalar(0, 0, 255), -1);

        //    // Calculate angle from the direction vector
        //    double angle = atan2(p.direct.y, p.direct.x) * 180 / M_PI;

        //    // Draw rectangle
        //    Point2f rectPoints[4];
        //    RotatedRect rect(center, Size2f(robot_foot_length / perPixelWidth, robot_foot_width / perPixelWidth), angle);
        //    rect.points(rectPoints);
        //    for (int j = 0; j < 4; ++j) {
        //        line(visImage, rectPoints[j], rectPoints[(j + 1) % 4], Scalar(255, 0, 0), 1);
        //    }
        //}
        for (const auto& p : leftPath)
        {
            // Draw center point
            cv::Point center(p.pos.x / perPixelWidth, p.pos.y / perPixelWidth);
            cv::circle(visImage, center, 2, cv::Scalar(84, 255, 159), -1);

            // Calculate angle from the direction vector
            double angle = atan2(p.direct.y, p.direct.x) * 180 / M_PI;

            // Draw rectangle
            cv::Point2f rectPoints[4];
            cv::RotatedRect rect(center, cv::Size2f(robot_foot_length / perPixelWidth, robot_foot_width / perPixelWidth), angle);
            rect.points(rectPoints);
            for (int j = 0; j < 4; ++j) {
                cv::line(visImage, rectPoints[j], rectPoints[(j + 1) % 4], cv::Scalar(255, 0, 0), 1);
            }
        }
        for (const auto& p : rightPath)
        {
            // Draw center point
            cv::Point center(p.pos.x / perPixelWidth, p.pos.y / perPixelWidth);
            cv::circle(visImage, center, 2, cv::Scalar(84, 255, 159), -1);

            // Calculate angle from the direction vector
            double angle = atan2(p.direct.y, p.direct.x) * 180 / M_PI;

            // Draw rectangle
            cv::Point2f rectPoints[4];
            cv::RotatedRect rect(center, cv::Size2f(robot_foot_length / perPixelWidth, robot_foot_width / perPixelWidth), angle);
            rect.points(rectPoints);
            for (int j = 0; j < 4; ++j) {
                line(visImage, rectPoints[j], rectPoints[(j + 1) % 4], cv::Scalar(255, 0, 0), 1);
            }
        }
        //for (const auto& p : pathPoints)
        //{
        //    visImage.at<Vec3b>(p.pos.y, p.pos.x) = Vec3b(0, 255, 0);  // Mark path with red color

        //}
        std::cout<<"路径分为了几段：" << segPaths.size() << std::endl;
        //for (const auto& p : segPaths[0])
        //{
        //    visImage.at<Vec3b>(p.pos.y, p.pos.x) = Vec3b(0, 255, 0);  // Mark path with red color

        //}
        //for (const auto& p : segPaths[1])
        //{
        //    visImage.at<Vec3b>(p.pos.y, p.pos.x) = Vec3b(255,192, 203);  // Mark path with red color

        //}
        //for (const auto& p : segPaths[2])
        //{
        //    visImage.at<Vec3b>(p.pos.y, p.pos.x) = Vec3b(255, 0, 255);  // Mark path with red color

        //}
        //for (const auto& p : segPaths[3])
        //{
        //    visImage.at<Vec3b>(p.pos.y, p.pos.x) = Vec3b(0, 255, 0);  // Mark path with red color

        //}
        //for (const auto& p : segPaths[4])
        //{
        //    visImage.at<Vec3b>(p.pos.y, p.pos.x) = Vec3b(255, 192, 203);  // Mark path with red color

        //}
        cv::imwrite("img/PathFootStep.jpg", visImage);
        //cv::namedWindow("Path Visualization", cv::WINDOW_NORMAL);
        //cv::setMouseCallback("Path Visualization", mouseCallback, &visImage);
        //cv::imshow("Path Visualization", visImage);
        //cv::waitKey(0);
    }
        
    return true;
}
