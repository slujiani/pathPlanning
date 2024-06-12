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
#include <Windows.h>

using namespace cv;
using namespace std;
namespace plt = matplotlibcpp;
//robot属性
#define maxa 10*M_PI/180
#define robot_feet_to_centerL 0.1
#define robot_feet_to_centerR 0.1
#define deflectionAngleRate 0.25  // 偏转角度的比例系数，在（0,1）之间取值，一般取0.25
#define max_line_transverse 0.4//机器人走直线一步最大走0.4m
#define max_line_forward 0.4
#define max_line_backward 0.2//机器人直线后退步长
#define robot_foot_width 0.12 //机器人脚的宽度
#define robot_foot_length 0.265 //机器人脚的长度

//路径规划参数
#define interpolation_multiple 10.0 //插值倍数

struct PathNode {
    int x, y;
    float cost, priority;

    bool operator>(const PathNode& other) const {
        return priority > other.priority;
    }
};
// Utility functions for foot planning
struct Position {
    double x;
    double y;
};
struct posDirect {
    Position pos;
    Position direct;
    int tag; //1表示直线上的点，-1表示曲线上的点，0表示终点
};
struct pixDir
{
    Point pos;
    Point dir;
};
int disBetwPix(Point a, Point b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}
float heuristic(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

const float maxStepHeight = 0.25; // 最大抬脚高度，单位m
const float perPixelHeight = 1.8 / 255.0; //每个像素代表的高度，单位是m
const float perPixelWidth = 0.01; //图片坐标好像一个像素点代表一厘米，你看对不对
const float robotWidth = 0.54; //机器人宽度0.6米
const float stepSize = 0.2 / perPixelWidth; //机器人一步的大小
const float distanceThreshold = 5.0; // 距离阈值，单位米

bool isCollisionFree(const Mat& elevationMap, int x, int y,Point direction) {
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

vector<pixDir> aStar(const Mat& elevationMap, Point start, Point goal, unordered_set<int>& visitedPoints) {
    auto start_time = std::chrono::high_resolution_clock::now();  // Start timing
    priority_queue<PathNode, vector<PathNode>, greater<PathNode>> openList;
    unordered_map<int, pixDir> cameFrom;
    unordered_map<int, float> costSoFar;

    auto hashPoint = [&elevationMap](int x, int y) { return y * elevationMap.cols + x; };

    openList.push({ start.x, start.y, 0, heuristic(start.x, start.y, goal.x, goal.y) });
    cameFrom[hashPoint(start.x, start.y)] = { {start.x , start.y }, {0, 0} };
    costSoFar[hashPoint(start.x, start.y)] = 0;

    vector<Point> directions = { {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1} };

    while (!openList.empty()) {
        PathNode current = openList.top();
        openList.pop();

        if (current.x == goal.x && current.y == goal.y) {
            auto end_time = std::chrono::high_resolution_clock::now();  // End timing
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            cout << "Path found in " << duration << " ms" << endl;

            vector<pixDir> path;
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

        for (const Point& dir : directions) {
            int newX = current.x + dir.x;
            int newY = current.y + dir.y;
            visitedPoints.insert(hashPoint(newX, newY));
            if (newX >= 0 && newX < elevationMap.cols && newY >= 0 && newY < elevationMap.rows) {
                float heightDiff = abs(elevationMap.at<uchar>(newY, newX) - elevationMap.at<uchar>(current.y, current.x)) * perPixelHeight;
                if (!isCollisionFree(elevationMap, newX, newY, dir)) {
                    continue; // 高度差超过最大抬脚高度，跳过
                }
                if (elevationMap.at<uchar>(newY, newX) == 255) {
                    continue;
                }
                float newCost = costSoFar[hashPoint(current.x, current.y)] + heightDiff * 0.5 + 1; // Reduce the impact of height difference


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
uchar curAreaHeight(const Mat& elevationMap, int width, int length, int x, int y)
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
//分割路径
vector<vector<pixDir>> segmentPath(const vector<pixDir>& path, const Mat& elevationMap, const Mat& edgeMap)
{
    vector<vector<pixDir>> segmentedPaths;
    vector<pixDir> currentSegment;
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
        segmentedPaths.push_back(currentSegment);
    }
    return segmentedPaths;
}

double dotVector(Position a, Position b) {
    return a.x * b.x + a.y * b.y;
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

int sgnOfk(Position v0, Position v1) {
    return sgn(-(v0.x * v1.y - v0.y * v1.x));
}

double angleBetweenVector(Position a, Position b) {
    double dot = a.x * b.x + a.y * b.y;
    double len1 = sqrt(a.x * a.x + a.y * a.y);
    double len2 = sqrt(b.x * b.x + b.y * b.y);
    double tmp = dot / (len1 * len2);
    return acos(fmin(fmax(tmp, -1.0), 1.0));
}

double disBetweenPoints(Position a, Position b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

Position vectAtoB(Position a, Position b) {
    Position res = { b.x - a.x, b.y - a.y };
    return res;
}

Position vectAmulC(Position a, double alpha) {
    Position res = { alpha * a.x, alpha * a.y };
    return res;
}

Position vectAaddB(Position a, Position b) {
    Position res = { a.x + b.x, a.y + b.y };
    return res;
}

Position normalizeVect(Position a) {
    double N = sqrt(a.x * a.x + a.y * a.y);
    Position res = { (1 / N) * a.x, (1 / N) * a.y };
    return res;
}

Position calNextPosition(double x0, double y0, double a, double r) {
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
    Position p1 = { x, y };
    return p1;
}

double cal_da(Position v0, Position v1) {
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

Position newRotationAngle(double da, Position v0, Position& v1) {
    Position tmpA = { cos(da), -sin(da) };
    Position tmpB = { sin(da), cos(da) };
    Position vetp = { dotVector(tmpA, v1), dotVector(tmpB, v1) };
    tmpA = { cos(-da), -sin(-da) };
    tmpB = { sin(-da), cos(-da) };
    Position vetn = { dotVector(tmpA, v1), dotVector(tmpB, v1) };
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

void dataToFile(std::vector<posDirect> Path, string fname) {
    std::ofstream outFile(fname, ios::out);
    int peSize = Path.size();
    for (int i = 0; i < peSize; i++) {
        outFile << to_string(Path[i].pos.x) << ',' << to_string(Path[i].pos.y) << ',' << to_string(Path[i].direct.x) << ',' << to_string(Path[i].direct.y) << ',' << to_string(Path[i].tag) << endl;
    }
}

void tagCurOrLine(std::vector<posDirect>& Path, double scTH) {
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
void short_planning(posDirect start, posDirect end, std::vector<posDirect>& Path, double r = 0.1)
{
    double D = disBetweenPoints(start.pos, end.pos);
    double ang = angleBetweenVector(start.direct, end.direct);
    Position startToEnd = normalizeVect(vectAtoB(start.pos, end.pos));
    Position v0 = { 1, 0 };
    
    posDirect curNode = start;
    if (D < robot_foot_length && abs(ang)<0.1)
    {
        curNode.pos.x = (start.pos.x + end.pos.x) / 2.0;
        curNode.pos.y = (start.pos.y + end.pos.y) / 2.0;
        curNode.direct = start.direct;
        curNode.tag = 1;
        Path.push_back(curNode);
    }
    else
    {
        //Path.push_back(start);
        while (D > r || abs(ang) > 0.1)
        {
            double da = min(D, max_line_forward);
            Position vst;
            if (abs(ang) > 0.1)
            {
                Position v1 = end.direct;
                vst = newRotationAngle(da, v0, v1);
            }
            else
            {
                vst = curNode.direct;
            }
            posDirect newNode;
            newNode.pos.x = curNode.pos.x + da * startToEnd.x;
            newNode.pos.y = curNode.pos.y + da * startToEnd.y;
            newNode.direct = vst;
            newNode.tag = 3;
            Path.push_back(newNode);
            curNode = newNode;
            D = disBetweenPoints(curNode.pos, end.pos);
            ang = angleBetweenVector(curNode.direct, end.direct);
        }
        Path.push_back(end);
    }
   
}
//长距离质心轨迹规划
void planning(posDirect start, posDirect end, std::vector<posDirect>& Path, double r = 0.1) 
{
    double msa = sqrt(dotVector(start.direct, start.direct));
    start.direct = { start.direct.x / msa, start.direct.y / msa };
    double mea = sqrt(dotVector(end.direct, end.direct));
    end.direct = { end.direct.x / mea, end.direct.y / mea };
    Position v0 = { 1, 0 };
    double a = sgnOfk(v0, end.direct) * angleBetweenVector(v0, end.direct);
    double x, y;
    int ks = 0;
    int ke = 0;
    Position v1 = { -end.direct.x, -end.direct.y };
    Position nea = v1;
    double D = disBetweenPoints(start.pos, end.pos);
    vector<posDirect> Pe = { {end.pos, nea} };
    Position vet = nea;
    Position epc = end.pos;
    vector<posDirect> Ps = { start };
    Position vst = start.direct;
    Position spc = start.pos;
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

        D = sqrt(dotVector(vectAtoB(spc, epc), vectAtoB(spc, epc)));
    }
    Path.insert(Path.end(), Ps.begin(), Ps.end());
    int peSize = Pe.size();
    posDirect nextNode;
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

Position oneFootNextPos(Position verVect, Position centerPos, double r) {
    Position v0 = { 1, 0 };
    Position v1 = verVect;
    double a = sgnOfk(v0, v1) * angleBetweenVector(v0, v1);
    Position nextPos = calNextPosition(centerPos.x, centerPos.y, a, r);
    return nextPos;
}

posDirect genInterpolationNew(posDirect pd1, posDirect pd2, int j) {
    double k = (j + 1) / interpolation_multiple;
    posDirect res;
    Position tmpPos = vectAmulC(vectAtoB(pd1.pos, pd2.pos), k);
    res.pos = vectAaddB(pd1.pos, tmpPos);
    Position tmpDirect = vectAmulC(vectAtoB(pd1.direct, pd2.direct), k);
    res.direct = normalizeVect(vectAaddB(pd1.direct, tmpDirect));
    res.tag = sgnTag(pd1.tag + k * (pd2.tag - pd1.tag));
    return res;
}

void feetPlanning(std::vector<posDirect> Path, std::vector<posDirect>& leftPath, std::vector<posDirect>& rightPath) {
    int pathSize = Path.size();
    std::vector <posDirect> LPath, RPath;
    for (int i = 0; i < pathSize; i++) {
        Position VerticalDireRight = { Path[i].direct.y, -Path[i].direct.x };
        Position VerticalDireleft = { -Path[i].direct.y, Path[i].direct.x };
        posDirect newLeft, newRight;
        newLeft.pos = oneFootNextPos(VerticalDireleft, Path[i].pos, robot_feet_to_centerL);
        newLeft.direct = Path[i].direct;
        newLeft.tag = Path[i].tag;
        LPath.push_back(newLeft);
        newRight.pos = oneFootNextPos(VerticalDireRight, Path[i].pos, robot_feet_to_centerR);
        newRight.direct = Path[i].direct;
        newRight.tag = Path[i].tag;
        RPath.push_back(newRight);
    }
    string fnameLeft = "file/leftRawPath.csv";
    dataToFile(LPath, fnameLeft);
    string fnameRight = "file/rightRawPath.csv";
    dataToFile(RPath, fnameRight);
    int Ls = 0;
    int Rs = 0;
    for (int i = 0; i < pathSize - 1; i++) {
        Ls++;
        leftPath.push_back(LPath[i]);
        rightPath.push_back(RPath[i]);
        for (int j = 0; j < interpolation_multiple - 1; j++) {
            posDirect newOne = genInterpolationNew(LPath[i], LPath[i + 1], j);
            leftPath.push_back(newOne);
            newOne = genInterpolationNew(RPath[i], RPath[i + 1], j);
            rightPath.push_back(newOne);
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
pair<posDirect, posDirect>calStartStop(vector<pixDir>path)
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
    posDirect start, end;
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
    return make_pair(start, end);
}
// Mouse callback variables
Rect zoomRect;
bool dragging = false;
Point origin;

// Callback function for mouse events
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    Mat& img = *(Mat*)userdata;

    if (event == EVENT_LBUTTONDOWN) {
        dragging = true;
        origin = Point(x, y);
        zoomRect = Rect(x, y, 0, 0);
    }
    else if (event == EVENT_MOUSEMOVE && dragging) {
        zoomRect.width = x - zoomRect.x;
        zoomRect.height = y - zoomRect.y;
    }
    else if (event == EVENT_LBUTTONUP) {
        dragging = false;
        if (zoomRect.width > 0 && zoomRect.height > 0) {
            Mat zoomedImg = img(zoomRect).clone();
            resize(zoomedImg, img, img.size());
            imshow("Path Visualization", img);
        }
    }
}
int main() 
{
    string imagePath = "ProcessedDem.jpg";  // Adjust the path to your image file

    Mat elevationMap = imread(imagePath, IMREAD_GRAYSCALE);
    if (elevationMap.empty()) {
        cout << "Error: Could not open or find the image!" << endl;
        return -1;
    }

    Point start(1302, 1142);  // Define start point
    //Point start(1361, 1155);  // Define start point
    Point goal(1183, 991);//efine goal point
    //Point goal(1150, 799);  // Define goal point
    //Point goal(1093, 874);  // Define goal point
    unordered_set<int> visitedPoints;
    //在图中找到路径
    vector<pixDir> pathPoints = aStar(elevationMap, start, goal, visitedPoints);

    if (pathPoints.empty())
    {
        cout << "No path found!" << endl;
    }
    else
    {
        // 计算梯度进行边缘检测
        Mat grad_x, grad_y, abs_grad_x, abs_grad_y, grad;
        Sobel(elevationMap, grad_x, CV_16S, 1, 0, 3);
        Sobel(elevationMap, grad_y, CV_16S, 0, 1, 3);
        convertScaleAbs(grad_x, abs_grad_x);
        convertScaleAbs(grad_y, abs_grad_y);
        addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

        // 使用Canny进行边缘检测
        Mat edges;
        Canny(grad, edges, 50, 150);
        //分割A*返回的路径，规划质心轨迹
        vector<vector<pixDir>> segPaths = segmentPath(pathPoints, elevationMap, edges);
        vector<posDirect> centerPath;
        posDirect curStart = { {segPaths[0].front().pos.x,segPaths[0].front().pos.y},{segPaths[0].front().dir.x,segPaths[0].front().dir.y} };
        int halfFootLen = robot_foot_length / perPixelWidth / 2;
        for (int i = 0; i < segPaths.size(); i++)
        {
            int curSegLen = segPaths[i].size();
            if (disBetwPix(segPaths[i].front().pos, segPaths[i].back().pos) * perPixelWidth < 5.0)
            {
                //小于5米用短距离规划，离起点 and 终点要有半只脚长度的距离 
                //直接走，直接转圈
                posDirect start_short, end_short;
                std::tie(start_short, end_short) = calStartStop(segPaths[i]);
                short_planning(start_short, end_short, centerPath);

            }
            else
            {
                //用长距离规划  ，离起点 and 终点要有半只脚长度的距离
                posDirect start_long, end_long;
                std::tie(start_long, end_long) = calStartStop(segPaths[i]);
                planning(start_long, end_long, centerPath);

            }

        }
        //规划双足的落脚点

        vector<posDirect>  leftPath, rightPath;

        //规划双足的落脚点
        //feetPlanning(centerPath, leftPath, rightPath);

        Mat visImage;
        cvtColor(elevationMap, visImage, COLOR_GRAY2BGR);

        for (const auto& p : centerPath)
        {
            // Draw center point
            Point center(p.pos.x / perPixelWidth, p.pos.y / perPixelWidth);
            circle(visImage, center, 2, Scalar(0, 0, 255), -1);

            // Calculate angle from the direction vector
            double angle = atan2(p.direct.y, p.direct.x) * 180 / M_PI;

            // Draw rectangle
            Point2f rectPoints[4];
            RotatedRect rect(center, Size2f(robot_foot_length / perPixelWidth, robot_foot_width / perPixelWidth), angle);
            rect.points(rectPoints);
            for (int j = 0; j < 4; ++j) {
                line(visImage, rectPoints[j], rectPoints[(j + 1) % 4], Scalar(255, 0, 0), 1);
            }
        }
        //for (const auto& p : pathPoints)
        //{
        //    visImage.at<Vec3b>(p.pos.y, p.pos.x) = Vec3b(0, 255, 0);  // Mark path with red color

        //}
        cout<<"路径分为了几段：" << segPaths.size() << endl;
        for (const auto& p : segPaths[0])
        {
            visImage.at<Vec3b>(p.pos.y, p.pos.x) = Vec3b(0, 255, 0);  // Mark path with red color

        }
        for (const auto& p : segPaths[1])
        {
            visImage.at<Vec3b>(p.pos.y, p.pos.x) = Vec3b(255,192, 203);  // Mark path with red color

        }
        for (const auto& p : segPaths[2])
        {
            visImage.at<Vec3b>(p.pos.y, p.pos.x) = Vec3b(255, 0, 255);  // Mark path with red color

        }
        for (const auto& p : segPaths[3])
        {
            visImage.at<Vec3b>(p.pos.y, p.pos.x) = Vec3b(0, 255, 0);  // Mark path with red color

        }
        //for (const auto& p : segPaths[4])
        //{
        //    visImage.at<Vec3b>(p.pos.y, p.pos.x) = Vec3b(255, 192, 203);  // Mark path with red color

        //}
        imwrite("PathFootStep.jpg", visImage);
        namedWindow("Path Visualization", WINDOW_NORMAL);
        setMouseCallback("Path Visualization", mouseCallback, &visImage);
        imshow("Path Visualization", visImage);
        waitKey(0);
    }
        
    return 0;
}
