#pragma once

#include <cmath>
#include <opencv2/core.hpp>

// Robot properties 规划属性
constexpr double maxa = 10 * M_PI / 180;
constexpr double robot_feet_to_centerL = 0.1;
constexpr double robot_feet_to_centerR = 0.1;
constexpr double deflectionAngleRate = 0.25;  // Proportion coefficient of deflection angle, usually 0.25
constexpr double max_line_transverse = 0.4;  // Max step length in straight line
constexpr double max_line_forward = 0.4;
constexpr double max_line_backward = 0.2;  // Max step length when moving backward
constexpr double robot_foot_width = 0.12;  // Robot foot width
constexpr double robot_foot_length = 0.265;  // Robot foot length
constexpr double vs = 0.4;	// 直线行走速度，每步米/秒
constexpr double vc = 0.1;	// 曲线行走速度，每步米/秒
constexpr double lcTh = 1* M_PI / 180;	// 直线路径和曲线路径转角区分阈值，单位：度
constexpr int replan_param = 5;//重规划以距离当前点最近点的接下来的第 replan_param 步为终点
// Robot 和地图相关的属性
constexpr float maxStepHeight = 0.25; // 最大抬脚高度，单位m
constexpr float perPixelHeight = 1.8 / 255.0; // 每个像素代表的高度，单位是m
constexpr float perPixelWidth = 0.01; // 图片坐标一个像素点代表一厘米
constexpr float robotWidth = 0.54; // 机器人宽度0.6米
constexpr float stepSize = 0.2 / perPixelWidth; // 机器人一步的大小
constexpr float distanceThreshold = 5.0; // 距离阈值，单位米

// Path planning parameters
constexpr double interpolation_multiple = 10.0;  // Interpolation multiplier



struct xy {
    double x;
    double y;
};

struct xyz {
    double x;
    double y;
    double z;
};

struct posDirect2D {
    xy pos;
    xy direct;
    int tag;  // 1: straight line, -1: curve, 0: endpoint
};

struct posDirect3D {
    xyz pos;
    xy direct;
    int tag;  // 1: straight line, -1: curve, 0: endpoint
};

struct pixDir {
    cv::Point pos;
    cv::Point dir;
};

struct PathNode {
    int x;
    int y;
    float cost;
    float priority;

    bool operator>(const PathNode& other) const {
        return priority > other.priority;
    }
};

// Function declarations
xy vecMulC(xy a, double C);
double dotOfVec(xy a, xy b);
xy vecAsubB(xyz a, xyz b);
xy vectAtoB(xy a, xy b);
xy vectAaddB(xy a, xy b);
xy normalizeVect(xy a);

double dotOfxyz(xyz a, xyz b);
xyz xyzMulC(xyz a, double C);
xyz normal_xyz(xyz a);

int disBetwPix(cv::Point a, cv::Point b);

double angleBetweenVector(xy a, xy b);
double disBetweenPoints(xy a, xy b);
double dis2D(xyz a, xyz b);
