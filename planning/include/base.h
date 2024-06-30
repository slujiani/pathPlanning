#pragma once

#include <cmath>
#include <opencv2/core.hpp>

// Robot properties �滮����
constexpr double maxa = 10 * M_PI / 180;
constexpr double robot_feet_to_centerL = 0.1;
constexpr double robot_feet_to_centerR = 0.1;
constexpr double deflectionAngleRate = 0.25;  // Proportion coefficient of deflection angle, usually 0.25
constexpr double max_line_transverse = 0.4;  // Max step length in straight line
constexpr double max_line_forward = 0.4;
constexpr double max_line_backward = 0.2;  // Max step length when moving backward
constexpr double robot_foot_width = 0.12;  // Robot foot width
constexpr double robot_foot_length = 0.265;  // Robot foot length
constexpr double vs = 0.4;	// ֱ�������ٶȣ�ÿ����/��
constexpr double vc = 0.1;	// ���������ٶȣ�ÿ����/��
constexpr double lcTh = 1* M_PI / 180;	// ֱ��·��������·��ת��������ֵ����λ����
constexpr int replan_param = 5;//�ع滮�Ծ��뵱ǰ�������Ľ������ĵ� replan_param ��Ϊ�յ�
// Robot �͵�ͼ��ص�����
constexpr float maxStepHeight = 0.25; // ���̧�Ÿ߶ȣ���λm
constexpr float perPixelHeight = 1.8 / 255.0; // ÿ�����ش���ĸ߶ȣ���λ��m
constexpr float perPixelWidth = 0.01; // ͼƬ����һ�����ص����һ����
constexpr float robotWidth = 0.54; // �����˿��0.6��
constexpr float stepSize = 0.2 / perPixelWidth; // ������һ���Ĵ�С
constexpr float distanceThreshold = 5.0; // ������ֵ����λ��

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
