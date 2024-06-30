#pragma once
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
#include "include/base.h"





// Function declarations
float heuristic(int x1, int y1, int x2, int y2);

bool isCollisionFree(const cv::Mat& elevationMap, int x, int y, cv::Point direction);

std::vector<pixDir> aStar(const cv::Mat& elevationMap, cv::Point start, cv::Point goal, std::unordered_set<int>& visitedPoints);

uchar curAreaHeight(const cv::Mat& elevationMap, int width, int length, int x, int y);



std::vector<std::vector<pixDir>> segmentPath(const std::vector<pixDir>& path, const cv::Mat& elevationMap, const cv::Mat& edgeMap);

int sgn(double k);

int sgnTag(double k);

int sgnOfk(xy v0, xy v1);

double angleBetweenVector(xy a, xy b);

double disBetweenPoints(xy a, xy b);

xy calNextPosition(double x0, double y0, double a, double r);

double cal_da(xy v0, xy v1);

xy newRotationAngle(double da, xy v0, xy& v1);

void dataToFile(std::vector<posDirect2D> Path, std::string fname, const cv::Mat& demMap);

void tagCurOrLine(std::vector<posDirect2D>& Path, double scTH);





xy oneFootNextPos(xy verVect, xy centerPos, double r);

posDirect2D genInterpolationNew(posDirect2D pd1, posDirect2D pd2, int j);

void feetPlanning(std::vector<posDirect2D> Path, std::vector<posDirect2D>& leftPath, std::vector<posDirect2D>& rightPath, const cv::Mat& elevationMap, const cv::Mat& edges);

std::pair<posDirect2D, posDirect2D> calStartStop(std::vector<pixDir> path);

void mouseCallback(int event, int x, int y, int flags, void* userdata);

int genernateFeetPos(cv::Point start, cv::Point goal, std::string fnameCenterPath, std::string fnameleftPath, std::string fnamerightPath);
