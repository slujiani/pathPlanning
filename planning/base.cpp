#define _USE_MATH_DEFINES
#include <cmath>
#include "include/base.h"

// Function definitions
xy vecMulC(xy a, double C) {
    return { a.x * C, a.y * C };
}

double dotOfVec(xy a, xy b) {
    return a.x * b.x + a.y * b.y;
}

xy vecAsubB(xyz a, xyz b) {
    return { a.x - b.x, a.y - b.y };
}
xy vectAtoB(xy a, xy b) {
    xy res = { b.x - a.x, b.y - a.y };
    return res;
}
xy vectAaddB(xy a, xy b) {
    xy res = { a.x + b.x, a.y + b.y };
    return res;
}
xy normalizeVect(xy a) {
    double N = sqrt(a.x * a.x + a.y * a.y);
    xy res = { (1 / N) * a.x, (1 / N) * a.y };
    return res;
}

double dotOfxyz(xyz a, xyz b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

xyz xyzMulC(xyz a, double C) {
    return { a.x * C, a.y * C, a.z * C };
}

xyz normal_xyz(xyz a) {
    double norm = sqrt(dotOfxyz(a, a));
    return xyzMulC(a, 1.0 / norm);
}

int disBetwPix(cv::Point a, cv::Point b) {
    return static_cast<int>(sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)));
}

double angleBetweenVector(xy a, xy b) {
    double dot = a.x * b.x + a.y * b.y;
    double len1 = sqrt(a.x * a.x + a.y * a.y);
    double len2 = sqrt(b.x * b.x + b.y * b.y);
    double tmp = dot / (len1 * len2);
    return acos(fmin(fmax(tmp, -1.0), 1.0));
}

double disBetweenPoints(xy a, xy b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}
double dis2D(xyz a, xyz b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}