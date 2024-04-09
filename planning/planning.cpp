#define _USE_MATH_DEFINES
#include<iostream>
#include<tuple>
#include<math.h>
#include<cmath>
#include<vector>
#include<matplotlibcpp.h>

using namespace std;
namespace plt = matplotlibcpp;

struct Point
{
    double x, y;
};

struct posVector
{
    ::Point position;//坐标
    double direction[2];//朝向
};

// typedef struct pointCp
// {
//     double sp[2]; //起点坐标
//     double sa[2]; //起点位置时机器人朝向的方向矢量
//     double ep[2]; //终点坐标
//     double ea[2]; //终点位置时机器人朝向的方向矢量
// }POINTCP;


//计算一个向量的两个切圆
std::tuple<double, double, double, double>tangentCircleCenter(::Point position, double direct[2], double r = 1)
{
    double x0 = position.x;
    double y0 = position.y;
    double v = direct[0];
    double u = direct[1];
    double a1, a2, b1, b2, tmp;
    if (fabs(u) > fabs(v) && fabs(u) > 1e-3)
    {
        tmp = fabs(u * r) / sqrt(pow(u, 2) + pow(v, 2));
        a1 = x0 - tmp;
        a2 = x0 + tmp;
        b1 = y0 + (x0 - a1) * v / u;
        b2 = y0 + (x0 - a2) * v / u;
    }
    else if (fabs(v) > fabs(u) && fabs(v) > 1e-3)
    {
        tmp = fabs(v * r) / sqrt(pow(u, 2) + pow(v, 2));
        b1 = y0 - tmp;
        b2 = y0 + tmp;
        a1 = x0 + (y0 - b1) * u / v;
        a2 = x0 + (y0 - b2) * u / v;
    }
    else
    {

    }
    return std::make_tuple(a1, b1, a2, b2);
}
//计算两点的距离
double disPointToPoint(double a1, double b1, double a2, double b2)
{
    return pow(a1 - a2, 2) + pow(b1 - b2, 2);
}
std::tuple<double, double> findNearCircle(posVector start, posVector end)
{
    // 在起点做半径为r的圆，该圆与方向矢量相切
    // 与起点方向矢量相切半径为r的圆有两个，圆心位置分别为(a1,b1)和(a2,b2)
    double a1, b1, a2, b2;
    std::tie(a1, b1, a2, b2) = tangentCircleCenter(start.position, start.direction);
    // 分别计算两个圆的圆心与终点的距离，L1，L2
    double L1 = disPointToPoint(a1, b1, end.position.x, end.position.y);
    double L2 = disPointToPoint(a2, b2, end.position.x, end.position.y);
    //选择与终点最近的圆,圆心坐标为(sox,soy)
    double sox;
    double soy;
    if (L1 <= L2)
    {
        sox = a1;
        soy = b1;
    }
    else
    {
        sox = a2;
        soy = b2;
    }
    return std::make_tuple(sox, soy);
}
::Point calLineCircleTangencyPoint(double l1, double l2, double x0, double y0, double r)
{
    ::Point res = {};
    double ox, oy;
    if (fabs(l1) > fabs(l2) && fabs(l1) > 1e-3)
    {
        oy = -(l2 - pow(l1, 2) * y0 + l1 * l2 * x0) / (pow(l1, 2) + pow(l2, 2));
        ox = (-1 / l1) * (l2 * oy + 1);
    }
    else if (fabs(l2) > fabs(l1) && fabs(l2) > 1e-3)
    {
        ox = -(l1 - pow(l2, 2) * x0 + l1 * l2 * y0) / (pow(l1, 2) + pow(l2, 2));
        oy = (-1 / l2) * (l1 * ox + 1);
    }
    else
    {
        return res; // 返回一个空向量
    }
    res = { ox,oy };
    return res;
}
//返回两个向量的夹角
double angleBetweenPoints(double ax,double ay, double bx,double by)
{
    double dot = ax * bx + ay * by;
    double len1 = sqrt(ax * ax + ay * ay);
    double len2 = sqrt(bx * bx + by * by);
    return acos(dot / (len1 * len2));
}
void plotCircle(double a, double b, double r)
{
    int n = 5000;
    vector<double> x(n), y(n);
    for (int i = 0; i < n; ++i) {
        double t = 2 * M_PI * i / n;
        x.at(i) = a + r * cos(t);
        y.at(i) = b + r * sin(t);
    }
    plt::plot(x, y);
}
//起点的位置和方向，终点的位置和方向，画圆的半径  返回参数S
void planning(posVector start, posVector end, std::vector<double>& S,double radius = 1.0 )
{
    //找起点的圆
    double sox, soy, eox, eoy;
    std::tie(sox, soy) = findNearCircle(start, end);
    //找终点的圆
    std::tie(eox, eoy) = findNearCircle(end, start);

     // 计算圆c1: （x-a1)^2+(y-b1)^2=pow(d1,2)和圆C2: (x-a2)^2+(y-b2)^2=pow(d2,2)的共切
     //线，即直线同时与圆c1和圆c2相切
     //两个圆的共切线有几种不同情况，分别是：
     //一个圆在另一个圆内部，这时没有共切线 
     //两个圆重合，这时有无数条通切线
     //两个圆有交点（一个或两个交点),这时有两条共切线
     //两个圆无交点，这时有4条共切线
     //该程序是起点和终点较远，机器人在长距离进行连续运动时的规划
     //这时两个圆无交点，4条共切线的一般方程为
     //l1: l11*x + l21*y + 1 = 0;
     //l2: l12*x + l22*y + 1 = 0;
     //l3: l13*x + l23*y + 1 = 0;
     //l4: l14*x + l24*y + 1 = 0;
     //下面用解析方法估计4条切线的参数(l11,l12),(l21,l22),(l31,l32),(l41,l42)
    double a1 = sox;
    double b1 = soy;
    double d1 = 1;
    double a2 = eox;
    double b2 = eoy;
    double d2 = 1;
    double com1 = (d1 - d2) / (a1 * d2 - a2 * d1);
    double com2 = b1 * d2 - b2 * d1;
    double com3 = pow(a1, 2) * b2 + pow(a2, 2) * b1 - b1 * pow(d2, 2) - b2 * pow(d1, 2) - a1 * a2 * b1 - a1 * a2 * b2 + b1 * d1 * d2 + b2 * d1 * d2;
    double com4 = a1 * d2 * sqrt(pow(a1, 2) - 2 * a1 * a2 + pow(a2, 2) + pow(b1, 2) - 2 * b1 * b2 + pow(b2, 2) - pow(d1, 2) + 2 * d1 * d2 - pow(d2, 2));
    double com5 = a2 * d1 * sqrt(pow(a1, 2) - 2 * a1 * a2 + pow(a2, 2) + pow(b1, 2) - 2 * b1 * b2 + pow(b2, 2) - pow(d1, 2) + 2 * d1 * d2 - pow(d2, 2));
    double com6 = ((a1 * d2 - a2 * d1) * (-pow(a1, 2) * pow(b2, 2) + pow(a1, 2) * pow(d2, 2) + 2 * a1 * a2 * b1 * b2 - 2 * a1 * a2 * d1 * d2 - pow(a2, 2) * pow(b1, 2) + pow(a2, 2) * pow(d1, 2) + pow(b1, 2) * pow(d2, 2) - 2 * b1 * b2 * d1 * d2 + pow(b2, 2) * pow(d1, 2)));

    double l11 = com1 - (com2 * (com3 + com4 - com5)) / com6;

    double l12 = com1 - (com2 * (com3 - com4 + com5)) / com6;

    double l13 = ((b1 * d2 + b2 * d1) * (b1 * pow(d2, 2) - pow(a2, 2) * b1 - pow(a1, 2) * b2 + b2 * pow(d1, 2) + a1 * a2 * b1 + a1 * a2 * b2 + b1 * d1 * d2 + b2 * d1 * d2 + a1 * d2 * sqrt(pow(a1, 2) - 2 * a1 * a2 + pow(a2, 2) + pow(b1, 2) - 2 * b1 * b2 + pow(b2, 2) - pow(d1, 2) - 2 * d1 * d2 - pow(d2, 2)) + a2 * d1 * sqrt(pow(a1, 2) - 2 * a1 * a2 + pow(a2, 2) + pow(b1, 2) - 2 * b1 * b2 + pow(b2, 2) - pow(d1, 2) - 2 * d1 * d2 - pow(d2, 2)))) / ((a1 * d2 + a2 * d1) * (-pow(a1, 2) * pow(b2, 2) + pow(a1, 2) * pow(d2, 2) + 2 * a1 * a2 * b1 * b2 + 2 * a1 * a2 * d1 * d2 - pow(a2, 2) * pow(b1, 2) + pow(a2, 2) * pow(d1, 2) + pow(b1, 2) * pow(d2, 2) + 2 * b1 * b2 * d1 * d2 + pow(b2, 2) * pow(d1, 2))) - (d1 + d2) / (a1 * d2 + a2 * d1);

    double l14 = ((b1 * d2 + b2 * d1) * (b1 * pow(d2, 2) - pow(a2, 2) * b1 - pow(a1, 2) * b2 + b2 * pow(d1, 2) + a1 * a2 * b1 + a1 * a2 * b2 + b1 * d1 * d2 + b2 * d1 * d2 - a1 * d2 * sqrt(pow(a1, 2) - 2 * a1 * a2 + pow(a2, 2) + pow(b1, 2) - 2 * b1 * b2 + pow(b2, 2) - pow(d1, 2) - 2 * d1 * d2 - pow(d2, 2)) - a2 * d1 * sqrt(pow(a1, 2) - 2 * a1 * a2 + pow(a2, 2) + pow(b1, 2) - 2 * b1 * b2 + pow(b2, 2) - pow(d1, 2) - 2 * d1 * d2 - pow(d2, 2)))) / ((a1 * d2 + a2 * d1) * (-pow(a1, 2) * pow(b2, 2) + pow(a1, 2) * pow(d2, 2) + 2 * a1 * a2 * b1 * b2 + 2 * a1 * a2 * d1 * d2 - pow(a2, 2) * pow(b1, 2) + pow(a2, 2) * pow(d1, 2) + pow(b1, 2) * pow(d2, 2) + 2 * b1 * b2 * d1 * d2 + pow(b2, 2) * pow(d1, 2))) - (d1 + d2) / (a1 * d2 + a2 * d1);

    double l21 = (pow(a1, 2) * b2 + pow(a2, 2) * b1 - b1 * pow(d2, 2) - b2 * pow(d1, 2) - a1 * a2 * b1 - a1 * a2 * b2 + b1 * d1 * d2 + b2 * d1 * d2 + a1 * d2 * sqrt(pow(a1, 2) - 2 * a1 * a2 + pow(a2, 2) + pow(b1, 2) - 2 * b1 * b2 + pow(b2, 2) - pow(d1, 2) + 2 * d1 * d2 - pow(d2, 2)) - a2 * d1 * sqrt(pow(a1, 2) - 2 * a1 * a2 + pow(a2, 2) + pow(b1, 2) - 2 * b1 * b2 + pow(b2, 2) - pow(d1, 2) + 2 * d1 * d2 - pow(d2, 2))) / (-pow(a1, 2) * pow(b2, 2) + pow(a1, 2) * pow(d2, 2) + 2 * a1 * a2 * b1 * b2 - 2 * a1 * a2 * d1 * d2 - pow(a2, 2) * pow(b1, 2) + pow(a2, 2) * pow(d1, 2) + pow(b1, 2) * pow(d2, 2) - 2 * b1 * b2 * d1 * d2 + pow(b2, 2) * pow(d1, 2));
    double  l22 = (pow(a1, 2) * b2 + pow(a2, 2) * b1 - b1 * pow(d2, 2) - b2 * pow(d1, 2) - a1 * a2 * b1 - a1 * a2 * b2 + b1 * d1 * d2 + b2 * d1 * d2 - a1 * d2 * sqrt(pow(a1, 2) - 2 * a1 * a2 + pow(a2, 2) + pow(b1, 2) - 2 * b1 * b2 + pow(b2, 2) - pow(d1, 2) + 2 * d1 * d2 - pow(d2, 2)) + a2 * d1 * sqrt(pow(a1, 2) - 2 * a1 * a2 + pow(a2, 2) + pow(b1, 2) - 2 * b1 * b2 + pow(b2, 2) - pow(d1, 2) + 2 * d1 * d2 - pow(d2, 2))) / (-pow(a1, 2) * pow(b2, 2) + pow(a1, 2) * pow(d2, 2) + 2 * a1 * a2 * b1 * b2 - 2 * a1 * a2 * d1 * d2 - pow(a2, 2) * pow(b1, 2) + pow(a2, 2) * pow(d1, 2) + pow(b1, 2) * pow(d2, 2) - 2 * b1 * b2 * d1 * d2 + pow(b2, 2) * pow(d1, 2));
    double l23 = -(b1 * pow(d2, 2) - pow(a2, 2) * b1 - pow(a1, 2) * b2 + b2 * pow(d1, 2) + a1 * a2 * b1 + a1 * a2 * b2 + b1 * d1 * d2 + b2 * d1 * d2 + a1 * d2 * sqrt(pow(a1, 2) - 2 * a1 * a2 + pow(a2, 2) + pow(b1, 2) - 2 * b1 * b2 + pow(b2, 2) - pow(d1, 2) - 2 * d1 * d2 - pow(d2, 2)) + a2 * d1 * sqrt(pow(a1, 2) - 2 * a1 * a2 + pow(a2, 2) + pow(b1, 2) - 2 * b1 * b2 + pow(b2, 2) - pow(d1, 2) - 2 * d1 * d2 - pow(d2, 2))) / (-pow(a1, 2) * pow(b2, 2) + pow(a1, 2) * pow(d2, 2) + 2 * a1 * a2 * b1 * b2 + 2 * a1 * a2 * d1 * d2 - pow(a2, 2) * pow(b1, 2) + pow(a2, 2) * pow(d1, 2) + pow(b1, 2) * pow(d2, 2) + 2 * b1 * b2 * d1 * d2 + pow(b2, 2) * pow(d1, 2));
    double l24 = -(b1 * pow(d2, 2) - pow(a2, 2) * b1 - pow(a1, 2) * b2 + b2 * pow(d1, 2) + a1 * a2 * b1 + a1 * a2 * b2 + b1 * d1 * d2 + b2 * d1 * d2 - a1 * d2 * sqrt(pow(a1, 2) - 2 * a1 * a2 + pow(a2, 2) + pow(b1, 2) - 2 * b1 * b2 + pow(b2, 2) - pow(d1, 2) - 2 * d1 * d2 - pow(d2, 2)) - a2 * d1 * sqrt(pow(a1, 2) - 2 * a1 * a2 + pow(a2, 2) + pow(b1, 2) - 2 * b1 * b2 + pow(b2, 2) - pow(d1, 2) - 2 * d1 * d2 - pow(d2, 2))) / (-pow(a1, 2) * pow(b2, 2) + pow(a1, 2) * pow(d2, 2) + 2 * a1 * a2 * b1 * b2 + 2 * a1 * a2 * d1 * d2 - pow(a2, 2) * pow(b1, 2) + pow(a2, 2) * pow(d1, 2) + pow(b1, 2) * pow(d2, 2) + 2 * b1 * b2 * d1 * d2 + pow(b2, 2) * pow(d1, 2));

    //计算直线l1与起点处圆的切点
    vector<::Point>seo1;//seo1是切点坐标，第一个，第二个元素是l1在起点圆上的切点坐标，第三个和第四个是l1与终点圆上的切点坐标
    //计算直线l1与起点处圆的切点
    seo1.push_back(calLineCircleTangencyPoint(l11, l21, sox, soy, radius));
    //计算直线l1与终点处圆的切点
    seo1.push_back(calLineCircleTangencyPoint(l11, l21, eox, eoy, radius));

    ////////////////////
    //计算直线l2与两个圆的切点
    vector<::Point>seo2;
    seo2.push_back(calLineCircleTangencyPoint(l12, l22, sox, soy, radius));
    seo2.push_back(calLineCircleTangencyPoint(l12, l22, eox, eoy, radius));
    ////////////////////
    //l3
    vector<::Point>seo3;
    seo3.push_back(calLineCircleTangencyPoint(l13, l23, sox, soy, radius));
    seo3.push_back(calLineCircleTangencyPoint(l13, l23, eox, eoy, radius));
    ////////////////////
    //l4
    vector<::Point>seo4;
    seo4.push_back(calLineCircleTangencyPoint(l14, l24, sox, soy, radius));
    seo4.push_back(calLineCircleTangencyPoint(l14, l24, eox, eoy, radius));

    // 把起点的圆、终点的圆和4条切线画出来
    // 画圆
    plotCircle(eox, eoy, radius);
    plotCircle(sox, soy, radius);
    //四条线
    std::vector <vector<::Point >> four_line = { seo1,seo2,seo3,seo4 };
    vector<double> plot_x(2);
    vector<double> plot_y(2);
    for (int i = 0; i < 4; i++)
    {
        plot_x[0] = four_line[i][0].x;
        plot_x[1] = four_line[i][1].x;
        plot_y[0] = four_line[i][0].y;
        plot_y[1] = four_line[i][1].y;
        plt::plot(plot_x, plot_y);
    }   

    //有4条切线，机器人从起点圆到终点圆 走那一条直线？
    //一条切线与起点处机器人方向矢量之间的夹角a1,与终点处机器人的方向矢量的夹角a2,这两个角度之和最小的切线就是机器人要行走的直线段
    std::vector<double>a(4);
    
    for (int i = 0; i < 4; i++)
    {
        double a1 = angleBetweenPoints(four_line[i][0].x, four_line[i][0].y, start.direction[0], start.direction[1]);//和起点
        double a2 = angleBetweenPoints(four_line[i][1].x, four_line[i][1].y, end.direction[0], end.direction[1]);//和终点
        a[i] = a1 + a2;
    }
    auto minIndex = min_element(a.begin(), a.end())-a.begin();
    //cout << minIndex << endl;
    
    S.push_back(start.position.x);//起点坐标
    S.push_back(start.position.y);
    S.push_back(sox);//起点圆心坐标
    S.push_back(soy);
    switch (minIndex)//切线参数
    {
    case 0:
        S.push_back(l11);
        S.push_back(l21);
        break;
    case 1:
        S.push_back(l12);
        S.push_back(l22);
        break;
    case 2:
        S.push_back(l13);
        S.push_back(l23);
        break;
    case 3:
        S.push_back(l14);
        S.push_back(l24);
        break;
    default:
        break;
    }
    
    S.push_back(four_line[minIndex][0].x);//起点圆切点坐标
    S.push_back(four_line[minIndex][0].y);
    S.push_back(eox);//终点圆心坐标
    S.push_back(eoy);
    S.push_back(four_line[minIndex][1].x);//终点圆切点坐标
    S.push_back(four_line[minIndex][1].y);
    S.push_back(end.position.x);//终点坐标
    S.push_back(end.position.y);

    plt::show();

}

int main()
{
    posVector start = { 10,0,0,-1 };
    posVector end = { 6.5,1.8,1,0 };
    std:vector<double>output;
    planning(start, end, output);

}