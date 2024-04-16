#define _USE_MATH_DEFINES
#include<iostream>
#include<tuple>
#include<math.h>
#include<cmath>
#include<vector>
#include<matplotlibcpp.h>

#define radStepLen 0.1 //圆弧上Robot走的步长
#define lineStepLen 0.4 //直线Robot走的步长
//#define sgn(x) (x>0)-(x<0)

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

double sgn(double x)
{
    if (x > 0)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

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
    return sqrt(pow(a1 - a2, 2) + pow(b1 - b2, 2));
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
    double cosV = acos(dot / (len1 * len2));
    return cosV;
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
void plotPathCircleStoreInPath(std::vector<Point>&Path,std::vector<double>S,double alpha,int startInd,int endInd,double radius,double sgnk,double da)
{
    double x0 = S[startInd];
    double y0 = S[startInd+1];
    double D1 = 100;
    double cox, coy;//画的圆心
    double L = radius * alpha; //弧长
    //double da = alpha / (trunc(L / radStepLen) + 1);
    do
    {
        alpha += sgnk * da;
        double abs_a = abs(alpha);
        if (alpha > -M_PI / 2 && alpha <= 0)
        {
            cox = x0 + radius * cos(abs_a);
            coy = y0 + radius * sin(abs_a);
        }
        else if (alpha > -M_PI && alpha <= -M_PI / 2)
        {
            cox = x0 - radius * cos(M_PI - abs_a);
            coy = y0 + radius * sin(M_PI - abs_a);
        }
        else if (alpha >= 0 && alpha < M_PI / 2)
        {
            cox = x0 + radius * cos(abs_a);
            coy = y0 - radius * sin(abs_a);
        }
        else
        {
            cox = x0 - radius * cos(M_PI - abs_a);
            coy = y0 - radius * sin(M_PI - abs_a);
        }
        Point newP = { cox,coy };
        Path.push_back(newP);
        plotCircle(cox, coy, da * radius);//每走一步画一个圆
        D1 = disPointToPoint(cox, coy, S[endInd], S[endInd+1]);
    } while (D1 > 0.05);
}
void plotPathLineStoreInPath(std::vector<Point>& Path, std::vector<double>S, double alpha, int startInd, int endInd, double dl)
{
    int k = 1;
    double x0 = S[startInd];//假设沿起点圆弧的最后位置点与起点圆上的切点位置很近，
    double y0 = S[startInd+1];//所以将起点圆上的切点作为临时的直线运动的起点
    double abs_a = abs(alpha);
    double cox, coy;//画的圆心
    //沿直线运动的轨迹
    double D1 = 100;
    do
    {
        if (alpha > -M_PI / 2 && alpha <= 0)
        {
            cox = x0 + k * dl * cos(abs_a);
            coy = y0 + k * dl * sin(abs_a);
        }
        else if (alpha > -M_PI && alpha <= -M_PI / 2)
        {
            cox = x0 - k * dl * cos(M_PI - abs_a);
            coy = y0 + k * dl * sin(M_PI - abs_a);
        }
        else if (alpha >= 0 && alpha < M_PI / 2)
        {
            cox = x0 + k * dl * cos(abs_a);
            coy = y0 - k * dl * sin(abs_a);
        }
        else
        {
            cox = x0 - k * dl * cos(M_PI - abs_a);
            coy = y0 - k * dl * sin(M_PI - abs_a);
        }


        k = k + 1;
        Point newP = { cox,coy };
        Path.push_back(newP);
        plotCircle(cox, coy, dl);//每走一步画一个圆
        D1 = disPointToPoint(cox, coy, S[endInd], S[endInd+1]);// 如果轨迹点很接近终点圆上的切点，就认为结束直线运动阶段，转入终点圆弧运动阶)段
    } while (D1 > 0.2);
}
//曲线运动
//起点的位置和方向，终点的位置和方向，画圆的半径  返回参数S
void CurvilinearMotionPlanning(posVector start, posVector end, std::vector<Point>& Path, double radius = 1.0)
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
    auto minIndex = min_element(a.begin(), a.end()) - a.begin();
    //cout << minIndex << endl;
    std::vector<double> S;
    S.push_back(start.position.x);//0 1 起点坐标
    S.push_back(start.position.y);
    S.push_back(sox);//2 3 起点圆心坐标
    S.push_back(soy);
    switch (minIndex)//4 5  切线参数
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

    S.push_back(four_line[minIndex][0].x);//6 7  起点圆切点坐标
    S.push_back(four_line[minIndex][0].y);
    S.push_back(eox);//8 9  终点圆心坐标
    S.push_back(eoy);
    S.push_back(four_line[minIndex][1].x);//10 11 终点圆切点坐标
    S.push_back(four_line[minIndex][1].y);
    S.push_back(end.position.x);//12 13  终点坐标
    S.push_back(end.position.y);

    //////////////////////////
    //Robot轨迹点
    //std::vector<Point>Path;

    //Robot从起点到起点圆切点，走圆弧，算走圆弧的步长
    std::vector<double>v0 = { S[0] - S[2],S[1] - S[3] };//起点圆指向起点的向量
    std::vector<double>v1 = { S[6] - S[2],S[7] - S[3] };//起点圆指向切点的向量
    double alpha = angleBetweenPoints(v0[0], v0[1], v1[0], v1[1]);
    //沿着起点圆弧行走时，以每步0.1m，走的步数da
    double L = radius * alpha; //弧长
    double da = alpha / (trunc(L / radStepLen) + 1);
    //从起点圆的圆心到起点的矢量与x轴正方向的夹角，a，a~(-pi,0),表示逆时针，其他表示顺时针
    v1[0] = S[0] - S[2];
    v1[1] = S[1] - S[3];
    v0 = { 1,0 };
    double k = -(v0[0] * v1[1] - v0[1] * v1[0]);
    alpha = sgn(k) * angleBetweenPoints(v0[0], v0[1], v1[0], v1[1]);
    //起点处机器人方向矢量与切线的夹角，该角度方向表示机器人在起点圆上的运动方向，顺时针+，逆时针-
    v1 = { S[10] - S[6], S[11] - S[7] };
    v0[0] = start.direction[0];
    v0[1] = start.direction[1];
    k = -(v0[0] * v1[1] - v0[1] * v1[0]);

    Path.push_back(start.position);//轨迹上的第一个点为起点
    //起点-->起点圆切点
    plotPathCircleStoreInPath(Path, S, alpha, 2, 6, radius, sgn(k),da);
    
    //计算直线运动时的步长，直线运动每步走0.4m 
    L = disPointToPoint(S[10], S[11], S[6], S[7]);
    double dl = L / (trunc(L / lineStepLen) + 1);
    //计算直线的方向
    v1 = { S[10] - S[6], S[11] - S[7] };
    v0 = {1 ,0};
    k = -(v0[0] * v1[1] - v0[1] * v1[0]);
    alpha =  angleBetweenPoints(v0[0], v0[1], v1[0], v1[1]);
    alpha = sgn(k) * alpha;
    //沿直线运动的轨迹
    //起点圆切点-->终点圆切点  ， 走的切线
    plotPathLineStoreInPath(Path, S, alpha, 6, 10, dl);
    
    //与起点圆弧运动相似，运动方向为直线方向与终点处机器人方向矢量之间的方向
    v0 = { S[10] - S[8], S[11] - S[9] };//终点圆圆心指向切点的向量
    v1 = { S[12] - S[8], S[13] - S[9] };//终点圆圆心指向终点的向量
    alpha = angleBetweenPoints(v0[0], v0[1], v1[0], v1[1]);
    //沿着起点圆弧行走时，以每步0.1m，走的步数da
    L = radius * alpha; //弧长
    da = alpha / (trunc(L / radStepLen) + 1);
    
    v1 = { S[10] - S[8],S[11] - S[9] };
    v0 = { 1,0 };
    k = -(v0[0] * v1[1] - v0[1] * v1[0]);
    alpha = sgn(k) * angleBetweenPoints(v0[0], v0[1], v1[0], v1[1]);
    //算k
    v0 = { S[10] - S[6], S[11] - S[7] };
    v1[0] = end.direction[0];
    v1[1] = end.direction[1];
    k = -(v0[0] * v1[1] - v0[1] * v1[0]);
    //终点圆切点-->终点，走的终点圆上的弧线
    plotPathCircleStoreInPath(Path, S, alpha, 8, 12, radius, sgn(k),da);
 
    //plt::show();
    plt::legend();
    plt::save("./output.png");

}

//int main()
//{
//    posVector start = { 0,0,1,1 };
//    posVector end = { 5,5,1,1 };
//    std::vector<Point>Path;
//    //同方向，且起点终点的连线和方向相同
//    std:vector<double>line = { end.position.x - start.position.x,end.position.y - start.position.y };
//    double ang = angleBetweenPoints(start.direction[0], start.direction[1], line[0], line[1]);
//    if (start.direction[0] == end.direction[0] && start.direction[1] == end.direction[1] && ang <= 1e-3)
//    {
//        if (start.position.x == end.position.x && start.position.y==end.position.y)//或者改成相对距离小于一个值（机器人迈一步的最小值，不然会脚踩脚）
//        {
//            cout << "起点和终点在同一位置" << endl;
//        }
//        else
//        {
//            //计算直线运动时的步长，直线运动每步走0.4m 
//            double L = disPointToPoint(start.position.x, start.position.y, end.position.x, end.position.y);
//            double dl = L / (trunc(L / lineStepLen) + 1);
//            //计算直线的方向
//            std::vector<double> v1 = { end.position.x - start.position.x, end.position.y - start.position.y };
//            std::vector<double> v0 = { 1 ,0 };
//            double k = -(v0[0] * v1[1] - v0[1] * v1[0]);
//            double alpha = angleBetweenPoints(v0[0], v0[1], v1[0], v1[1]);
//            alpha = sgn(k) * alpha;
//            //沿直线运动的轨迹
//            //起点圆切点-->终点圆切点  ， 走的切线
//            std::vector<double>S = { start.position.x ,start.position.y, end.position.x, end.position.y };
//            plotPathLineStoreInPath(Path, S, alpha, 0, 2, dl);
//            plt::legend();
//            plt::save("./output.png");
//        }
//    }
//    else if (start.direction[0] == end.direction[0] && start.direction[1] == end.direction[1])
//    {
//        //短距离平移运动 
//    }
//    else
//    {
//        CurvilinearMotionPlanning(start, end, Path);
//    }
//
//}