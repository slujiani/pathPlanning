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
    ::Point position;//����
    double direction[2];//����
};

// typedef struct pointCp
// {
//     double sp[2]; //�������
//     double sa[2]; //���λ��ʱ�����˳���ķ���ʸ��
//     double ep[2]; //�յ�����
//     double ea[2]; //�յ�λ��ʱ�����˳���ķ���ʸ��
// }POINTCP;


//����һ��������������Բ
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
//��������ľ���
double disPointToPoint(double a1, double b1, double a2, double b2)
{
    return pow(a1 - a2, 2) + pow(b1 - b2, 2);
}
std::tuple<double, double> findNearCircle(posVector start, posVector end)
{
    // ��������뾶Ϊr��Բ����Բ�뷽��ʸ������
    // ����㷽��ʸ�����а뾶Ϊr��Բ��������Բ��λ�÷ֱ�Ϊ(a1,b1)��(a2,b2)
    double a1, b1, a2, b2;
    std::tie(a1, b1, a2, b2) = tangentCircleCenter(start.position, start.direction);
    // �ֱ��������Բ��Բ�����յ�ľ��룬L1��L2
    double L1 = disPointToPoint(a1, b1, end.position.x, end.position.y);
    double L2 = disPointToPoint(a2, b2, end.position.x, end.position.y);
    //ѡ�����յ������Բ,Բ������Ϊ(sox,soy)
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
        return res; // ����һ��������
    }
    res = { ox,oy };
    return res;
}
//�������������ļн�
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
//����λ�úͷ����յ��λ�úͷ��򣬻�Բ�İ뾶  ���ز���S
void planning(posVector start, posVector end, std::vector<double>& S,double radius = 1.0 )
{
    //������Բ
    double sox, soy, eox, eoy;
    std::tie(sox, soy) = findNearCircle(start, end);
    //���յ��Բ
    std::tie(eox, eoy) = findNearCircle(end, start);

     // ����Բc1: ��x-a1)^2+(y-b1)^2=pow(d1,2)��ԲC2: (x-a2)^2+(y-b2)^2=pow(d2,2)�Ĺ���
     //�ߣ���ֱ��ͬʱ��Բc1��Բc2����
     //����Բ�Ĺ������м��ֲ�ͬ������ֱ��ǣ�
     //һ��Բ����һ��Բ�ڲ�����ʱû�й����� 
     //����Բ�غϣ���ʱ��������ͨ����
     //����Բ�н��㣨һ������������),��ʱ������������
     //����Բ�޽��㣬��ʱ��4��������
     //�ó����������յ��Զ���������ڳ�������������˶�ʱ�Ĺ滮
     //��ʱ����Բ�޽��㣬4�������ߵ�һ�㷽��Ϊ
     //l1: l11*x + l21*y + 1 = 0;
     //l2: l12*x + l22*y + 1 = 0;
     //l3: l13*x + l23*y + 1 = 0;
     //l4: l14*x + l24*y + 1 = 0;
     //�����ý�����������4�����ߵĲ���(l11,l12),(l21,l22),(l31,l32),(l41,l42)
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

    //����ֱ��l1����㴦Բ���е�
    vector<::Point>seo1;//seo1���е����꣬��һ�����ڶ���Ԫ����l1�����Բ�ϵ��е����꣬�������͵��ĸ���l1���յ�Բ�ϵ��е�����
    //����ֱ��l1����㴦Բ���е�
    seo1.push_back(calLineCircleTangencyPoint(l11, l21, sox, soy, radius));
    //����ֱ��l1���յ㴦Բ���е�
    seo1.push_back(calLineCircleTangencyPoint(l11, l21, eox, eoy, radius));

    ////////////////////
    //����ֱ��l2������Բ���е�
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

    // ������Բ���յ��Բ��4�����߻�����
    // ��Բ
    plotCircle(eox, eoy, radius);
    plotCircle(sox, soy, radius);
    //������
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

    //��4�����ߣ������˴����Բ���յ�Բ ����һ��ֱ�ߣ�
    //һ����������㴦�����˷���ʸ��֮��ļн�a1,���յ㴦�����˵ķ���ʸ���ļн�a2,�������Ƕ�֮����С�����߾��ǻ�����Ҫ���ߵ�ֱ�߶�
    std::vector<double>a(4);
    
    for (int i = 0; i < 4; i++)
    {
        double a1 = angleBetweenPoints(four_line[i][0].x, four_line[i][0].y, start.direction[0], start.direction[1]);//�����
        double a2 = angleBetweenPoints(four_line[i][1].x, four_line[i][1].y, end.direction[0], end.direction[1]);//���յ�
        a[i] = a1 + a2;
    }
    auto minIndex = min_element(a.begin(), a.end())-a.begin();
    //cout << minIndex << endl;
    
    S.push_back(start.position.x);//�������
    S.push_back(start.position.y);
    S.push_back(sox);//���Բ������
    S.push_back(soy);
    switch (minIndex)//���߲���
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
    
    S.push_back(four_line[minIndex][0].x);//���Բ�е�����
    S.push_back(four_line[minIndex][0].y);
    S.push_back(eox);//�յ�Բ������
    S.push_back(eoy);
    S.push_back(four_line[minIndex][1].x);//�յ�Բ�е�����
    S.push_back(four_line[minIndex][1].y);
    S.push_back(end.position.x);//�յ�����
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