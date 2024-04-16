#define _USE_MATH_DEFINES
#include<iostream>
#include<tuple>
#include<math.h>
#include<cmath>
#include<vector>
#include<matplotlibcpp.h>

#define radStepLen 0.1 //Բ����Robot�ߵĲ���
#define lineStepLen 0.4 //ֱ��Robot�ߵĲ���
//#define sgn(x) (x>0)-(x<0)

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
    return sqrt(pow(a1 - a2, 2) + pow(b1 - b2, 2));
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
    double cox, coy;//����Բ��
    double L = radius * alpha; //����
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
        plotCircle(cox, coy, da * radius);//ÿ��һ����һ��Բ
        D1 = disPointToPoint(cox, coy, S[endInd], S[endInd+1]);
    } while (D1 > 0.05);
}
void plotPathLineStoreInPath(std::vector<Point>& Path, std::vector<double>S, double alpha, int startInd, int endInd, double dl)
{
    int k = 1;
    double x0 = S[startInd];//���������Բ�������λ�õ������Բ�ϵ��е�λ�úܽ���
    double y0 = S[startInd+1];//���Խ����Բ�ϵ��е���Ϊ��ʱ��ֱ���˶������
    double abs_a = abs(alpha);
    double cox, coy;//����Բ��
    //��ֱ���˶��Ĺ켣
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
        plotCircle(cox, coy, dl);//ÿ��һ����һ��Բ
        D1 = disPointToPoint(cox, coy, S[endInd], S[endInd+1]);// ����켣��ܽӽ��յ�Բ�ϵ��е㣬����Ϊ����ֱ���˶��׶Σ�ת���յ�Բ���˶���)��
    } while (D1 > 0.2);
}
//�����˶�
//����λ�úͷ����յ��λ�úͷ��򣬻�Բ�İ뾶  ���ز���S
void CurvilinearMotionPlanning(posVector start, posVector end, std::vector<Point>& Path, double radius = 1.0)
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
    auto minIndex = min_element(a.begin(), a.end()) - a.begin();
    //cout << minIndex << endl;
    std::vector<double> S;
    S.push_back(start.position.x);//0 1 �������
    S.push_back(start.position.y);
    S.push_back(sox);//2 3 ���Բ������
    S.push_back(soy);
    switch (minIndex)//4 5  ���߲���
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

    S.push_back(four_line[minIndex][0].x);//6 7  ���Բ�е�����
    S.push_back(four_line[minIndex][0].y);
    S.push_back(eox);//8 9  �յ�Բ������
    S.push_back(eoy);
    S.push_back(four_line[minIndex][1].x);//10 11 �յ�Բ�е�����
    S.push_back(four_line[minIndex][1].y);
    S.push_back(end.position.x);//12 13  �յ�����
    S.push_back(end.position.y);

    //////////////////////////
    //Robot�켣��
    //std::vector<Point>Path;

    //Robot����㵽���Բ�е㣬��Բ��������Բ���Ĳ���
    std::vector<double>v0 = { S[0] - S[2],S[1] - S[3] };//���Բָ����������
    std::vector<double>v1 = { S[6] - S[2],S[7] - S[3] };//���Բָ���е������
    double alpha = angleBetweenPoints(v0[0], v0[1], v1[0], v1[1]);
    //�������Բ������ʱ����ÿ��0.1m���ߵĲ���da
    double L = radius * alpha; //����
    double da = alpha / (trunc(L / radStepLen) + 1);
    //�����Բ��Բ�ĵ�����ʸ����x��������ļнǣ�a��a~(-pi,0),��ʾ��ʱ�룬������ʾ˳ʱ��
    v1[0] = S[0] - S[2];
    v1[1] = S[1] - S[3];
    v0 = { 1,0 };
    double k = -(v0[0] * v1[1] - v0[1] * v1[0]);
    alpha = sgn(k) * angleBetweenPoints(v0[0], v0[1], v1[0], v1[1]);
    //��㴦�����˷���ʸ�������ߵļнǣ��ýǶȷ����ʾ�����������Բ�ϵ��˶�����˳ʱ��+����ʱ��-
    v1 = { S[10] - S[6], S[11] - S[7] };
    v0[0] = start.direction[0];
    v0[1] = start.direction[1];
    k = -(v0[0] * v1[1] - v0[1] * v1[0]);

    Path.push_back(start.position);//�켣�ϵĵ�һ����Ϊ���
    //���-->���Բ�е�
    plotPathCircleStoreInPath(Path, S, alpha, 2, 6, radius, sgn(k),da);
    
    //����ֱ���˶�ʱ�Ĳ�����ֱ���˶�ÿ����0.4m 
    L = disPointToPoint(S[10], S[11], S[6], S[7]);
    double dl = L / (trunc(L / lineStepLen) + 1);
    //����ֱ�ߵķ���
    v1 = { S[10] - S[6], S[11] - S[7] };
    v0 = {1 ,0};
    k = -(v0[0] * v1[1] - v0[1] * v1[0]);
    alpha =  angleBetweenPoints(v0[0], v0[1], v1[0], v1[1]);
    alpha = sgn(k) * alpha;
    //��ֱ���˶��Ĺ켣
    //���Բ�е�-->�յ�Բ�е�  �� �ߵ�����
    plotPathLineStoreInPath(Path, S, alpha, 6, 10, dl);
    
    //�����Բ���˶����ƣ��˶�����Ϊֱ�߷������յ㴦�����˷���ʸ��֮��ķ���
    v0 = { S[10] - S[8], S[11] - S[9] };//�յ�ԲԲ��ָ���е������
    v1 = { S[12] - S[8], S[13] - S[9] };//�յ�ԲԲ��ָ���յ������
    alpha = angleBetweenPoints(v0[0], v0[1], v1[0], v1[1]);
    //�������Բ������ʱ����ÿ��0.1m���ߵĲ���da
    L = radius * alpha; //����
    da = alpha / (trunc(L / radStepLen) + 1);
    
    v1 = { S[10] - S[8],S[11] - S[9] };
    v0 = { 1,0 };
    k = -(v0[0] * v1[1] - v0[1] * v1[0]);
    alpha = sgn(k) * angleBetweenPoints(v0[0], v0[1], v1[0], v1[1]);
    //��k
    v0 = { S[10] - S[6], S[11] - S[7] };
    v1[0] = end.direction[0];
    v1[1] = end.direction[1];
    k = -(v0[0] * v1[1] - v0[1] * v1[0]);
    //�յ�Բ�е�-->�յ㣬�ߵ��յ�Բ�ϵĻ���
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
//    //ͬ����������յ�����ߺͷ�����ͬ
//    std:vector<double>line = { end.position.x - start.position.x,end.position.y - start.position.y };
//    double ang = angleBetweenPoints(start.direction[0], start.direction[1], line[0], line[1]);
//    if (start.direction[0] == end.direction[0] && start.direction[1] == end.direction[1] && ang <= 1e-3)
//    {
//        if (start.position.x == end.position.x && start.position.y==end.position.y)//���߸ĳ���Ծ���С��һ��ֵ����������һ������Сֵ����Ȼ��ŲȽţ�
//        {
//            cout << "�����յ���ͬһλ��" << endl;
//        }
//        else
//        {
//            //����ֱ���˶�ʱ�Ĳ�����ֱ���˶�ÿ����0.4m 
//            double L = disPointToPoint(start.position.x, start.position.y, end.position.x, end.position.y);
//            double dl = L / (trunc(L / lineStepLen) + 1);
//            //����ֱ�ߵķ���
//            std::vector<double> v1 = { end.position.x - start.position.x, end.position.y - start.position.y };
//            std::vector<double> v0 = { 1 ,0 };
//            double k = -(v0[0] * v1[1] - v0[1] * v1[0]);
//            double alpha = angleBetweenPoints(v0[0], v0[1], v1[0], v1[1]);
//            alpha = sgn(k) * alpha;
//            //��ֱ���˶��Ĺ켣
//            //���Բ�е�-->�յ�Բ�е�  �� �ߵ�����
//            std::vector<double>S = { start.position.x ,start.position.y, end.position.x, end.position.y };
//            plotPathLineStoreInPath(Path, S, alpha, 0, 2, dl);
//            plt::legend();
//            plt::save("./output.png");
//        }
//    }
//    else if (start.direction[0] == end.direction[0] && start.direction[1] == end.direction[1])
//    {
//        //�̾���ƽ���˶� 
//    }
//    else
//    {
//        CurvilinearMotionPlanning(start, end, Path);
//    }
//
//}