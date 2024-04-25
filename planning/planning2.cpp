#define _USE_MATH_DEFINES
#include<iostream>
#include<fstream>
#include<cmath>
#include<vector>
#include<matplotlibcpp.h>


#define maxa 10*M_PI/180
#define robot_feet_to_centerL 0.1
#define robot_feet_to_centerR 0.1
#define interpolation_multiple 10.0 //插值倍数
#define deflectionAngleRate 0.25  // 偏转角度的比例系数，在（0,1）之间取值，一般取0.25

using namespace std;
namespace plt = matplotlibcpp;


struct Position {
	double x;
	double y;
};
struct posDirect {
	Position pos;
	Position direct;
	int tag; //1表示直线上的点，-1表示曲线上的点，0表示终点
};
//返回两个向量的内积
double dotVector(Position a, Position b)
{
	return a.x * b.x + a.y * b.y;
}
//返回符号
int sgn(double k)
{
	if (k > 0)
	{
		return 1;
	}
	//else if (k == 0)
	//{
	//	return 0;
	//}
	else
	{
		return -1;
	}
}
//返回符号
int sgnTag(double k)
{
	if (k > 0)
	{
		return 1;
	}
	else if (k == 0)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}
//计算k,返回k的符号正负
int sgnOfk(Position v0, Position v1)
{
	return sgn(-(v0.x * v1.y - v0.y * v1.x));
}

//返回两个向量之间的角度
double angleBetweenVector(Position a, Position b)
{
	double dot = a.x * b.x + a.y * b.y;
	double len1 = sqrt(a.x * a.x + a.y * a.y);
	double len2 = sqrt(b.x * b.x + b.y * b.y);
	return acos(dot / (len1 * len2));
}
//返回两点间的距离
double disBetweenPoints(Position a, Position b)
{
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}
//返回b指向a的向量的x y Position形式
Position vectAtoB(Position a, Position b)
{
	Position res = { b.x - a.x,b.y - a.y };
	return res;
}
Position vectAmulC(Position a, double alpha)
{
	Position res = { alpha * a.x,alpha * a.y };
	return res;
}
Position vectAaddB(Position a, Position b)
{
	Position res = { a.x + b.x,a.y + b.y };
	return res;
}
Position normalizeVect(Position a)
{
	double N = sqrt(a.x * a.x + a.y * a.y);
	Position res = { (1 / N) * a.x,(1 / N) * a.y };
	return res;
}

Position calNextPosition(double x0, double y0, double a, double r)
{
	double x, y;
	double abs_a = abs(a);
	if (a <= 0 && a > -M_PI / 2)
	{
		x = x0 + r * cos(abs_a);
		y = y0 + r * sin(abs_a);
	}
	else if (a <= -M_PI / 2 && a >= -M_PI)
	{
		x = x0 - r * cos(M_PI - abs_a);
		y = y0 + r * sin(M_PI - abs_a);
	}
	else if (a >= 0 && a <= M_PI / 2)
	{
		x = x0 + r * cos(abs_a);
		y = y0 - r * sin(abs_a);
	}
	else
	{
		x = x0 - r * cos(M_PI - abs_a);
		y = y0 - r * sin(M_PI - abs_a);
	}
	Position p1 = { x,y };
	return p1;
}
//计算da
double cal_da(Position v0, Position v1)
{
	double a = sgnOfk(v0, v1) * angleBetweenVector(v0, v1);
	double abs_a = abs(a);
	double da;
	if (deflectionAngleRate * abs_a > maxa)
	{
		da = maxa;
	}
	else
	{
		da = deflectionAngleRate * abs_a;
	}
	return da;
}
Position newRotationAngle(double da,Position v0,Position& v1)
{
	//顺时针旋转da角度
	Position tmpA = { cos(da),-sin(da) };
	Position tmpB = { sin(da),cos(da) };
	Position vetp = { dotVector(tmpA,v1),dotVector(tmpB,v1) };
	tmpA = { cos(-da),-sin(-da) };
	tmpB = { sin(-da),cos(-da) };
	Position vetn = { dotVector(tmpA,v1),dotVector(tmpB,v1) };
	//计算顺时针旋转后与前终点到起点的矢量之间的夹角
	v1 = vetp;
	double ap = angleBetweenVector(v0, v1);
	v1 = vetn;
	double an = angleBetweenVector(v0, v1);
	//向哪个转向运动后的指向与当前终点到起点的矢量之间的夹角减小就用哪个方向旋转，更新新的终点的朝向
	if (ap <= an)
	{
		return vetp;
	}
	{
		return vetn;
	}

}
void dataToFile(std::vector<posDirect>Path,string fname)
{
	std::ofstream outFile(fname, ios::out);
	int peSize = Path.size();
	for (int i = 0; i < peSize; i++)
	{
		//outFile << to_string(round(Path[i].pos.x*10000)/10000) << ','
		//	<< to_string(round(Path[i].pos.y * 10000) / 10000) << ','
		//	<< to_string(round(Path[i].direct.x * 10000) / 10000) << ','
		//	<< to_string(round(Path[i].direct.y * 10000) / 10000) << ','
		//	<< to_string(Path[i].tag) << endl;
		outFile << to_string(Path[i].pos.x ) << ','
			<< to_string(Path[i].pos.y ) << ','
			<< to_string(Path[i].direct.x ) << ','
			<< to_string(Path[i].direct.y ) << ','
			<< to_string(Path[i].tag) << endl;
	}
}
//区分轨迹上的点是曲线还是直线，曲线则tag为-1，直线tag为1，终点为0
//scTH：判断直线或曲线的依据，当前点朝向和下一点朝向变化的阈值，单位：度
void tagCurOrLine(std::vector<posDirect>& Path, double scTH)
{
	int pathSize = Path.size();
	scTH = scTH * M_PI / 180;
	for (int i = 0; i < pathSize - 1; i++)
	{
		double a = angleBetweenVector(Path[i].direct, Path[i + 1].direct);
		if (a < scTH)
		{
			Path[i].tag = 1;	// 直线
		}
		else
		{
			Path[i].tag = -1;	// 曲线
		}
	}
	Path[pathSize - 1].tag = 0;	//终点
}
//质心轨迹规划
void planning(posDirect start, posDirect end, std::vector<posDirect>& Path, double r = 0.1)
{
	double msa = sqrt(dotVector(start.direct, start.direct));	// 起点朝向矢量归一化
	start.direct = { start.direct.x / msa,start.direct.y / msa };
	double mea = sqrt(dotVector(end.direct, end.direct));	//终点朝向矢量归一化
	end.direct = { end.direct.x / mea,end.direct.y / mea };
	//计算ea与x轴正向夹角
	Position v0 = { 1,0 };
	double a = sgnOfk(v0, end.direct) * angleBetweenVector(v0, end.direct);
	double x, y;
	int ks = 0;
	int ke = 0;
	// 将ea旋转180度->v1
	Position v1 = { -end.direct.x,-end.direct.y };//ea的反向矢量
	Position nea = v1;

	double D = disBetweenPoints(start.pos, end.pos);// D：当前起点和终点之间的距离

	vector<posDirect>Pe = { {end.pos,nea} };//Pe：从终点反向出发的路径点坐标和朝向
	Position vet = nea;
	Position epc = end.pos;

	vector<posDirect>Ps = { start };//Ps：从起点沿着朝向出发的路径点坐标和朝向
	Position vst = start.direct;
	Position spc = start.pos;

	double da;
	while (D >= r)
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
	//整理整条路径
	Path = Ps;
	int peSize = Pe.size();
	posDirect nextNode;
	for (int i = peSize - 2; i >= 0; i--)
	{
		nextNode.pos = Pe[i].pos;
		nextNode.direct = { -Pe[i].direct.x,-Pe[i].direct.y };
		nextNode.direct = normalizeVect(nextNode.direct);
		nextNode.tag = Pe[i].tag;
		Path.push_back(nextNode);
	}
	//将数据写入文件  ---为了matlab画图
	string fname = "rawPath.csv";
	dataToFile(Path,fname);
	//区分点是曲线上的还是直线上的
	double scTH = 1;
	tagCurOrLine(Path, scTH);
	string fname2 = "Path.csv";
	dataToFile(Path, fname2);
}
Position oneFootNextPos(Position verVect,Position centerPos, double  r)
{
	Position v0 = { 1,0 };
	Position v1 = verVect;
	double a = sgnOfk(v0, v1) * angleBetweenVector(v0, v1);
	Position nextPos = calNextPosition(centerPos.x, centerPos.y, a, r);
	return nextPos;
}
//产生下一个插值点
posDirect genInterpolationNew(posDirect pd1,posDirect pd2,int j)
{
	double k = (j+1) / interpolation_multiple;
	posDirect res;
	Position tmpPos = vectAmulC(vectAtoB(pd1.pos, pd2.pos), k);
	res.pos = vectAaddB(pd1.pos, tmpPos);
	Position tmpDirect = vectAmulC(vectAtoB(pd1.direct, pd2.direct), k);
	res.direct = normalizeVect(vectAaddB(pd1.direct, tmpDirect));
	res.tag = sgnTag( pd1.tag + k * (pd2.tag - pd1.tag));
	return res;
}
void feetPlanning(std::vector<posDirect>Path, std::vector<posDirect>& leftPath, std::vector<posDirect>& rightPath)
{
	int pathSize = Path.size();
	std::vector <posDirect> LPath, RPath;
	for (int i = 0; i < pathSize; i++)
	{
		Position VerticalDireRight = { Path[i].direct.y,-Path[i].direct.x };//和Path[i]方向垂直的两个方向
		Position VerticalDireleft = { -Path[i].direct.y,Path[i].direct.x };
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
	//将数据写入文件  ---为了matlab画图
	string fnameLeft = "leftRawPath.csv";
	dataToFile(LPath, fnameLeft);
	string fnameRight = "rightRawPath.csv";
	dataToFile(RPath, fnameRight);

	//分别对双足规划的位置和朝向进行插值
	int Ls = 0;
	int Rs = 0;
	for (int i = 0; i < pathSize - 1; i++)
	{

		Ls++;
		leftPath.push_back( LPath[i]);
		rightPath.push_back(RPath[i]);
		for (int j = 0; j < interpolation_multiple - 1; j++)
		{
			//左
			posDirect newOne = genInterpolationNew(LPath[i], LPath[i + 1], j);
			leftPath.push_back(newOne);
			//右
			newOne = genInterpolationNew(RPath[i], RPath[i + 1], j);
			rightPath.push_back(newOne);
		}
	}
	//修改中间数据第5列为0的点的属性。
	pathSize = leftPath.size();
	for (int i = 0; i < pathSize-1; i++)
	{
		if (leftPath[i].tag == 0)
		{
			leftPath[i].tag = leftPath[i + 1].tag;
		}
		if (rightPath[i].tag == 0)
		{
			rightPath[i].tag = rightPath[i + 1].tag;
		}
	}
	//
	string fnameLeft2 = "leftPath.csv";
	dataToFile(leftPath, fnameLeft2);
	string fnameRight2 = "rightPath.csv";
	dataToFile(rightPath, fnameRight2);
}

int main()
{
	posDirect sp_sa = { 0,0,1,0 ,0};
	posDirect ep_ea = { 0,6,0,1 ,0};
	std::vector <posDirect>Path,leftPath,rightPath;
	planning(sp_sa, ep_ea, Path);//规划质心轨迹
	feetPlanning(Path, leftPath, rightPath);//规划双足轨迹线
	return 0;
}