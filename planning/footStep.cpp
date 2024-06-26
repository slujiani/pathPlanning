#define _USE_MATH_DEFINES
#include<iostream>
#include<fstream>
#include<sstream>
#include<vector>
#include<cmath>
#include<random>
#include<tuple>
#include<float.h>
#include<algorithm>
//#include "footstep_planning/Ethercat.h"
//#include "footstep_planning/Footstep.h"
//#include <ros/ros.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Bool.h>
//#include <std_msgs/Bool.h>
#include <condition_variable>
//#include <unistd.h>
#include <thread>
#include "include/base.h"
#include "include/footDEMPlanning.h"



std::vector<double> split(std::string str, char del);
void fileToData(std::vector<posDirect3D>& data, std::string filename);
std::tuple<double, double, double> getRoadLen(std::vector<posDirect3D>path);
int searchNearestPos(int low, int high, xyz cur, std::vector<posDirect3D>path);
std::tuple<posDirect3D, int> getOneFoot(std::vector<posDirect3D>path, int ind, posDirect3D cur, double v, int state_change);
std::tuple<posDirect3D, int> getPosFit(std::vector<posDirect3D>path_behind, posDirect3D cur_b, int ind_b, double v_b);
std::tuple<int, posDirect3D, int, int> getNextLocat(std::vector<posDirect3D>centerPath, std::vector<posDirect3D>left, std::vector<posDirect3D>right, posDirect3D Lcur, posDirect3D Rcur, double vk, int pll, int SC);

int get_R_Q2(posDirect3D cur, double z, xyz n, std::vector<std::vector<double>>& R, std::vector<double>& Q);
double generateNormalRandom();
xy addGaussinToSim(xy goalPos);
//void dataToFile(std::vector<posDirect3D>Path, std::string fname);
//void outToEthercat(std::vector<float> outp);
std::tuple<posDirect3D, posDirect3D> getCurFeet();
int run();
//void ethercat_callback(const footstep_planning::Ethercat::ConstPtr& msg);
//footstep_planning::Ethercat ethercatMsg;
//footstep_planning::Footstep footStepMsg;
//ros::Subscriber sub_ethercat;
//ros::Subscriber sub_walkingflag;
//ros::Publisher pub_footstep;
bool walkingFlag;


// str 为输入字符串，del为分隔符，返回分割之后的字符串数组
std::vector<double> split(std::string str, char del) {
	std::stringstream ss(str);
	std::string tmp;
	std::vector<double> res;
	while (getline(ss, tmp, del)) {
		res.push_back(stod(tmp));
	}
	return res;
}
void fileToData(std::vector<posDirect3D>& data, std::string filename)
{
	std::ifstream csvData(filename, std::ios::in);
	if (!csvData.is_open())
	{
		std::cout << "Error: opening file fail" << std::endl;
		exit(1);
	}
	else
	{
		std::string line, word;
		std::stringstream sin;
		posDirect3D newNode;
		while (getline(csvData, line))
		{
			std::vector<double>dataVec = split(line, ',');

			newNode.pos.x = dataVec[0];
			newNode.pos.y = dataVec[1];
			newNode.pos.z = dataVec[2];

			newNode.direct.x = dataVec[3];
			newNode.direct.y = dataVec[4];

			newNode.tag = dataVec[5];

			data.push_back(newNode);
		}
		csvData.close();
	}
}
//lcTh：直线路径和曲线路径转角区分阈值，单位：度
std::tuple<double, double, double> getRoadLen(std::vector<posDirect3D>path)
{
	int pathSize = path.size();
	double L = 0;	// 路径长度
	double Ll = 0;	// 直线长度
	double Lc = 0;	// 曲线长度
	for (int i = 0; i < pathSize - 1; i++)
	{
		xy diff = vecAsubB(path[i + 1].pos, path[i].pos);
		double D = sqrt(dotOfVec(diff, diff));
		L += D;
		if (path[i].tag == -1)
		{
			Lc += D;
		}
		else
		{
			Ll += D;
		}
	}
	return std::make_tuple(L, Ll, Lc);
}
//搜索路径中离当前点最近的点，将当前点归到那一组去
int searchNearestPos(int low, int high, xyz cur, std::vector<posDirect3D>path)
{
	double minp = DBL_MAX;
	int ret = -1;
	for (int i = low; i < high; i++)
	{
		xy diff = vecAsubB(path[i].pos, cur);
		double D = dotOfVec(diff, diff);
		if (D <= minp)
		{
			minp = D;
			ret = i;
		}
	}

	return ret;
}
std::tuple<posDirect3D, int> getOneFoot(std::vector<posDirect3D>path, int ind, posDirect3D cur, double v, int state_change)
{
	int pathSize = path.size();
	int lowInd = min(ind + 1, pathSize - 1);
	int i, e = pathSize - 1;
	posDirect3D ret;
	for (i = lowInd; i < pathSize; i++)
	{
		xy diff = vecAsubB(path[i].pos, cur.pos);
		double D = dotOfVec(diff, diff);
		if (D >= pow(v, 2))
		{
			ret = path[i];
			e = i;
			break;
		}
	}
	//判断在这个过程中是否发生状态变化，如果变化，state_change== 1
	for (i = ind; i < e; i++)
	{
		if (path[i].tag != path[i + 1].tag)
		{
			state_change = 1;
			break;
		}
	}
	//如果落后足达到终点，则下一步规划还是终点位置
	if (i == pathSize - 1)
	{
		ret = path[i];
	}
	return std::make_tuple(ret, state_change);
}
//path是落后足的路径，返回落后足及前足的点位,state_change返回值得与之前的或一下
std::tuple<posDirect3D, int> getPosFit(std::vector<posDirect3D>path_behind, posDirect3D cur_b, int ind_b, double v_b)
{
	posDirect3D pD_b;
	int state_change = 0;
	//落后足从当前位置开始向前搜索，直到运动长度达到指定步长
	int pathSize = path_behind.size();

	std::tie(pD_b, state_change) = getOneFoot(path_behind, ind_b, cur_b, v_b, state_change);
	//std::tie(pD_f, state_change) = getOneFoot(path_forward, ind_f, cur_f, v_f, state_change);只规划落后一步
	return std::make_tuple(pD_b, state_change);
}
//vk=LR/LL LR\LL左右足完整轨迹中曲线段的长度
//返回值
//FR 是否是终点
//下一步的pos direct
//state_change
std::tuple<int, posDirect3D, int, int> getNextLocat(std::vector<posDirect3D>centerPath,std::vector<posDirect3D>left, std::vector<posDirect3D>right, posDirect3D Lcur, posDirect3D Rcur, double vk, int pll, int SC)
{
	int FR = 0;	//先走哪一足，1 左足，-1 右足，0 不动
	posDirect3D nextLeftF = Lcur, nextRightF = Rcur;
	double lt = 0;	//左足运动时间
	double rt = 0;
	int stateChange = 0;	// state_change==1 在机器人按规划路径行走会发生状态变化，state_change==0，无状态
	int vw = 1;	//右足根据vk该比例调节步长（速度）
	//找到左足当前位置离规划的位置序列中哪个位置最近
	int pathSize = left.size();

	int lind = searchNearestPos(0, pathSize, Lcur.pos, left);
	if (lind == -1)
	{
		std::cout << "左腿未找到静态路径中的邻近点" << std::endl;
	}
	int disTmp = dis2D(Lcur.pos, left[lind].pos);
	if (disTmp > robot_foot_length/perPixelWidth)
	{
		//最近点离静态路径中的点都远-->重规划
		int replan_ind = min(lind + 5, pathSize - 1);
		cv::Point start(Lcur.pos.x / perPixelWidth, Lcur.pos.y / perPixelWidth);
		cv::Point goal(centerPath[replan_ind].pos.x / perPixelWidth ,centerPath[replan_ind].pos.y / perPixelWidth);

		// 重新规划路径
		std::string fnCenter = "replanCenter.csv";
		std::string fnLeft = "replanLeft.csv";
		std::string fnRight = "replanRight.csv";
		genernateFeetPos(start, goal, fnCenter, fnLeft, fnRight);
		//静态路径拼接 将replan的路径插到原来的路径中，覆盖掉原来的lind到Lind+5这段

		std::string fname1 = "file/" + fnLeft;
		std::string fname2 = "file/" + fnRight;
		std::string fname3 = "file/" + fnCenter;
		std::vector<posDirect3D>newLeftPath;
		std::vector<posDirect3D>newRightPath;
		std::vector<posDirect3D>newCenterPath;
		fileToData(newLeftPath, fname1);
		fileToData(newRightPath, fname2);
		fileToData(newCenterPath, fname3);
		
		// 拼接路径：将新路径插入到原路径中，覆盖掉原来的 lind 到 replan_ind 这段
		std::vector<posDirect3D> updatedLeftPath(left.begin(), left.begin() + lind);
		updatedLeftPath.insert(updatedLeftPath.end(), newLeftPath.begin(), newLeftPath.end());
		updatedLeftPath.insert(updatedLeftPath.end(), left.begin() + replan_ind, left.end());

		std::vector<posDirect3D> updatedRightPath(right.begin(), right.begin() + lind);
		updatedRightPath.insert(updatedRightPath.end(), newRightPath.begin(), newRightPath.end());
		updatedRightPath.insert(updatedRightPath.end(), right.begin() + replan_ind, right.end());

		std::vector<posDirect3D> updatedCenterPath(centerPath.begin(), centerPath.begin() + lind);
		updatedCenterPath.insert(updatedCenterPath.end(), newCenterPath.begin(), newCenterPath.end());
		updatedCenterPath.insert(updatedCenterPath.end(), centerPath.begin() + replan_ind, centerPath.end());

		return getNextLocat(updatedCenterPath,updatedLeftPath, updatedRightPath, Lcur, Rcur, vk, pll, SC);
	}
	//在左足最近位置附近，在右足序列中找到与右足位置最近的点
	int lowInd = max((int)(lind - 0.1 * pathSize), 0);
	int highInd = min((int)(lind + 0.1 * pathSize), pathSize);
	if ( highInd < pathSize-1)
	{
		highInd += 2;
	}
	int rind = searchNearestPos(lowInd, highInd, Rcur.pos, right);

	double lv, rv;//左右脚速度
	int i;
	int e = -1;
	//两足并列
	if ( ((abs(lind - rind) <= 3 ) && left[lind].tag!=3) || pll == 1)
	{
		if (lind == pathSize - 1)//已到达终点，结束
		{
			FR = 0;
			lt = 0;
			std::cout << "两足并列、而且已经到达终点，规划结束。" << std::endl;
			return std::make_tuple(FR, nextLeftF, lt, stateChange);
		}
		//根据规划中下一点的状态，直线还是曲线，决定双足速度
		if (left[min(lind + 1, pathSize - 1)].tag == -1 || right[min(rind + 1, pathSize - 1)].tag == -1)
		{
			// 曲线运动
			lv = 0.5 * vc;// 左足权限运动，双足并列后，左足先动，运动半步
			rv = vk * vc;// 随后右足运动，右足速度（步长）根据曲线段长度的比例调整
			std::cout << "下一步是曲线运动" << std::endl;
		}
		else
		{
			// 直线运动
			lv = 0.5 * vs; // 左足半步
			rv = vw * vs; // 右足正常
			std::cout << "下一步是直线运动" << std::endl;
		}
		//左足从当前位置开始，向前搜索，直到运动长度达到指定步长
		lowInd = min(lind + 1, pathSize - 1);

		for (i = lowInd; i < pathSize; i++)
		{
			xy diff = vecAsubB(left[i].pos, Lcur.pos);
			double D = dotOfVec(diff, diff);
			if (D >= lv * lv)
			{
				nextLeftF = left[i];
				lt = 0; //先运动，为0
				FR = 1;
				e = i;
				break;
			}
		}
		//判断在这个过程中是否发生状态变化，如果变化，state_change== 1

		for (i = lind; i < e; i++)
		{
			if (left[i].tag != left[i + 1].tag)
			{
				stateChange = 1;
				break;
			}
		}
		//如果左足达到终点，则下一步规划还是终点位置
		if (i == pathSize - 1)
		{
			nextLeftF = left[i];
			lt = 0;
			FR = 1;
		}
		std::cout << "运动左脚,左脚坐标：" << nextLeftF.pos.x << "," << nextLeftF.pos.y <<","<<nextLeftF.pos.z << std::endl;
		std::cout << "左脚方向：" << nextLeftF.direct.x << "," << nextLeftF.direct.y << std::endl;
		return std::make_tuple(FR, nextLeftF, lt, stateChange);
	}
	//一足超前，一足落后，先运动落后足
	if (lind > rind)//左足超前,右足落后
	{
		rt = 0;
		FR = -1;
		std::cout << "左脚在前，右脚在后" << std::endl;
		//如果在上次运动中发生的状态变化，则强制落后足一步并列，另一足不动
		if (SC == 1 ||((left[lind].tag!=3) && left[lind].tag != left[min(lind + 1, pathSize - 1)].tag))//左脚不能是一个segment的开头，只有在直线和曲线切换时，左右脚对齐
		{
			nextRightF = right[lind];
			lt = 1;//左足不动
			std::cout << "曲直变化，右脚上前和左脚齐平。" << std::endl;
			std::cout << "运动右脚,右脚坐标：" << nextRightF.pos.x << "," << nextRightF.pos.y << "," << nextRightF.pos.z << std::endl;
			std::cout << "右脚方向：" << nextRightF.direct.x << "," << nextRightF.direct.y << std::endl;
			return std::make_tuple(FR, nextRightF, rt, stateChange);
		}
		//判断右足系一步的状态是直线还是曲线，决定左右足步长
		if (right[rind + 1].tag == -1)
		{
			lv = vc;
			rv = vk * vc;
			std::cout << "下一步是曲线运动" << std::endl;
		}
		else
		{
			lv = vs;
			rv = vw * vs;
			std::cout << "下一步是直线运动" << std::endl;
		}
		std::tie(nextRightF, stateChange) = getPosFit(right, Rcur, rind+1, rv);
		std::cout << "运动右脚,右脚坐标：" << nextRightF.pos.x << "," << nextRightF.pos.y << "," << nextRightF.pos.z<< std::endl;
		std::cout << "右脚方向：" << nextRightF.direct.x << "," << nextRightF.direct.y << std::endl;
		//判断在这个过程中是否发生状态变化，如果变化，state_change == 1
		return std::make_tuple(FR, nextRightF, rt, stateChange);
	}
	else
	{
		lt = 0;
		FR = 1;
		std::cout << "右脚在前，左脚在后" << std::endl;
		//如果在上次运动中发生的状态变化，则强制落后足一步并列，另一足不动
		if (SC == 1 ||(( right[rind].tag != 3) && right[lind].tag != right[min(lind + 1, pathSize - 1)].tag))
		{
			nextLeftF = left[rind];
			rt = 1;//左足不动
			std::cout << "曲直变化，左脚上前和右脚齐平。" << std::endl;
			std::cout << "运动左脚,左脚坐标：" << nextLeftF.pos.x << "," << nextLeftF.pos.y << "," << nextLeftF.pos.z << std::endl;
			std::cout << "左脚方向：" << nextLeftF.direct.x << "," << nextLeftF.direct.y << std::endl;
			return std::make_tuple(FR, nextRightF, rt, stateChange);
		}
		//判断右足系一步的状态是直线还是曲线，决定左右足步长
		if (left[lind + 1].tag == -1)
		{
			lv = vc;
			rv = vk * vc;
			std::cout << "下一步是曲线运动" << std::endl;
		}
		else
		{
			lv = vs;
			rv = vw * vs;
			std::cout << "下一步是直线运动" << std::endl;
		}
		std::tie(nextLeftF, stateChange) = getPosFit(left, Lcur, lind+1, lv);
		std::cout << "运动左脚,左脚坐标：" << nextLeftF.pos.x << "," << nextLeftF.pos.y << "," << nextLeftF.pos.z << std::endl;
		std::cout << "左脚方向：" << nextLeftF.direct.x << "," << nextLeftF.direct.y << std::endl;
		return std::make_tuple(FR, nextLeftF, lt, stateChange);
	}

}


//计算落足点在地图坐标系中的位置和姿态
// 输入
// cur 当前落足点的坐标 x y ;方向 x y
//z：DEM地图上对应(x,y)位置的高程
//n DEM地图上(x,y,z)点的表面法线方向矢量，行向量,也是落足点法向量
//输出：
//flag：标志，1，正常，其他，异常
//vx，vy，vz：分别是足坐标系的x，y，z轴在DEM坐标中的单位向量
//R：足坐标系，x—前进方向，z—足地面发现方向，向上，y—与x，z形成右手坐标系
int get_R_Q2(posDirect3D cur, double z, xyz n, std::vector<std::vector<double>>& R, std::vector<double>& Q)
{
	int flag = 1;
	double N = 1 / (sqrt(dotOfxyz(n, n)));
	n = xyzMulC(n, N);

	//计算落足点法向量与DEM坐标系Z轴夹角，如果角度大于90度，返回异常，-1；
	xyz v0 = { 0,0,1 };
	xyz v1 = n;
	xyz vx, vy, vz;
	double a0 = acos(dotOfxyz(v0, v1));
	if (abs(a0) >= M_PI / 2)
	{
		flag = -1;
		return flag;
	}

	//
	double v0_3 = -(1 / n.z) * (n.x * cur.direct.x + n.y * cur.direct.y);
	//足坐标系x轴在DEM坐标中的矢量v
	xyz cur_dire_xyz = { cur.direct.x,cur.direct.y,v0_3 };
	cur_dire_xyz = normal_xyz(cur_dire_xyz);
	// 足坐标系的y轴与足坐标系x轴，z轴构成右手系
	vz = n;
	vx = cur_dire_xyz;

	vy.x = vz.y * vx.z - vz.z * vx.y;
	vy.y = vz.z * vx.x - vz.x * vx.z;
	vy.z = vz.x * vx.y - vz.y * vx.x;
	vy = normal_xyz(vy);
	//落足点坐标系在DEM坐标系中的方向（姿态）矩阵R
	R = { 
		{vx.x,vy.x,vz.x},
		{vx.y,vy.y,vz.y},
		{vx.z,vy.z,vz.z}};
	//将方向矩阵转换成四元数表示
	//q0为四元数幅度，标量
	Q.push_back(0.5 * sqrt(1 + vx.x + vy.y + vz.z));
	Q.push_back(0.25 * (vy.z - vz.y) / Q[0]);
	Q.push_back(0.25 * (vz.x - vx.z) / Q[0]);
	Q.push_back(0.25 * (vx.y - vy.x) / Q[0]);
	return flag;
}
// 生成一个符合标准正态分布的随机数
double generateNormalRandom()
{
	// 使用标准正态分布，均值为0，方差为1
	std::normal_distribution<double> distribution(0.0, 1);
	// 创建一个随机数生成器
	std::random_device randomDevice;
	std::mt19937 generator(randomDevice());
	// 生成随机数
	return distribution(generator);
}
xy addGaussinToSim(xy goalPos)
{
	xy res;
	double ran = generateNormalRandom();
	res.x = goalPos.x + 0.005 * generateNormalRandom();
	res.y = goalPos.y + 0.005 * generateNormalRandom();
	return res;
}
//void dataToFile(std::vector<posDirect3D>Path, std::string fname)
//{
//	std::ofstream outFile(fname, std::ios::out);
//	int peSize = Path.size();
//	for (int i = 0; i < peSize; i++)
//	{
//		outFile << std::to_string(Path[i].pos.x) << ','
//			<< std::to_string(Path[i].pos.y) << ','
//			<< std::to_string(Path[i].direct.x) << ','
//			<< std::to_string(Path[i].direct.y) << ','
//			<< std::to_string(Path[i].tag) << std::endl;
//	}
//	outFile.close();
//}
//void outToEthercat(std::vector<float> outp)
//{
//	footStepMsg.nextFirstStepX = outp[0];
//	footStepMsg.nextFirstStepY = outp[1];
//	footStepMsg.nextFirstStepZ = outp[2];
//	footStepMsg.nextFirstStepOrientW = outp[3];
//	footStepMsg.nextFirstStepOrientX = outp[4];
//	footStepMsg.nextFirstStepOrientY = outp[5];
//	footStepMsg.nextFirstStepOrientZ = outp[6];
//	footStepMsg.nextFirstStepTime = outp[7];
//
//	footStepMsg.nextSecondStepX = outp[8];
//	footStepMsg.nextSecondStepY = outp[9];
//	footStepMsg.nextSecondStepZ = outp[10];
//	footStepMsg.nextSecondStepOrientW = outp[11];
//	footStepMsg.nextSecondStepOrientX = outp[12];
//	footStepMsg.nextSecondStepOrientY = outp[13];
//	footStepMsg.nextSecondStepOrientZ = outp[14];
//	footStepMsg.nextSecondStepTime = outp[15];
//
//	footStepMsg.nextThirdStepX = outp[16];
//	footStepMsg.nextThirdStepY = outp[17];
//	footStepMsg.nextThirdStepZ = outp[18];
//	footStepMsg.nextThirdStepOrientW = outp[19];
//	footStepMsg.nextThirdStepOrientX = outp[20];
//	footStepMsg.nextThirdStepOrientY = outp[21];
//	footStepMsg.nextThirdStepOrientZ = outp[22];
//	footStepMsg.nextThirdStepTime = outp[23];
//
//	footStepMsg.nextFourthStepX = outp[24];
//	footStepMsg.nextFourthStepY = outp[25];
//	footStepMsg.nextFourthStepZ = outp[26];
//	footStepMsg.nextFourthStepOrientW = outp[27];
//	footStepMsg.nextFourthStepOrientX = outp[28];
//	footStepMsg.nextFourthStepOrientY = outp[29];
//	footStepMsg.nextFourthStepOrientZ = outp[30];
//	footStepMsg.nextFourthStepTime = outp[31];
//
//	footStepMsg.nextFifthStepX = outp[32];
//	footStepMsg.nextFifthStepY = outp[33];
//	footStepMsg.nextFifthStepZ = outp[34];
//	footStepMsg.nextFifthStepOrientW = outp[35];
//	footStepMsg.nextFifthStepOrientX = outp[36];
//	footStepMsg.nextFifthStepOrientY = outp[37];
//	footStepMsg.nextFifthStepOrientZ = outp[38];
//	footStepMsg.nextFifthStepTime = outp[39];
//	pub_footstep.publish(footStepMsg);
//}
std::tuple<posDirect3D, posDirect3D> getCurFeet()
{
	posDirect3D left, right;
	left.pos.x = 12.920711;
	left.pos.y = 11.179289;
	left.pos.z = 0.148235;
	right.pos.x = 12.779289;
	right.pos.y = 11.320710;
	right.pos.z = 0.148235;
	//float q0 = ethercatMsg.LeftFootOrientW;
	//float q1 = ethercatMsg.LeftFootOrientX;
	//float q2 = ethercatMsg.LeftFootOrienty;
	//float q3 = ethercatMsg.LeftFootOrientZ;
	//left.direct = { 1 - 2 * q2 * q2 - 2 * q3 * q3,2 * q1 * q2 + 2 * q0 * q3 };
	left.direct = { -1.000000,-1.000000 };
	//q0 = ethercatMsg.RightFootOrientW;
	//q1 = ethercatMsg.RightFootOrientX;
	//q2 = ethercatMsg.RightFootOrienty;
	//q3 = ethercatMsg.RightFootOrientZ;
	right.direct = { -1.000000,-1.000000 };
	return std::make_tuple(left, right);
}
void show_output(std::vector<float> nextfiveVec)
{
	std::string imagePath = "img/dem.jpg";  
	cv::Mat NextFiveStepsMap = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
	if (NextFiveStepsMap.empty()) {
		std::cout << "Error: Could not open or find the image!" << std::endl;
	}
	else
	{
		cv::Mat visImage;
		cv::cvtColor(NextFiveStepsMap, visImage, cv::COLOR_GRAY2BGR);
		for (int i = 0; i < nextfiveVec.size(); i=i+8)
		{
			// Draw center point
			cv::Point center(nextfiveVec[i] / perPixelWidth, nextfiveVec[i+1] / perPixelWidth);
			cv::circle(visImage, center, 2, cv::Scalar(84, 255, 159), -1);
			// Calculate angle from the direction vector
			/*double theta_x = 1 - 2 * nextfiveVec[i + 5] * nextfiveVec[i + 5] - 2 * nextfiveVec[i + 6] * nextfiveVec[i + 6];
			double theta_y = 2 * nextfiveVec[i + 4] * nextfiveVec[i + 5] + 2 * nextfiveVec[i + 3] * nextfiveVec[i + 6];
			double angle = atan2(theta_y,theta_x) * 180 / M_PI;*/
			double angle = atan2(nextfiveVec[i+4], nextfiveVec[i + 3]) * 180 / M_PI;
			// Draw rectangle
			cv::Point2f rectPoints[4];
			cv::RotatedRect rect(center, cv::Size2f(robot_foot_length / perPixelWidth, robot_foot_width / perPixelWidth), angle);
			rect.points(rectPoints);
			for (int j = 0; j < 4; ++j) {
				cv::line(visImage, rectPoints[j], rectPoints[(j + 1) % 4], cv::Scalar(255, 0, 0), 1);
			}
		}
		// Save the image and check if it was successful
		if (!cv::imwrite("img/NextFiveSteps.jpg", visImage)) {
			std::cout << "Error: Could not save the image!" << std::endl;
		}
		else {
			std::cout << "Image saved successfully!" << std::endl;
		}
		cv::namedWindow("Path Visualization", cv::WINDOW_NORMAL);
		cv::setMouseCallback("Path Visualization", mouseCallback, &visImage);
		cv::imshow("Path Visualization", visImage);
		cv::waitKey(0);
	}
}
int run(std::string fnameCenterPath, std::string fnameleftPath, std::string fnamerightPath) {
	std::vector<posDirect3D> leftPath, rightPath,centerPath;
	std::string fname1 = "file/" + fnameleftPath;
	std::string fname2 = "file/" + fnamerightPath;
	std::string fname3 = "file/" + fnameCenterPath;
	fileToData(leftPath, fname1);
	fileToData(rightPath, fname2);
	fileToData(centerPath, fname3);
	double lenLeft, lenLline, lenLcurve;
	double lenRight, lenRline, lenRcurve;
	std::tie(lenLeft, lenLline, lenLcurve) = getRoadLen(leftPath);
	std::tie(lenRight, lenRline, lenRcurve) = getRoadLen(rightPath);
	int FR = -1;	//到达终点的标志 =0 when reach the final
	int SC = 0;	// 在行走中检测的状态变化，从曲线段到直线段变化，或相反
	int last_SC = 0;
	int pll = 1;	// 在行走过程中是否出现双足并列的情况
	double vk = lenRcurve / lenLcurve;

	posDirect3D nextStep;
	int stateChange, nextStepTime;
	posDirect3D leftcur;
	posDirect3D rightcur;
	std::tie(leftcur, rightcur) = getCurFeet();
	//std::vector<posDirect3D>recordcal;
	//std::vector<posDirect3D>recordreal;
	//真实值和模拟值起点都一样
	//recordcal.push_back(leftcur);
	//recordcal.push_back(rightcur);
	//recordreal.push_back(leftcur);
	//recordreal.push_back(rightcur);
	//输出列表
	std::vector<xyz>lx, ly, lz;
	std::vector<float>output;
	int step = 0;
	while (step < 5)
	{
		//根据当前落足点位置和规划的全部路径和朝向得到下一步双足的落足点位置和方向
		std::tie(FR, nextStep, nextStepTime, stateChange) = getNextLocat(centerPath,leftPath, rightPath, leftcur, rightcur, vk, pll, SC);
		//计算得到的下一个点  发送给机器人
		//左脚
		nextStep.tag = FR; //1 左足， - 1 右足，0 不动
		//recordcal.push_back(nextStep);
		step++;

		if (FR == 1)
		{
			//leftcur.pos = addGaussinToSim(nextStep.pos);
			leftcur.pos = nextStep.pos;
			leftcur.direct = nextStep.direct;
			leftcur.tag = nextStep.tag;
		}
		else if (FR == -1)
		{
			//rightcur.pos = addGaussinToSim(nextStep.pos);
			rightcur.pos = nextStep.pos;
			rightcur.direct = nextStep.direct;
			rightcur.tag = nextStep.tag;
		}
		else
		{
			//arrive at final point


		}
		//加高斯偏置模拟的实际值
		//recordreal.push_back(nextStep);


		SC = stateChange;
		if (last_SC == 1 && stateChange == 0)
		{
			pll = 1;
		}
		else
		{
			pll = 0;
		}
		last_SC = stateChange;
		//按照要求的格式输出，双足的落足点在DEM地图坐标系中的位置和足坐标系在DEM地图坐标系中的姿态
		int flag;
		xyz n = { 0,0,1 };
		std::vector<std::vector<double>> R;
		std::vector<double>Q;
		flag = get_R_Q2(nextStep, 0, n, R, Q);
		if (flag != 1)
		{
			std::cout << "出错啦" << std::endl;
		}
		// output
		output.push_back(nextStep.pos.x);
		output.push_back(nextStep.pos.y);
		output.push_back(nextStep.pos.z);
		/*output.push_back(Q[0]);
		output.push_back(Q[1]);
		output.push_back(Q[2]);
		output.push_back(Q[3]);*/
		output.push_back(nextStep.direct.x);
		output.push_back(nextStep.direct.y);
		output.push_back(Q[2]);
		output.push_back(Q[3]);
		output.push_back(nextStepTime);
		if (FR == 0)
		{
			//arrive at final point
			for (step; step < 5; step++)
			{
				output.push_back(nextStep.pos.x);
				output.push_back(nextStep.pos.y);
				output.push_back(nextStep.pos.z);
				output.push_back(nextStep.direct.x);
				output.push_back(nextStep.direct.y);
				output.push_back(Q[2]);
				output.push_back(Q[3]);
				output.push_back(nextStepTime);
			}
			show_output(output);
			return FR;
		}
	}
	//std::string filesim = "file/file_sim.csv";
	//std::string filereal = "file/file_real.csv";
	//dataToFile(recordcal, filesim);
	//dataToFile(recordreal, filereal);
	//outToEthercat(output);
	//仅调试用
	show_output(output);
	return FR;
}

//void ethercat_callback(const footstep_planning::Ethercat::ConstPtr& msg) {
//	ethercatMsg = *msg;
//}
//
//void walkingflag_callback(const std_msgs::Bool::ConstPtr& msg) {
//	walkingFlag = msg->data;
//}

void run_() {
	//ethercatMsg.FeetGroundStatus = 0x11;
	int  reach_final = 1;
	//while (reach_final != 0)
	//{
		walkingFlag = true;
		if (walkingFlag == true)
			//if (walkingFlag == true and ethercatMsg.FeetGroundStatus == 0x11)
		{
			cv::Point start(1302, 1142);  // Define start point
			//Point start(1361, 1155);  // Define start point
			cv::Point goal(1183, 991);//efine goal point
			//Point goal(1150, 799);  // Define goal point
			//Point goal(1093, 874);  // Define goal point
			std::string fnC = "Path.csv";
			std::string fnL = "leftPath.csv";
			std::string fnR = "rightPath.csv";
			genernateFeetPos(start,goal,fnC, fnL, fnR);
			reach_final = run(fnC,fnL,fnR);
		}
	//}
}

int main(int argc, char** argv) {
	/*ros::init(argc, argv, "footstepplanning");
	ros::NodeHandle n("~");
	ros::Rate poll_rate(100);
	sub_ethercat = n.subscribe("/ethercat", 1000, &ethercat_callback);
	sub_walkingflag = n.subscribe("/walkingflag", 1000, &walkingflag_callback);
	pub_footstep = n.advertise<footstep_planning::Footstep>("/footstep", 1000);
	while (pub_footstep.getNumSubscribers() == 0) {
		poll_rate.sleep();
	}*/
	//std::thread run_thread(std::thread(static_cast<void(*)()>(run_)));
	//ros::spin();
	//run_thread.join();
	run_();
	return 0;
}