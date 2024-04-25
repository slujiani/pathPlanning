#define _USE_MATH_DEFINES
#include<iostream>
#include<fstream>
#include<sstream>
#include<vector>
#include<cmath>
#include<random>

#define vs 0.4	// ֱ�������ٶȣ�ÿ����/��
#define vc 0.1	// ���������ٶȣ�ÿ����/��
#define lcTh 1* M_PI / 180;	// ֱ��·��������·��ת��������ֵ����λ����
using namespace std;
struct xy
{
	double x;
	double y;
};
struct posDirect
{
	xy pos;
	xy direct;
	int tag;	// 1 ֱ�߶Σ�-1 ���߶Σ�0 �յ�
};

xy vecMulC(xy a, double C)
{
	xy res = { a.x * C,a.y * C };
	return res;
}
double dotOfVec(xy a, xy b)
{
	double res = a.x * b.x + a.y * b.y;
	return res;
}
xy vecAsubB(xy a, xy b)
{
	xy res = { a.x - b.x,a.y - b.y };
	return res;
}
void fileToData(vector<posDirect>& data, string filename)
{
	ifstream csvData(filename, ios::in);
	if (!csvData.is_open())
	{
		cout << "Error: opening file fail" << endl;
		exit(1);
	}
	else
	{
		string line, word;
		istringstream sin;
		posDirect newNode;
		while (getline(csvData, line))
		{
			sin.str(line);
			getline(sin, word, ',');
			newNode.pos.x = stod(word);
			getline(sin, word, ',');
			newNode.pos.y = stod(word);
			getline(sin, word, ',');
			newNode.direct.x = stod(word);
			getline(sin, word, ',');
			newNode.direct.y = stod(word);
			getline(sin, word, ',');
			newNode.tag = stod(word);

			data.push_back(newNode);
		}
		csvData.close();
	}
}
//lcTh��ֱ��·��������·��ת��������ֵ����λ����
std::tuple<double, double, double> getRoadLen(vector<posDirect>path)
{
	int pathSize = path.size();
	double L = 0;	// ·������
	double Ll = 0;	// ֱ�߳���
	double Lc = 0;	// ���߳���
	for (int i = 0; i < pathSize; i++)
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
	return make_tuple(L, Lc, Ll);
}
//����·�����뵱ǰ������ĵ㣬����ǰ��鵽��һ��ȥ
int searchNearestPos(int low, int high, xy cur, vector<posDirect>path)
{
	int minp = INT_MAX;
	int ret=-1;
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
std::tuple<posDirect,int> getOneFoot(vector<posDirect>path,int ind,posDirect cur,double v,int state_change)
{
	int pathSize = path.size();
	int lowInd = min(ind + 1, pathSize - 1);
	int i, e;
	posDirect ret;
	for (i = lowInd; i < pathSize - 1; i++)
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
	//�ж�������������Ƿ���״̬�仯������仯��state_change== 1
	for (i = ind; i < e; i++)
	{
		if (path[i].tag != path[i+1].tag)
		{
			state_change = -1;
			break;
		}
	}
	//��������ﵽ�յ㣬����һ���滮�����յ�λ��
	if (i == pathSize - 1)
	{
		ret = path[i];
	}
	return make_tuple(ret, state_change);
}
//path��������·������������㼰ǰ��ĵ�λ,state_change����ֵ����֮ǰ�Ļ�һ��
std::tuple<posDirect,posDirect,int> getPosFit(vector<posDirect>path_behind, vector<posDirect>path_forward, posDirect cur_b,posDirect cur_f, int ind_b,int ind_f,double v_b,double v_f)
{
	posDirect pD_b, pD_f;
	//�����ӵ�ǰλ�ÿ�ʼ��ǰ������ֱ���˶����ȴﵽָ������
	int pathSize = path_behind.size();
	int lowInd = min(ind_b + 1, pathSize - 1);
	int i,e;
	int state_change=0;
	std::tie(pD_b, state_change) = getOneFoot(path_behind, ind_b, cur_b, v_b, state_change);
	std::tie(pD_f, state_change) = getOneFoot(path_forward, ind_f, cur_f, v_f,state_change);
	return make_tuple(pD_b, pD_f, state_change);
}
//vk=LR/LL LR\LL�����������켣�����߶εĳ���
std::tuple<int,posDirect,int,posDirect,int,int> getNextLocat(vector<posDirect>left, vector<posDirect>right, posDirect Lcur, posDirect Rcur, double vk, int pll, int SC)
{
	int FR = 0;	//������һ�㣬1 ���㣬-1 ���㣬0 ����
	posDirect nextLeftF = Lcur, nextRightF = Rcur;
	double lt = 0;	//�����˶�ʱ��
	double rt = 0;
	int stateChange;	// state_change==1 �ڻ����˰��滮·�����߻ᷢ��״̬�仯��state_change==0����״̬
	int vw = 1;	//�������vk�ñ������ڲ������ٶȣ�
	//�ҵ����㵱ǰλ����滮��λ���������ĸ�λ�����
	int pathSize = left.size();

	int lind = searchNearestPos(0, pathSize, Lcur.pos, left);
	//���������λ�ø������������������ҵ�������λ������ĵ�
	int lowInd = max((int)(lind - 0.1 * pathSize), 0);
	int highInd = min((int)(lind - 0.1 * pathSize), pathSize);
	int rind = searchNearestPos(lowInd, highInd, Lcur.pos, right);

	double lv, rv;//���ҽ��ٶ�
	int i;
	int e;
	//���㲢��
	if (abs(lind - rind) <= 3 || pll == 1)
	{
		if (lind == pathSize-1)//�ѵ����յ㣬����
		{
			FR = 0;
			lt = 0;
			return std::make_tuple(FR, nextLeftF, lt, nextRightF, rt, stateChange);
		}
		//���ݹ滮����һ���״̬��ֱ�߻������ߣ�����˫���ٶ�
		if (left[min(lind + 1, pathSize-1)].tag == -1 || right[min(rind + 1, pathSize-1)].tag == -1)
		{
			// �����˶�
			lv = 0.5 * vc;// ����Ȩ���˶���˫�㲢�к������ȶ����˶��벽
			rv = vk * vc;// ��������˶��������ٶȣ��������������߶γ��ȵı�������
		}
		else
		{
			// ֱ���˶�
			lv = 0.5 * vs; // ����벽
			rv = vw * vs; // ��������
		}
		//����ӵ�ǰλ�ÿ�ʼ����ǰ������ֱ���˶����ȴﵽָ������
		lowInd = min(lind + 1, pathSize - 1);

		for ( i = lowInd; i < pathSize; i++)
		{
			xy diff = vecAsubB(left[i].pos, Lcur.pos);
			double D = dotOfVec(diff, diff);
			if (D >= lv * lv)
			{
				nextLeftF = left[i];
				lt = 0; //���˶���Ϊ0
				FR = 1;
				e = i;
				break;
			}
		}
		//�ж�������������Ƿ���״̬�仯������仯��state_change== 1
		
		for ( i = lind; i < e; i++)
		{
			if (left[i].tag != left[i + 1].tag)
			{
				stateChange = 1;
				break;
			}
		}
		//�������ﵽ�յ㣬����һ���滮�����յ�λ��
		if (i == pathSize - 1)
		{
			nextLeftF = left[i];
			lt = 0;
			FR = 1;
		}
		//����ӵ�ǰλ�ÿ�ʼ��ǰ������ֱ���˶����ȴﵽָ������
		lowInd = min(rind + 1, pathSize - 1);
		for ( i = lowInd; i < pathSize; i++)
		{
			xy diff = vecAsubB(right[i].pos, Lcur.pos);
			double D = dotOfVec(diff, diff);
			if (D >= rv * rv)
			{
				nextRightF = right[i];
				rt = 1; //���˶���Ϊ1
				FR = 1;
				e = i;
				break;
			}
		}
		//�ж�������������Ƿ���״̬�仯������仯��state_change == 1
		for (i = rind; i < e; i++)
		{
			if (right[i].tag != right[i + 1].tag)
			{
				stateChange = 1;
				break;
			}
		}
		//�������ﵽ�յ㣬����һ���滮�����յ�λ��
		if (i == pathSize - 1)
		{
			nextRightF = right[pathSize - 1];
			rt = 1;
			FR = 1;
		}

	}
	//һ�㳬ǰ��һ��������˶������
	if (lind > rind)//���㳬ǰ,�������
	{
		rt = 0;
		FR = -1;
		//������ϴ��˶��з�����״̬�仯����ǿ�������һ�����У���һ�㲻��
		if (SC == 1 || left[lind].tag != left[min(lind + 1, pathSize - 1)].tag)
		{
			nextRightF = right[lind];
			lt = 1;//���㲻��
			return std::make_tuple(FR, nextLeftF, lt, nextRightF, rt, stateChange);
		}
		//�ж�����ϵһ����״̬��ֱ�߻������ߣ����������㲽��
		if (right[rind + 1].tag == -1)
		{
			lv = vc;
			rv = vk * vc;
		}
		else
		{
			lv = vs;
			rv = vw * vs;
		}
		std::tie(nextRightF, nextLeftF, stateChange) = getPosFit(right, left, Rcur, Lcur, rind, lind, rv, lv);
		//�ж�������������Ƿ���״̬�仯������仯��state_change == 1

	}
	else
	{
		lt = 0;
		FR = 1;
		//������ϴ��˶��з�����״̬�仯����ǿ�������һ�����У���һ�㲻��
		if (SC == 1 || right[lind].tag != right[min(lind + 1, pathSize - 1)].tag)
		{
			nextLeftF = left[lind];
			rt = 1;//���㲻��
			return std::make_tuple(FR, nextLeftF, lt, nextRightF, rt, stateChange);
		}
		//�ж�����ϵһ����״̬��ֱ�߻������ߣ����������㲽��
		if (left[lind + 1].tag == -1)
		{
			lv = vc;
			rv = vk * vc;
		}
		else
		{
			lv = vs;
			rv = vw * vs;
		}
		std::tie(nextLeftF, nextRightF,  stateChange) = getPosFit(left, right, Lcur, Rcur, lind, rind, lv, rv);
	}
	return std::make_tuple(FR, nextLeftF, lt, nextRightF, rt, stateChange);
}
struct xyz
{
	double x;
	double y;
	double z;
};
double dotOfxyz(xyz a, xyz b)
{
	double res = a.x * b.x + a.y * b.y + a.z * b.z;
	return res;
}
xyz xyzMulC(xyz a, double C)
{
	xyz res = { a.x * C,a.y * C,a.z * C };
	return res;
}
xyz normal_xyz(xyz a)
{
	double N = sqrt(dotOfxyz(a, a));
	xyz res = xyzMulC(a, (1 / N));
	return res;
}
std::tuple<int,xyz,xyz,xyz> get_R_Q2(posDirect cur, double z, xyz n, vector<vector<double>>&R, vector<double>&Q)
{
	int flag = 1;
	double N = 1/(sqrt(dotOfxyz(n,n)));
	n = xyzMulC(n, N);

	//��������㷨������DEM����ϵZ��нǣ�����Ƕȴ���90�ȣ������쳣��-1��
	xyz v0 = { 0,0,1 };
	xyz v1 = n;
	xyz vx,vy,vz;
	double a0 = acos(dotOfxyz(v0, v1));
	if (abs(a0) >= M_PI / 2)
	{
		flag = -1;
		return make_tuple(flag, vx, vy, vz);
	}
	
	//
	double v0_3 = -(1 / n.z) * (n.x * cur.direct.x + n.y * cur.direct.y);
	//������ϵx����DEM�����е�ʸ��v
	xyz cur_dire_xyz = { cur.direct.x,cur.direct.y,v0_3 };
	cur_dire_xyz = normal_xyz(cur_dire_xyz);
	// ������ϵ��y����������ϵx�ᣬz�ṹ������ϵ
	vz = n;
	vx = cur_dire_xyz;
	 
	vy.x = vz.y * vx.z - vz.z * vx.y;
	vy.y = vz.z * vx.x - vz.x * vx.z;
	vy.z = vz.x * vx.y - vz.y * vx.x;
	vy = normal_xyz(vy);
	//���������ϵ��DEM����ϵ�еķ�����̬������R
	R = { {vx.x,vy.x,vz.x},{vx.y,vy.y,vz.y},{vx.z,vy.z,vz.z} };
	//���������ת������Ԫ����ʾ
	//q0Ϊ��Ԫ�����ȣ�����
	
	Q[0] = 0.5 * sqrt(1 + vx.x + vy.y + vz.z);
	Q[1] = 0.25 * (vy.z - vz.y) / Q[0];
	Q[2] = 0.25 * (vz.x - vx.z) / Q[0];
	Q[3] = 0.25 * (vx.y - vy.x) / Q[0];
	return make_tuple(flag, vx, vy, vz);
}
// ����һ�����ϱ�׼��̬�ֲ��������
double generateNormalRandom() 
{
    // ʹ�ñ�׼��̬�ֲ�����ֵΪ0������Ϊ1
    std::normal_distribution<double> distribution(0.0, 1.0);
    // ����һ�������������
    std::random_device randomDevice;
    std::mt19937 generator(randomDevice());
    // ���������
    return distribution(generator);
}
xy addGaussinToSim(xy goalPos)
{
	xy res;
	res.x = goalPos.x + 0.005 * generateNormalRandom();
	res.y = goalPos.y + 0.005 * generateNormalRandom();
	return res;
}
void dataToFile(vector<posDirect>Path, string fname)
{
	std::ofstream outFile(fname, ios::out);
	int peSize = Path.size();
	for (int i = 0; i < peSize; i++)
	{
		outFile << to_string(Path[i].pos.x) << ','
			<< to_string(Path[i].pos.y) << ','
			<< to_string(Path[i].direct.x) << ','
			<< to_string(Path[i].direct.y) << ','
			<< to_string(Path[i].tag) << endl;
	}
}
int main()
{
	vector<posDirect> leftPath, rightPath;
	string fname1= "leftPath.csv";
	string fname2 = "rightPath.csv";
	fileToData(leftPath, fname1);
	fileToData(rightPath, fname2);
	double lenLeft, lenLline, lenLcurve;
	double lenRight, lenRline, lenRcurve;
	std::tie(lenLeft, lenLline, lenLcurve) = getRoadLen(leftPath);
	std::tie(lenRight, lenRline, lenRcurve) = getRoadLen(rightPath);
	int FR = -1;	//�����յ�ı�־
	int SC = 0;	// �������м���״̬�仯�������߶ε�ֱ�߶α仯�����෴
	int last_SC = 0;
	int pll = 1;	// �����߹������Ƿ����˫�㲢�е����
	double vk = lenRight / lenLcurve;
	posDirect nextLeftF, nextRightF;
	int stateChange,lt,rt;
	posDirect leftcur = leftPath[0];
	posDirect rightcur = rightPath[0];
	vector<posDirect>recordcalL, recordcalR;
	vector<posDirect>recordrealL, recordrealR;
	recordcalL.push_back(leftcur);
	recordcalR.push_back(rightcur);
	recordrealL.push_back(leftcur);
	recordrealR.push_back(rightcur);
	while (FR!=0)
	{
		//���ݵ�ǰ�����λ�ú͹滮��ȫ��·���ͳ���õ���һ��˫��������λ�úͷ���
		std::tie(FR, nextLeftF, lt, nextRightF, rt, stateChange) = getNextLocat(leftPath, rightPath, leftcur, rightcur, vk, pll, SC);
		//����õ�����һ����
		recordcalL.push_back(nextLeftF);
		recordcalR.push_back(nextRightF);
		
		leftcur.pos = addGaussinToSim(nextLeftF.pos);
		leftcur.direct = nextLeftF.direct;
		leftcur.tag = nextLeftF.tag;

		rightcur.pos = addGaussinToSim(nextRightF.pos);
		rightcur.direct = nextRightF.direct;
		rightcur.tag = nextRightF.tag;
		//�Ӹ�˹ƫ��ģ���ʵ��ֵ
		recordrealL.push_back(leftcur);
		recordrealR.push_back(rightcur);

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
	}
	string fileLsim = "file_sim_left.csv";
	string fileRsim = "file_sim_right.csv";
	string fileLreal = "file_real_left.csv";
	string fileRreal = "file_real_right.csv";
	dataToFile(recordcalL, fileLsim);
	dataToFile(recordcalR, fileRsim);
	dataToFile(recordrealL, fileLreal);
	dataToFile(recordrealR, fileRreal);
	return 0;
}