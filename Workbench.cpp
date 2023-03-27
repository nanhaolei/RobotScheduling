#include <iostream>
#include <math.h>
#include <unordered_map>
#include <random>
#include "constant.h"
#include "utils.cpp"
using namespace std;

class Workbench {
public:
	// ���㹤��̨�빤��̨֮��ľ���
	inline double calDistance(const Workbench& workbench) {
		double r_coor_x = this->getCoordinateX();
		double r_coor_y = this->getCoordinateY();
		double w_coor_x = workbench.getCoordinateX();
		double w_coor_y = workbench.getCoordinateY();
		double dx = w_coor_x - r_coor_x;
		double dy = w_coor_y - r_coor_y;
		double distance = length(dx, dy);
		return distance;
	}

	// ���ò��ϸ��Ƿ����
	bool checkMaterialStatus(const int& goodsType) {
		int materialStatus = this->getMaterialStatus();
		int i = 0;
		vector<int> binary(8, 0);
		while (materialStatus > 0) {
			binary[i] = materialStatus % 2;
			materialStatus /= 2;
			++i;
		}
		if (binary[goodsType] == 0) {
			return true;
		}
		else {
			return false;
		}
	}

	// ���ò��ϸ��Ƿ���� �����ؿ������
	bool checkMaterialStatus(const int& goodsType, vector<int>& binary) {
		int materialStatus = this->getMaterialStatus();
		int i = 0;
		while (materialStatus > 0) {
			binary[i] = materialStatus % 2;
			materialStatus /= 2;
			++i;
		}
		if (binary[goodsType] == 0) {
			return true;
		}
		else {
			return false;
		}
	}

	// ����ȱʧ�Ĳ���
	int getLostMaterial() {
		//int bench_type = workbenchs[workbenchId].getType();
		//int material = workbenchs[workbenchId].getMaterialStatus();
		int bench_type = this->type;
		int material = this->materialStatus;
		int goods_type = -1;
		int ran[2]{ -1, 1 };
		random_device rd;  //������õĻ�����һ��������������ϻ��һ�������������
		mt19937 gen(rd()); //gen��һ��ʹ��rd()�����ӳ�ʼ���ı�׼÷ɭ��ת�㷨�������������
		uniform_int_distribution<> distrib1(0, 1);
		uniform_int_distribution<> distrib2(0, 2);
		switch (bench_type)
		{
		case 4:
			// ��1ȱ2
			if (material == 2) {
				goods_type = 2;
			}
			// ��2ȱ1
			else if (material == 4) {
				goods_type = 1;
			}
			// ��ȱ
			else {
				goods_type = 2;
			}
			break;
		case 5:
			// ��1ȱ3
			if (material == 2) {
				goods_type = 3;
			}
			// ��3ȱ1
			else if (material == 8) {
				goods_type = 1;
			}
			// ��ȱ
			else {
				goods_type = 1;
			}
			break;
		case 6:
			// ��2ȱ3
			if (material == 4) {
				goods_type = 3;
			}
			// ��3ȱ2
			else if (material == 8) {
				goods_type = 2;
			}
			// ��ȱ
			else {
				goods_type = 3;
			}
			break;
		case 7:
			switch (material)
			{
				// ��45ȱ6
			case 48:
				goods_type = 6;
				break;
				// ��46ȱ5
			case 80:
				goods_type = 5;
				break;
				// ��56ȱ4
			case 96:
				goods_type = 4;
				break;
				// ��4ȱ56
			case 16:
				goods_type = 5 + distrib1(gen); // ���56
				break;
				// ��5ȱ46
			case 32:
				goods_type = 5 + ran[rand() % 2]; // ���46
				break;
				// ��6ȱ45
			case 64:
				goods_type = 4 + distrib1(gen); // ���45
				break;
				// ��ȱ
			default:
				goods_type = 4 + distrib2(gen); // ���456
				break;
			}
			break;
		default:
			break;
		}

		return goods_type;
	}

	// ���ز�Ʒ����
	double getProfit(double actual_time) {
		double timeCoefficient = 1 - sqrt(1 - (1 - actual_time * FPS / 9000) * (1 - actual_time * FPS / 9000)) * 0.2 + 0.8;
		int goods_type = this->type;
		switch (goods_type)
		{
		case 1:
			return 6000 * timeCoefficient - 3000;
			break;
		case 2:
			return 7600 * timeCoefficient - 4400;
			break;
		case 3:
			return 9200 * timeCoefficient - 5800;
			break;
		case 4:
			return 22500 * timeCoefficient - 15400;
			break;
		case 5:
			return 25000 * timeCoefficient - 17200;
			break;
		case 6:
			return 27500 * timeCoefficient - 19200;
			break;
		case 7:
			return 105000 * timeCoefficient - 76000;
			break;
		default:
			break;
		}
		cerr << "err:calProfit" << endl;
		return -1;
	}


private:
	int workbenchId;
	int type;
	double coordinate[2];
	int restFrame;
	int materialStatus;
	int productStatus;
	unordered_map <int, bool> reservedGoods;
	unordered_map <int, bool> holdGoods;
public:
	Workbench(int _workbenchId, int _type) :workbenchId(_workbenchId), type(_type) {
		coordinate[0] = 0;
		coordinate[1] = 0;
		restFrame = -1;
		materialStatus = 0;
		productStatus = 0;
		reservedGoods = {
			{1,false},
			{2,false},
			{3,false},
			{4,false},
			{5,false},
			{6,false},
			{7,false},
			{8,false},
			{9,false}
		};
		holdGoods = {
			{1,false},
			{2,false},
			{3,false},
			{4,false},
			{5,false},
			{6,false},
			{7,false},
			{8,false},
			{9,false}
		};
	};
	~Workbench() {};
	int getWorkbenchId() const { return workbenchId; }
	void setWorkbenchId(int id) { workbenchId = id; }
	int getType() const { return type; }
	void setType(int t) { type = t; }
	double getCoordinateX() const { return coordinate[0]; }
	void setCoordinateX(double x) { coordinate[0] = x; }
	double getCoordinateY() const { return coordinate[1]; }
	void setCoordinateY(double y) { coordinate[1] = y; }
	int getRestFrame() const { return restFrame; }
	void setRestFrame(int rt) { restFrame = rt; }
	int getMaterialStatus() const { return materialStatus; }
	// to do:bug
	void setMaterialStatus(int ms) {
		materialStatus = ms;
		int i = 0;
		vector<int> binary(8, 0);
		while (ms > 0) {
			binary[i] = ms % 2;
			ms /= 2;
			++i;
		}
		for (int i = 1; i < 8; ++i) {
			if (binary[i] == 1) {
				holdGoods[i] = true;
			}
			else {
				holdGoods[i] = false;
			}
		}
		/*while (ms > 0) {
			if (ms % 2 == 1) {
				holdGoods[i] = true;
			}
			else {
				holdGoods[i] = false;
			}
			ms /= 2;
			++i;
		}*/
	}
	int getProductStatus() const { return productStatus; }
	void setProductStatus(int ps) { productStatus = ps; }
	bool getReservedGoods(int goods) { return reservedGoods[goods]; }
	void setReservedGoods(int goods, bool status) { reservedGoods[goods] = status; }
	bool getHoldGoods(int goods) { return holdGoods[goods]; }
	void setHoldGoods(int goods, bool status) { holdGoods[goods] = status; }

	void print() const {
		std::cerr << "workbenchId: " << workbenchId << std::endl;
		std::cerr << "type: " << type << std::endl;
		std::cerr << "coordinate: (" << coordinate[0] << ", " << coordinate[1] << ")" << std::endl;
		std::cerr << "restFrame: " << restFrame << std::endl;
		std::cerr << "materialStatus: " << materialStatus << std::endl;
		std::cerr << "productStatus: " << productStatus << std::endl;
	}


};
