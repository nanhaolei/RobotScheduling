#include "workbench.h"
using namespace std;

// 计算工作台与工作台之间的距离
double Workbench::calDistance(const Workbench& workbench) {
	double r_coor_x = this->getCoordinateX();
	double r_coor_y = this->getCoordinateY();
	double w_coor_x = workbench.getCoordinateX();
	double w_coor_y = workbench.getCoordinateY();
	double dx = w_coor_x - r_coor_x;
	double dy = w_coor_y - r_coor_y;
	double distance = sqrt(dx * dx + dy * dy);
	return distance;
}

// 返回缺失的材料
int Workbench::getLostMaterial() {
	//int bench_type = workbenchs[workbenchId].getType();
	//int material = workbenchs[workbenchId].getMaterialStatus();
	int bench_type = this->type;
	int material = this->materialStatus;
	int goods_type = -1;
	int ran[2]{ -1, 1 };
	random_device rd;  //如果可用的话，从一个随机数发生器上获得一个真正的随机数
	mt19937 gen(rd()); //gen是一个使用rd()作种子初始化的标准梅森旋转算法的随机数发生器
	uniform_int_distribution<> distrib1(0, 1);
	uniform_int_distribution<> distrib2(0, 2);
	switch (bench_type)
	{
	case 4:
		// 有1缺2
		if (material == 2) {
			goods_type = 2;
		}
		// 有2缺1
		else if (material == 4) {
			goods_type = 1;
		}
		// 都缺
		else {
			goods_type = 2;
		}
		break;
	case 5:
		// 有1缺3
		if (material == 2) {
			goods_type = 3;
		}
		// 有3缺1
		else if (material == 8) {
			goods_type = 1;
		}
		// 都缺
		else {
			goods_type = 1;
		}
		break;
	case 6:
		// 有2缺3
		if (material == 4) {
			goods_type = 3;
		}
		// 有3缺2
		else if (material == 8) {
			goods_type = 2;
		}
		// 都缺
		else {
			goods_type = 3;
		}
		break;
	case 7:
		switch (material)
		{
			// 有45缺6
		case 48:
			goods_type = 6;
			break;
			// 有46缺5
		case 80:
			goods_type = 5;
			break;
			// 有56缺4
		case 96:
			goods_type = 4;
			break;
			// 有4缺56
		case 16:
			goods_type = 5 + distrib1(gen); // 随机56
			break;
			// 有5缺46
		case 32:
			goods_type = 5 + ran[rand() % 2]; // 随机46
			break;
			// 有6缺45
		case 64:
			goods_type = 4 + distrib1(gen); // 随机45
			break;
			// 都缺
		default:
			goods_type = 4 + distrib2(gen); // 随机456
			break;
		}
		break;
	default:
		break;
	}

	return goods_type;
}

// 返回产品利润
double Workbench::getProfit(double actual_time) {
	double timeCoefficient = 1 - sqrt(1 - (1 - actual_time * FPS / TOTAL_FRAME) * (1 - actual_time * FPS / TOTAL_FRAME)) * 0.2 + 0.8;
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

// 将材料格状态从int型转换到map
void Workbench::convertMaterialStatus(int ms) {
	if (ms == 0) {
		for (int i = 1; i <= GOODS_TYPE_SIZE; ++i) {
			holdMaterial[i] = false;
		}
		return;
	}
	int i = 0;
	while (ms > 0) {
		if (ms % 2 == 1) {
			holdMaterial[i++] = true;
		}
		else {
			holdMaterial[i++] = false;
		}
		ms /= 2;
	}
}

// 获取已持有的材料数量	
int Workbench::getFullCount() {
	int count = 0;
	for (int i = 1; i <= GOODS_TYPE_SIZE; i++) {
		if (holdMaterial[i])
			count++;
	}
	return count;
}

Workbench::Workbench(int _workbenchId, int _type) :workbenchId(_workbenchId), type(_type) {
	coordinate[0] = 0;
	coordinate[1] = 0;
	restFrame = -1;
	materialStatus = 0;
	productStatus = 0;
	reservedMaterial = {
		{1,false},
		{2,false},
		{3,false},
		{4,false},
		{5,false},
		{6,false},
		{7,false}
	};
	holdMaterial = {
		{1,false},
		{2,false},
		{3,false},
		{4,false},
		{5,false},
		{6,false},
		{7,false}
	};
};

