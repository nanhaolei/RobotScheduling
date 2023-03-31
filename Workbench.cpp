#include "workbench.h"
using namespace std;

// ���㹤��̨�빤��̨֮��ľ���
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

// ����ȱʧ�Ĳ���
int Workbench::getLostMaterial() {
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

// �����ϸ�״̬��int��ת����map
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

// ��ȡ�ѳ��еĲ�������	
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

