#ifndef WORKBENCH_H_
#define WORKBENCH_H_

#include <iostream>
#include <math.h>
#include <unordered_map>
#include <random>
#include "constant.h"
#include "geometry.h"
using namespace std;
using namespace geometry;

class Workbench {
public:
	// ���㹤��̨�빤��̨֮��ľ���
	double calDistance(const Workbench& workbench);

	// ����ȱʧ�Ĳ���
	int getLostMaterial();

	// ���ز�Ʒ����
	double getProfit(double actual_time);

	// ��ȡ�ѳ��еĲ�������
	int getFullCount();

	// �����ϸ�״̬��int��ת����map
	void convertMaterialStatus(int ms);
private:
	int workbenchId;
	int type;
	Vec2 coordinate;
	int restFrame;
	int materialStatus;
	int productStatus;
	unordered_map <int, bool> reservedMaterial;
	unordered_map <int, bool> holdMaterial;
public:
	Workbench(int _workbenchId, int _type);
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
	int getProductStatus() const { return productStatus; }
	void setProductStatus(int ps) { productStatus = ps; }
	bool getReservedMaterial(int goods) { return reservedMaterial[goods]; }
	void setReservedMaterial(int goods, bool status) { reservedMaterial[goods] = status; }
	bool getHoldMaterial(int goods) { return holdMaterial[goods]; }
	void setHoldMaterial(int goods, bool status) { holdMaterial[goods] = status; }
	int getMaterialStatus() const { return materialStatus; }
	void setMaterialStatus(int ms) { 
		materialStatus = ms;
		convertMaterialStatus(ms);
	};
	Vec2 getCoordinate() { return Vec2{ coordinate[0], coordinate[1] }; }
	void print() const {
		std::cerr << "workbenchId: " << workbenchId << std::endl;
		std::cerr << "type: " << type << std::endl;
		std::cerr << "coordinate: (" << coordinate[0] << ", " << coordinate[1] << ")" << std::endl;
		std::cerr << "restFrame: " << restFrame << std::endl;
		std::cerr << "materialStatus: " << materialStatus << std::endl;
		std::cerr << "productStatus: " << productStatus << std::endl;
	}
};

#endif // !WORKBENCH_H
