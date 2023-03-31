#ifndef ROBOT_H_
#define ROBOT_H_

#include "workbench.h"
#include <vector>
using namespace std;

class Robot {
public:
	// ����������빤��̨֮��ľ���
	double calDistance(const Workbench& workbench);

	// ����������������֮��ľ���
	double calDistance(const Robot& robot);

	// ������������֮��ļнǣ���λΪ���ȣ�
	double angleBetween(double dx1, double dy1, double dx2, double dy2);

	// ������Ҫת�ĽǶȴ�С
	double adjustDirection(double targetX, double targetY, double x, double y, double angle);

	// ������ٶ�
	double calAngleSpeed(const Workbench& workbench, double& angleToTarget);

	// �ж��Ƿ���ǽ��
	bool isBesideBoundary();

	// �����ڹ���̨��Χ�ھ�����֡��
	void calWaitFrame(const Workbench& workbench);

	// �����ٶ�
	void calSpeed(const Workbench& workbench, int& lineSpeed, double& angleSpeed, int map = 0);

	// �ƶ�
	void move(Workbench& workbench, int map = 0);

	// ��ײ���
	void checkCollision(vector<Robot> robots, int map = 0);

	// ��countѭ��
	int getCount();
	int getCount12();
	int getCount23();
	int getCount13();

private:
	int robotId;
	int workbenchId;
	int goodsType;
	double timeCoefficient;
	double collisionCoefficient;
	double angleSpeed;
	double lineSpeed[2];
	double direction;
	double coordinate[2];
	int targetBenchId;
	int sellBenchId;
	int waitFrame;
	int count;
	int waitSellFrame;

public:
	Robot(int _robotId);
	~Robot() {};
	inline void forward(double speed) {
		printf("forward %d %f\n", this->robotId, speed);
	}
	inline void rotate(double speed) {
		printf("rotate %d %f\n", this->robotId, speed);
	}
	inline void buy() {
		printf("buy %d\n", this->robotId);
	}
	inline void sell() {
		printf("sell %d\n", this->robotId);
	}
	inline void destroy() {
		printf("destroy %d\n", this->robotId);
	}
	int getRobotId() const { return robotId; }
	void setRobotId(int id) { robotId = id; }
	int getWorkbenchId() const { return workbenchId; }
	void setWorkbenchId(int id) { workbenchId = id; }
	int getGoodsType() const { return goodsType; }
	void setGoodsType(int type) { goodsType = type; }
	double getTimeCoefficient() const { return timeCoefficient; }
	void setTimeCoefficient(double coefficient) { timeCoefficient = coefficient; }
	double getCollisionCoefficient() const { return collisionCoefficient; }
	void setCollisionCoefficient(double coefficient) { collisionCoefficient = coefficient; }
	double getAngleSpeed() const { return angleSpeed; }
	void setAngleSpeed(double velocity) { angleSpeed = velocity; }
	double getLineSpeedX() const { return lineSpeed[0]; }
	void setLineSpeedX(double velocity) { lineSpeed[0] = velocity; }
	double getLineSpeedY() const { return lineSpeed[1]; }
	void setLineSpeedY(double velocity) { lineSpeed[1] = velocity; }
	double getDirection() const { return direction; }
	void setDirection(double _direction) { direction = _direction; }
	double getCoordinateX() const { return coordinate[0]; }
	void setCoordinateX(double x) { coordinate[0] = x; }
	double getCoordinateY() const { return coordinate[1]; }
	void setCoordinateY(double y) { coordinate[1] = y; }
	int getTargetBenchId() const { return targetBenchId; }
	void setTargetBenchId(int id) { targetBenchId = id; }
	int getSellBenchId() const { return sellBenchId; }
	void setSellBenchId(int id) { sellBenchId = id; }
};

#endif