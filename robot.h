#ifndef ROBOT_H_
#define ROBOT_H_

#include "workbench.h"
#include <vector>
using namespace std;

class Robot {
public:
	// 计算机器人与工作台之间的距离
	double calDistance(const Workbench& workbench);

	// 计算机器人与机器人之间的距离
	double calDistance(const Robot& robot);

	// 计算两个向量之间的夹角（单位为弧度）
	double angleBetween(double dx1, double dy1, double dx2, double dy2);

	// 计算需要转的角度大小
	double adjustDirection(double targetX, double targetY, double x, double y, double angle);

	// 计算角速度
	double calAngleSpeed(const Workbench& workbench, double& angleToTarget);

	// 判断是否在墙边
	bool isBesideBoundary();

	// 计算在工作台范围内经过的帧数
	void calWaitFrame(const Workbench& workbench);

	// 计算速度
	void calSpeed(const Workbench& workbench, int& lineSpeed, double& angleSpeed, int map = 0);

	// 移动
	void move(Workbench& workbench, int map = 0);

	// 令count循环
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