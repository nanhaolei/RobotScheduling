#ifndef ROBOT_H_
#define ROBOT_H_

#include "workbench.h"
#include "geometry.h"
#include <unordered_set>
#include <vector>
#include <cassert>
#include <random>
using namespace std;
using namespace geometry;

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
	void calSpeed(const Workbench& workbench, int& lineSpeed, double& angleSpeed, int cur_map = 0);

	// 移动
	void move_old(Workbench& workbench, int cur_map = 0);

	// 碰撞检测
	void checkCollision_old(vector<Robot*> robots, int cur_map = 0);

	// 令count循环
	int getCount();
	int getCount12();
	int getCount23();
	int getCount13();

	// 计算角速度
	double calAngleSpeed(const Vec2 target, double& angleToTarget);
	// 计算线速度
	void calSpeed(const Vec2 target, int& lineSpeed, double& angleSpeed);
	// 计算角速度
	double CalcNeedRotateAngle(const Vec2& point) const;
	// 计算线速度
	void CalcForwardSpeedAndRotateSpeed(const Vec2& target, int& lineSpeed, double& angleSpeed);
	// 移动
	void move();
	// 判断是否已到达下一个节点
	bool isReachNode();
	// 记录不可达工作台
	void addUnreachableBench(int workbench_id);
	// 判断工作台是否可达
	bool isReachable(int workbench_id);
	// 碰撞检测
	void checkCollision(vector<Robot*> robots);
	// 记录碰撞时长
	void addCollisionFrame();
	// 检测路径为空
	bool checkPath();
	// 检测静止
	void checkStatic();
	void reset();
private:
	int robotId;
	int workbenchId;
	int goodsType;
	double timeCoefficient;
	double collisionCoefficient;
	double angleSpeed;
	Vec2 lineSpeed;
	double direction;
	//double coordinate[2];
	int targetBenchId;
	int sellBenchId;
	int waitFrame;
	int waitSellFrame;
	int count;
	vector<Vec2> path;
	vector<Vec2> rawPath;
	Vec2 coordinate;
	unordered_set<int> unreachableBench;
	Workbench* targetBench;
	Vec2 startCoor;
	Vec2 goalCoor;
	int collisionFrame;
	int isCollision;
	int moveStatus;
	int staticFrame;
	unordered_set<Robot*> otherRobots;
	Vec2 blockCoor;
	int blockStatus;
	Vec2 oldCoor;

public:
	Robot(int _robotId);
	void setOldCoordinate(Vec2 oldCoordinate) { this->oldCoor = oldCoordinate; }
	Vec2 getOldCoordinate() { return oldCoor; }
	void addOtherRobots(Robot* otherRobot) { otherRobots.insert(otherRobot); }
	int getBlockStatus() { return blockStatus; }
	void setBlockStatus(int status) { blockStatus = status; }
	int getMoveStatus() { return moveStatus; }
	void setMoveStatus(int status) { moveStatus = status; }
	int getIsCollision() { return isCollision; }
	void setIsCollision(int is) { isCollision = is; }
	int getCollisionFrame() { return collisionFrame; }
	void setCollisionFrame(int frame) { collisionFrame = frame; }
	Vec2 getCoordinate() { return coordinate; }
	void setPath(vector<Vec2> path) {
		this->path = path;
		if(path.size() > 0) {
			setStartCoor(path[0]);
			setGoalCoor(path.back());
		}
	}
	vector<Vec2> getPath() { return path; }
	void setTargetBench(Workbench* workbench) { targetBench = workbench; }
	Workbench* getTargetBench() { return targetBench; }
	void setStartCoor(Vec2 startCoor) { this->startCoor = startCoor; }
	Vec2 getStartCoor() { return startCoor; }
	void setGoalCoor(Vec2 goalCoor) { this->goalCoor = goalCoor; }
	Vec2 getsetGoalCoor() { return goalCoor; }

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

};

#endif