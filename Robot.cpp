#include "Workbench.cpp"
#include <vector>
using namespace std;

class Robot {
public:
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

	// 计算机器人与工作台之间的距离
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

	// 计算机器人与机器人之间的距离
	inline double calDistance(const Robot& robot) {
		double r_coor_x = this->getCoordinateX();
		double r_coor_y = this->getCoordinateY();
		double w_coor_x = robot.getCoordinateX();
		double w_coor_y = robot.getCoordinateY();
		double dx = w_coor_x - r_coor_x;
		double dy = w_coor_y - r_coor_y;
		double distance = length(dx, dy);
		return distance;
	}

	// 计算两个向量之间的夹角（单位为弧度）
	inline double angleBetween(double dx1, double dy1, double dx2, double dy2) {
		double len1 = length(dx1, dy1);
		double len2 = length(dx2, dy2);
		double dot = dx1 * dx2 + dy1 * dy2;
		double cos = dot / (len1 * len2);
		if (cos < -1) {
			cos = -1;
		}
		if (cos > 1) {
			cos = 1;
		}
		return acos(cos);
	}

	// 计算需要转的角度大小
	inline double adjustDirection(double targetX, double targetY, double x, double y, double angle) {
		double dx2 = targetX - x;
		double dy2 = targetY - y;
		double dx1 = cos(angle);
		double dy1 = sin(angle);
		double angleToTarget = angleBetween(dx1, dy1, dx2, dy2);
		double cross = dx1 * dy2 - dy1 * dx2;
		if (cross > 0) {
			return angleToTarget;
		}
		else {
			return -angleToTarget;
		}
	}

	// 计算角速度
	inline double calAngleSpeed(const Workbench& workbench, double& angleToTarget) {
		double r_coor_x = this->getCoordinateX();
		double r_coor_y = this->getCoordinateY();
		double w_coor_x = workbench.getCoordinateX();
		double w_coor_y = workbench.getCoordinateY();
		//double angleToTarget = adjustDirection(w_coor_x, w_coor_y, r_coor_x, r_coor_y, this->getDirection());
		angleToTarget = adjustDirection(w_coor_x, w_coor_y, r_coor_x, r_coor_y, this->getDirection());
		double angleSpeed = angleToTarget / DELTATIME / 6;
		return angleSpeed;
	}

	// 判断是否在墙边
	inline bool isBesideBoundary() {
		double allow_distance = 2;
		if (this->getCoordinateX() < allow_distance || this->getCoordinateX() > 50 - allow_distance ||
			this->getCoordinateY() < allow_distance || this->getCoordinateY() > 50 - allow_distance) {
			return true;
		}
		return false;
	}

	// 计算在工作台范围内经过的帧数
	void calWaitFrame(const Workbench& workbench) {
		double distance = calDistance(workbench);
		// 在范围内计时
		if (distance < JUDGE_DISTANCE * 2) {
			++this->waitFrame;
		}
		// 计算在大范围内的总时间
		if (distance < JUDGE_DISTANCE * 4) {
			++this->waitSellFrame;
		}
		// 超出给定范围 或在墙边 或目标变更 则重置时间
		if (distance > JUDGE_DISTANCE * 4 || isBesideBoundary() || this->targetBenchId != workbench.getWorkbenchId()) {
			this->waitFrame = 0;
			this->waitSellFrame = 0;
		}
	}

	// 计算速度
	void calSpeed(const Workbench& workbench, int& lineSpeed, double& angleSpeed, int map = 0) {
		// 计算在工作台范围内的等待时间 超出允许等待时间让出位置
		int allow_frame = 80;
		calWaitFrame(workbench);
		if (this->waitFrame > allow_frame) {
			lineSpeed = MIN_FORWARD_SPEED;
			angleSpeed = 0;
			return;
		}

		// 计算运动速度
		double distance = calDistance(workbench);
		double cur_speed = length(this->lineSpeed[0], this->lineSpeed[1]);
		double angleToTarget;
		angleSpeed = calAngleSpeed(workbench, angleToTarget);
		lineSpeed = MAX_FORWARD_SPEED;

		// 在目标工作台附近
		if (distance < JUDGE_DISTANCE * 3) {
			lineSpeed = 1;
		}
		/*if (distance < JUDGE_DISTANCE * 4) {
			lineSpeed = 2;
		}
		else if (distance < JUDGE_DISTANCE * 2) {
			lineSpeed = 1;
		}*/

		if (map == 0)
		{
			if (abs(angleToTarget) > PI / 3) {
				lineSpeed = 0;
			}
			// 在墙边 且转弯角度大于60
			//if (isBesideBoundary() && abs(angleToTarget) > PI / 3) {
			//	lineSpeed = 0;
			//}
			//// 不在墙边 转弯角度大于90
			//if (abs(angleToTarget) > PI / 2) {
			//	lineSpeed = 0;
			//}
		}
		// 图3特化
		if (map == 3) {
			if (isBesideBoundary() && abs(angleToTarget) > PI / 3) {
				lineSpeed = 0;
			}
		}
		// 图4特化
		if (map == 4) {
			if (isBesideBoundary() && abs(angleToTarget) > PI / 3) {
				lineSpeed = 0;
			}
			if (distance < JUDGE_DISTANCE * 5 && abs(angleToTarget) > PI / 3) {
				lineSpeed = 0;
			}
		}



	}

	// 移动
	void move(Workbench& workbench, int map = 0) {
		int lineSpeed = 0;
		double angleSpeed = 0;
		calSpeed(workbench, lineSpeed, angleSpeed, map);
		this->forward(lineSpeed);
		this->rotate(angleSpeed);

		this->targetBenchId = workbench.getWorkbenchId();
		// 到达目标工作台
		if (this->workbenchId == this->targetBenchId) {
			// 买
			if (this->goodsType == 0) {
				// 产品还没生产好则等待
				if (workbench.getProductStatus() == 1) {
					this->buy();
					this->targetBenchId = -1;
					workbench.setProductStatus(0);
				}
			}
			// 卖
			else {
				// 材料格还没空出来则等待
				if (workbench.checkMaterialStatus(this->goodsType)) {
					this->sell();
					this->targetBenchId = -1;
					this->sellBenchId = -1;
					workbench.setReservedGoods(this->goodsType, false);
					workbench.setHoldGoods(this->goodsType, true);
				}

				// 特殊情况下在工作台附近无限等待 时间超过150帧就走
				if (this->waitSellFrame > 150) {
					cerr << "err:wait unfinitly" << endl;
					//this->sell();
					this->targetBenchId = -1;
					this->sellBenchId = -1;
					workbench.setReservedGoods(this->goodsType, false);
				}

			}
		}
	}

	// 令count在321循环
	inline int getCount() {
		if (this->count == 0) {
			this->count = 3;
			//this->count = 6;
		}
		return this->count--;
	}

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
	Robot(int _robotId) :robotId(_robotId) {
		workbenchId = -1;
		goodsType = 0;
		timeCoefficient = 0;
		collisionCoefficient = 0;
		angleSpeed = 0;
		lineSpeed[0] = 0;
		lineSpeed[1] = 0;
		direction = 0;
		coordinate[0] = 0;
		coordinate[1] = 0;

		targetBenchId = -1;
		sellBenchId = -1;
		waitFrame = 0;
		count = 3;
		waitSellFrame = 0;
	};
	~Robot() {};
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