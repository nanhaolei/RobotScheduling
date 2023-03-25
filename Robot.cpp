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

	// ����������빤��̨֮��ľ���
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

	// ����������������֮��ľ���
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

	// ������������֮��ļнǣ���λΪ���ȣ�
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

	// ������Ҫת�ĽǶȴ�С
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

	// ������ٶ�
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

	// �ж��Ƿ���ǽ��
	inline bool isBesideBoundary() {
		double allow_distance = 2;
		if (this->getCoordinateX() < allow_distance || this->getCoordinateX() > 50 - allow_distance ||
			this->getCoordinateY() < allow_distance || this->getCoordinateY() > 50 - allow_distance) {
			return true;
		}
		return false;
	}

	// �����ڹ���̨��Χ�ھ�����֡��
	void calWaitFrame(const Workbench& workbench) {
		double distance = calDistance(workbench);
		// �ڷ�Χ�ڼ�ʱ
		if (distance < JUDGE_DISTANCE * 2) {
			++this->waitFrame;
		}
		// �����ڴ�Χ�ڵ���ʱ��
		if (distance < JUDGE_DISTANCE * 4) {
			++this->waitSellFrame;
		}
		// ����������Χ ����ǽ�� ��Ŀ���� ������ʱ��
		if (distance > JUDGE_DISTANCE * 4 || isBesideBoundary() || this->targetBenchId != workbench.getWorkbenchId()) {
			this->waitFrame = 0;
			this->waitSellFrame = 0;
		}
	}

	// �����ٶ�
	void calSpeed(const Workbench& workbench, int& lineSpeed, double& angleSpeed, int map = 0) {
		// �����ڹ���̨��Χ�ڵĵȴ�ʱ�� ��������ȴ�ʱ���ó�λ��
		int allow_frame = 80;
		calWaitFrame(workbench);
		if (this->waitFrame > allow_frame) {
			lineSpeed = MIN_FORWARD_SPEED;
			angleSpeed = 0;
			return;
		}

		// �����˶��ٶ�
		double distance = calDistance(workbench);
		double cur_speed = length(this->lineSpeed[0], this->lineSpeed[1]);
		double angleToTarget;
		angleSpeed = calAngleSpeed(workbench, angleToTarget);
		lineSpeed = MAX_FORWARD_SPEED;

		// ��Ŀ�깤��̨����
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
			// ��ǽ�� ��ת��Ƕȴ���60
			//if (isBesideBoundary() && abs(angleToTarget) > PI / 3) {
			//	lineSpeed = 0;
			//}
			//// ����ǽ�� ת��Ƕȴ���90
			//if (abs(angleToTarget) > PI / 2) {
			//	lineSpeed = 0;
			//}
		}
		// ͼ3�ػ�
		if (map == 3) {
			if (isBesideBoundary() && abs(angleToTarget) > PI / 3) {
				lineSpeed = 0;
			}
		}
		// ͼ4�ػ�
		if (map == 4) {
			if (isBesideBoundary() && abs(angleToTarget) > PI / 3) {
				lineSpeed = 0;
			}
			if (distance < JUDGE_DISTANCE * 5 && abs(angleToTarget) > PI / 3) {
				lineSpeed = 0;
			}
		}



	}

	// �ƶ�
	void move(Workbench& workbench, int map = 0) {
		int lineSpeed = 0;
		double angleSpeed = 0;
		calSpeed(workbench, lineSpeed, angleSpeed, map);
		this->forward(lineSpeed);
		this->rotate(angleSpeed);

		this->targetBenchId = workbench.getWorkbenchId();
		// ����Ŀ�깤��̨
		if (this->workbenchId == this->targetBenchId) {
			// ��
			if (this->goodsType == 0) {
				// ��Ʒ��û��������ȴ�
				if (workbench.getProductStatus() == 1) {
					this->buy();
					this->targetBenchId = -1;
					workbench.setProductStatus(0);
				}
			}
			// ��
			else {
				// ���ϸ�û�ճ�����ȴ�
				if (workbench.checkMaterialStatus(this->goodsType)) {
					this->sell();
					this->targetBenchId = -1;
					this->sellBenchId = -1;
					workbench.setReservedGoods(this->goodsType, false);
					workbench.setHoldGoods(this->goodsType, true);
				}

				// ����������ڹ���̨�������޵ȴ� ʱ�䳬��150֡����
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

	// ��count��321ѭ��
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