#include "robot.h"
using namespace std;


// 计算机器人与工作台之间的距离
double Robot::calDistance(const Workbench& workbench) {
	double r_coor_x = this->getCoordinateX();
	double r_coor_y = this->getCoordinateY();
	double w_coor_x = workbench.getCoordinateX();
	double w_coor_y = workbench.getCoordinateY();
	double dx = w_coor_x - r_coor_x;
	double dy = w_coor_y - r_coor_y;
	double distance = sqrt(dx * dx + dy * dy);
	return distance;
}

// 计算机器人与机器人之间的距离
double Robot::calDistance(const Robot& robot) {
	double r_coor_x = this->getCoordinateX();
	double r_coor_y = this->getCoordinateY();
	double w_coor_x = robot.getCoordinateX();
	double w_coor_y = robot.getCoordinateY();
	double dx = w_coor_x - r_coor_x;
	double dy = w_coor_y - r_coor_y;
	double distance = sqrt(dx * dx + dy * dy);
	return distance;
}

// 计算两个向量之间的夹角（单位为弧度）
double Robot::angleBetween(double dx1, double dy1, double dx2, double dy2) {
	double len1 = sqrt(dx1 * dx1 + dy1 * dy1);
	double len2 = sqrt(dx2 * dx2 + dy2 * dy2);
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
double Robot::adjustDirection(double targetX, double targetY, double x, double y, double angle) {
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
double Robot::calAngleSpeed(const Workbench& workbench, double& angleToTarget) {
	double r_coor_x = this->getCoordinateX();
	double r_coor_y = this->getCoordinateY();
	double w_coor_x = workbench.getCoordinateX();
	double w_coor_y = workbench.getCoordinateY();
	angleToTarget = adjustDirection(w_coor_x, w_coor_y, r_coor_x, r_coor_y, this->getDirection());
	double angleSpeed = angleToTarget / DELTATIME / 6;
	return angleSpeed;
}

// 判断是否在墙边
bool Robot::isBesideBoundary() {
	double allow_distance = 2;
	if (this->getCoordinateX() < allow_distance || this->getCoordinateX() > 50 - allow_distance ||
		this->getCoordinateY() < allow_distance || this->getCoordinateY() > 50 - allow_distance) {
		return true;
	}
	return false;
}

// 计算在工作台范围内经过的帧数
void Robot::calWaitFrame(const Workbench& workbench) {
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
void Robot::calSpeed(const Workbench& workbench, int& lineSpeed, double& angleSpeed, int map) {
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
	double cur_speed = sqrt(this->lineSpeed[0] * this->lineSpeed[0] + this->lineSpeed[1] * this->lineSpeed[1]); 
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

	if (map == 0) {
		if (abs(angleToTarget) > PI / 3) {
			lineSpeed = 0;
		}
		//// 在墙边 且转弯角度大于60
		//if (isBesideBoundary() && abs(angleToTarget) > PI / 3) {
		//	lineSpeed = 0;
		//}
		//// 不在墙边 转弯角度大于90
		//if (abs(angleToTarget) > PI / 2) {
		//	lineSpeed = 0;
		//}
	}

	// 图2特化
	if (map == 2) {
		if (abs(angleToTarget) > PI / 3) {
			lineSpeed = 0;
		}
	}
	// 图1特化
	if (map == 1) {
		if (abs(angleToTarget) > PI / 3) {
			lineSpeed = 0;
		}
		if (distance < JUDGE_DISTANCE * 5 && abs(angleToTarget) > PI / 3) {
			lineSpeed = 0;
		}
	}
	// 图3特化
	if (map == 3) {
		if (isBesideBoundary() && abs(angleToTarget) > PI / 3) {
			lineSpeed = 0;
		}
		if (distance < JUDGE_DISTANCE * 5 && abs(angleToTarget) > PI / 3) {
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
void Robot::move(Workbench& workbench, int map) {
	int lineSpeed = 0;
	double angleSpeed = 0;
	calSpeed(workbench, lineSpeed, angleSpeed, map);
	this->forward(lineSpeed);
	this->rotate(angleSpeed);

	// 已到达目标工作台
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
			if (!workbench.getHoldMaterial(this->goodsType)) {
				this->sell();
				this->targetBenchId = -1;
				this->sellBenchId = -1;
				workbench.setReservedMaterial(this->goodsType, false);
				workbench.setHoldMaterial(this->goodsType, true);
			}

			// 特殊情况下在工作台附近无限等待 时间超过150帧就走
			if (this->waitSellFrame > 150) {
				cerr << "err:wait unfinitly" << endl;
				//this->sell();
				this->targetBenchId = -1;
				this->sellBenchId = -1;
				workbench.setReservedMaterial(this->goodsType, false);
			}

		}
	}
}

// 碰撞检测
void Robot::checkCollision(const vector<Robot>& robots, int map) {
	// 特殊情况下在墙边 不检测
	if (map == 3) {
		if (this->isBesideBoundary()) {
			return;
		}
	}
	double offset_angle = PI / 1.1;
	double cur_dirction[2]{ cos(this->getDirection()),sin(this->getDirection()) };
	double cur_coor[2]{ this->getCoordinateX(),this->getCoordinateY() };

	for (auto& robot : robots) {
		double distance = this->calDistance(robot);
		double check_distance = 4.5;
		double spped_check_distance = 2;
		double check_angle = PI / 8;
		//double ratio = distance > 5 ? 5 : distance;
		//double check_distance = RADUIS_FULL * 2.1 * ratio;
		//double check_angle = PI / 2 / ratio;
		//double check_angle = PI / 2 + 1.0 / 4 + 1 / (ratio - 5);
		//double check_angle = PI * 3 / 5 - ratio * PI / 10;

		if (robot.getRobotId() != this->robotId) {
			double other_coor[2]{ robot.getCoordinateX(),robot.getCoordinateY() };
			// 计算当前机器人朝向和目标位置之间的叉乘
			double dx = other_coor[0] - cur_coor[0];
			double dy = other_coor[1] - cur_coor[1];
			double dir[2]{ dx / sqrt(dx * dx + dy * dy), dy / sqrt(dx * dx + dy * dy) };
			double cross = cur_dirction[0] * dir[1] - cur_dirction[1] * dir[0]; // A×B为正 B在A的逆时针方向 否则顺时针方向 

			// 计算当前机器人朝向和目标位置之间的夹角
			double cos = cur_dirction[0] * dir[0] + cur_dirction[1] * dir[1];
			if (cos < -1) { cos = -1; }
			if (cos > 1) { cos = 1; }
			double between_angle = acos(cos);

			// 计算角度差
			double cur_angle = this->getDirection() > 0 ? this->getDirection() : this->getDirection() + 2 * PI;
			double other_angle = robot.getDirection() > 0 ? robot.getDirection() : robot.getDirection() + 2 * PI;
			double dif = abs(cur_angle - other_angle); // 角度差

			// 只检测当前朝向一定范围内的扇形区域
			if (between_angle < check_angle && distance < check_distance) {
				if (PI / 2 <= dif && dif < PI * 5 / 8) {
					this->rotate(-offset_angle);
				}
				else if (PI * 11 / 8 < dif && dif <= PI * 3 / 2) {
					this->rotate(offset_angle);
				}
				else if (PI * 5 / 8 <= dif && dif <= PI * 11 / 8)
				{
					if (cross > 0) {
						this->rotate(-offset_angle);
					}
					else {
						this->rotate(offset_angle);
					}
				}

				/*if (cross > 0) {
					this->rotate(-offset_angle);
				}
				else {
					this->rotate(offset_angle);
				}
				if (PI / 2  <= dif && dif < PI) {
					this->rotate(-offset_angle);
				}
				else if (PI <= dif && dif <= PI * 3 / 2 ) {
					this->rotate(offset_angle);
				}*/

				/*if (cross > 0) {
					if(PI /2 <=dif && dif <= PI)
					{
						this->rotate(-offset_angle);
					}
					else if(dif <= PI  && dif <= PI * 3/2) {
						this->rotate(offset_angle);
					}
				}
				else {
					if (PI  <= dif && dif <= PI * 3/2)
					{
						this->rotate(offset_angle);
					}
					else if(PI / 2 <= dif && dif <= PI) {
						this->rotate(-offset_angle);
					}
				}*/

				// 距离非常接近减速
				// TODO：范围调大一点
				if (distance < spped_check_distance) {
					this->forward(3);
				}
			}

			// 特殊情况下两个一直对向贴在一起
			if (distance < RADUIS_FULL * 2.1 && PI / 2 <= dif && dif < PI * 3 / 2) {
				this->rotate(offset_angle);
			}

			// 同时去墙边
			if (map == 1 || map == 2) {
				if (between_angle < PI / 3 && distance < RADUIS_FULL * 12 && robot.isBesideBoundary()) {
					this->forward(3);
					this->rotate(offset_angle / 2);
				}
			}
			// 同时去一个工作台
			if (map == 3) {
				if (between_angle < PI / 3 && distance < RADUIS_FULL * 12 && robot.getTargetBenchId() == this->getTargetBenchId()) {
					this->forward(3);
					this->rotate(offset_angle / 2);
				}
			}
		}
	}
}

// 令count循环
int Robot::getCount() {
	if (this->count == 4) {
		this->count = 1;
	}
	return this->count++;
}
int Robot::getCount12() {
	if (this->count == 3) {
		this->count = 1;
	}
	return this->count++;
}
int Robot::getCount23() {
	if (this->count == 1) {
		this->count++;
	}
	if (this->count == 4) {
		this->count = 2;
	}
	return this->count++;
}
int Robot::getCount13() {
	if (this->count == 5) {
		this->count = 1;
	}
	int count = this->count;
	this->count += 2;
	return count;
}

Robot::Robot(int _robotId) :robotId(_robotId) {
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
	waitSellFrame = 0;

	count = 1;
};

