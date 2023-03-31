#include "robot.h"
using namespace std;


// ����������빤��̨֮��ľ���
double Robot::calDistance(const Workbench& workbench) {
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
double Robot::calDistance(const Robot& robot) {
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
double Robot::angleBetween(double dx1, double dy1, double dx2, double dy2) {
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

// ������ٶ�
double Robot::calAngleSpeed(const Workbench& workbench, double& angleToTarget) {
	double r_coor_x = this->getCoordinateX();
	double r_coor_y = this->getCoordinateY();
	double w_coor_x = workbench.getCoordinateX();
	double w_coor_y = workbench.getCoordinateY();
	angleToTarget = adjustDirection(w_coor_x, w_coor_y, r_coor_x, r_coor_y, this->getDirection());
	double angleSpeed = angleToTarget / DELTATIME / 6;
	return angleSpeed;
}

// �ж��Ƿ���ǽ��
bool Robot::isBesideBoundary() {
	double allow_distance = 2;
	if (this->getCoordinateX() < allow_distance || this->getCoordinateX() > 50 - allow_distance ||
		this->getCoordinateY() < allow_distance || this->getCoordinateY() > 50 - allow_distance) {
		return true;
	}
	return false;
}

// �����ڹ���̨��Χ�ھ�����֡��
void Robot::calWaitFrame(const Workbench& workbench) {
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
void Robot::calSpeed(const Workbench& workbench, int& lineSpeed, double& angleSpeed, int map) {
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

	if (map == 0) {
		if (abs(angleToTarget) > PI / 3) {
			lineSpeed = 0;
		}
		//// ��ǽ�� ��ת��Ƕȴ���60
		//if (isBesideBoundary() && abs(angleToTarget) > PI / 3) {
		//	lineSpeed = 0;
		//}
		//// ����ǽ�� ת��Ƕȴ���90
		//if (abs(angleToTarget) > PI / 2) {
		//	lineSpeed = 0;
		//}
	}
	// ͼ1�ػ�
	if (map == 1) {
		if (abs(angleToTarget) > PI / 3) {
			lineSpeed = 0;
		}
		if (distance < JUDGE_DISTANCE * 5 && abs(angleToTarget) > PI / 3) {
			lineSpeed = 0;
		}
	}
	// ͼ3�ػ�
	if (map == 3) {
		if (isBesideBoundary() && abs(angleToTarget) > PI / 3) {
			lineSpeed = 0;
		}
		if (distance < JUDGE_DISTANCE * 5 && abs(angleToTarget) > PI / 3) {
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
void Robot::move(Workbench& workbench, int map) {
	int lineSpeed = 0;
	double angleSpeed = 0;
	calSpeed(workbench, lineSpeed, angleSpeed, map);
	this->forward(lineSpeed);
	this->rotate(angleSpeed);

	// �ѵ���Ŀ�깤��̨
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
			if (!workbench.getHoldMaterial(this->goodsType)) {
				this->sell();
				this->targetBenchId = -1;
				this->sellBenchId = -1;
				workbench.setReservedMaterial(this->goodsType, false);
				workbench.setHoldMaterial(this->goodsType, true);
			}

			// ����������ڹ���̨�������޵ȴ� ʱ�䳬��150֡����
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

// ��countѭ��
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

