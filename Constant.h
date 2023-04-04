#ifndef CONSTANT_H_
#define CONSTANT_H_

constexpr auto FPS = 50;
constexpr auto JUDGE_DISTANCE = 0.4;
constexpr auto RADUIS_EMPTY = 0.45;
constexpr auto RADUIS_FULL = 0.53;
constexpr auto DENSITY = 20;
constexpr auto MAX_FORWARD_SPEED = 6;
constexpr auto MIN_FORWARD_SPEED = -2;
constexpr auto PI = 3.141592654;
constexpr auto MAX_ROTATE_SPEED = PI;
constexpr auto MIN_ROTATE_SPEED = -PI;
constexpr auto MAX_FORCE = 250;
constexpr auto MAX_TORQUE = 50;

constexpr auto DELTATIME = 0.02;
constexpr auto MASS_EMPTY = PI * RADUIS_EMPTY * RADUIS_EMPTY * DENSITY;
constexpr auto MASS_FULL = PI * RADUIS_FULL * RADUIS_FULL * DENSITY;
constexpr auto TOTAL_FRAME = 9000;

constexpr int ROBOT_SIZE = 4;
constexpr int GOODS_TYPE_SIZE = 7;
constexpr int WORKBENCH_TYPE_SIZE = 9;
constexpr int MAP_SIZE = 100;

constexpr double EPSILON = 1e-7;

#endif