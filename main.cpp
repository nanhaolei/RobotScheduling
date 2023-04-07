#include <iostream>
#include <vector>
#include <sstream>
#include <cmath>
#include <climits>
#include <map>
#include "robot.h"
#include "graph.h"
#include "astar.h"
#include <algorithm>
#ifdef _WIN32
#include <ctime>
#else
#include <sys/time.h> // linux
#endif
using namespace std;
using namespace geometry;

vector<Workbench*> workbenchs;
vector<Robot*> robots;
vector<Workbench*> workbenchs_1;
vector<Workbench*> workbenchs_2;
vector<Workbench*> workbenchs_3;
vector<Workbench*> workbenchs_4;
vector<Workbench*> workbenchs_5;
vector<Workbench*> workbenchs_6;
vector<Workbench*> workbenchs_7;
vector<Workbench*> workbenchs_8;
vector<Workbench*> workbenchs_9;
int workbench_num = 0;
int frame_id = 0;
char map_data[MAP_SIZE][MAP_SIZE];
int cur_map = 0;
Graph* graph = new Graph();
map<array<int, 2>, vector<Node*>> all_path;

// 目标冲突检测
bool checkConflict(const int& robotId, const int& target_bench) {
    int count = 0;
    int type = workbenchs[target_bench]->getType();
    // 89不做冲突检测
    if (type == 8 || type == 9) {
        return false;
    }
    for (auto& robot : robots) {
        // 目标相同且携带物品相同
        if (robot->getRobotId() != robotId &&
            robot->getTargetBenchId() == target_bench &&
            robot->getGoodsType() == robots[robotId]->getGoodsType())
        {
            count++;
        }
    }
    // 工作台阻塞时允许有两个同时去
    if (workbenchs[target_bench]->getRestFrame() == 0) {
        if (count > 1) return true;
        else return false;
    }
    // 否则只允许一个去
    else {
        if (count > 0) return true;
        else return false;
    }
}

// 计算到目标工作台的时间是否在允许等待范围内
double calAllowWaitTime(double rest_time, double move_time, int bench_type) {
    double allow_wait_frame;
    // 生产时间大于移动时间
    if (rest_time > move_time)
    {
        // 123工作台允许等待10帧
        if (bench_type == 1 || bench_type == 2 || bench_type == 3) {
            allow_wait_frame = 10;
        }
        // 7工作台不等
        else if (bench_type == 7) {
            allow_wait_frame = 0;
        }
        // 456工作台允许等待10帧
        else {
            allow_wait_frame = 10;
        }
        // 在允许等待范围内
        if (rest_time - move_time <= allow_wait_frame / FPS) {
            return rest_time;
        }
        // 不在允许等待范围内
        else {
            return INT_MAX;
        }
    }
    // 生产时间小于移动时间
    else {
        return move_time;
    }
}

// 查找能收购该物品类型的工作台
vector<Workbench*> findSellBenchs(const int& goods_type) {
    vector<Workbench*> workbenchs_n;
    switch (goods_type)
    {
    case 1:
        if (workbenchs_4.size() > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_4.begin(), workbenchs_4.end());
        if (workbenchs_5.size() > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_5.begin(), workbenchs_5.end());
        if (workbenchs_9.size() > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_9.begin(), workbenchs_9.end());
        break;
    case 2:
        if (workbenchs_4.size() > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_4.begin(), workbenchs_4.end());
        if (workbenchs_5.size() > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_6.begin(), workbenchs_6.end());
        if (workbenchs_9.size() > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_9.begin(), workbenchs_9.end());
        break;
    case 3:
        if (workbenchs_5.size() > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_5.begin(), workbenchs_5.end());
        if (workbenchs_6.size() > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_6.begin(), workbenchs_6.end());
        if (workbenchs_9.size() > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_9.begin(), workbenchs_9.end());
        break;
    case 4:
    case 5:
    case 6:
        if (workbenchs_7.size() > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_7.begin(), workbenchs_7.end());
        if (workbenchs_9.size() > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_9.begin(), workbenchs_9.end());
        break;
    case 7:
        if (workbenchs_8.size() > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_8.begin(), workbenchs_8.end());
        if (workbenchs_9.size() > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_9.begin(), workbenchs_9.end());
        break;
    default:
        std::cerr << "error:findSellBenchs" << endl;
        break;
    }
    return workbenchs_n;
}

// 计算卖出优先级
double calSellPriority_old(const int& robotId, const double& distance, const int& workbenchId, const int& goods_type, double& actual_time, const bool& last = false) {
    //vector<int> binary(8, 0);
    //bool material_status = workbenchs[workbenchId]->checkMaterialStatus(goods_type, binary);
    bool material_status = !workbenchs[workbenchId]->getHoldMaterial(goods_type);
    int full_count = workbenchs[workbenchId]->getFullCount();
    // 目标材料格不空余而且不在生产
    if (!material_status && workbenchs[workbenchId]->getRestFrame() <= 0) {
        return INT_MAX;
    }
    // 检测预订目标冲突
    if (workbenchs[workbenchId]->getReservedMaterial(goods_type)) {
        return INT_MAX;
    }
    int bench_type = workbenchs[workbenchId]->getType();
    double speed = MAX_FORWARD_SPEED;
    double offset = 1.2;
    double move_time = distance / speed * offset;
    double rest_time = workbenchs[workbenchId]->getRestFrame() / static_cast<double>(FPS);

    // 降低123456卖到9的优先级
    if (bench_type == 9 && goods_type != 7 && workbenchs_7.size() > 0) {
        return INT_MAX - 1;
    }

    // 89工作台始终为运动时间
    if (bench_type == 8 || bench_type == 9) {
        actual_time = move_time;
        return move_time;
    }

    // 最后时刻只看时间最短的
    if (last) {
        if (material_status) {
            actual_time = move_time;
            return move_time;
        }
        else if (full_count == 2 && workbenchs[workbenchId]->getProductStatus() == 0 && (bench_type == 4 || bench_type == 5 || bench_type == 6)) {
            actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
            return actual_time;
        }
        else  if (full_count == 3 && workbenchs[workbenchId]->getProductStatus() == 0 && bench_type == 7) {
            actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
            return actual_time;
        }
        return INT_MAX;
    }

    // 7工作台
    if (bench_type == 7) {
        // 材料格空余
        if (material_status)
        {
            // 只缺一个 
            if (full_count == 2) {
                actual_time = move_time;
                return move_time / 6;
            }
            // 缺两个 
            if (full_count == 1) {
                actual_time = move_time;
                return move_time / 3;
            }
            // 缺三个
            if (full_count == 0) {
                actual_time = move_time;
                return move_time;
            }
        }
        // 正在生产 且 所有材料格都不空余 且 产品格没物品 
        else if (rest_time > 0 && full_count == 3 && workbenchs[workbenchId]->getProductStatus() == 0) {
            actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
            return actual_time;
        }
        return INT_MAX;
    }

    // 456工作台
    else if ((bench_type == 4 || bench_type == 5 || bench_type == 6)) {
        // 图4特化
        if (cur_map == 4) {
            double coefficient = 1.0;
            if (bench_type == 4) {
                coefficient *= 12; 

            }
            if (bench_type == 5) {
                coefficient *= 2; 
            }
            // 材料格空余 且 只缺一个
            if (full_count == 1) {
                coefficient *= 4; 
            }

            // 材料格空余
            if (material_status) {
                actual_time = move_time;
                return move_time / coefficient;
            }
            // 正在生产 且 所有材料格都不空余 且 产品格没物品 
            else if (rest_time > 0 && full_count == 2 && workbenchs[workbenchId]->getProductStatus() == 0) {
                actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
                return actual_time / coefficient;
            }
            return INT_MAX;
        }
        // 图2特化
        else if (cur_map == 2) {
            double coefficient = 1.0;
            bool sell = material_status || (rest_time > 0 && full_count == 2 && workbenchs[workbenchId]->getProductStatus() == 0);
            if (!sell) return INT_MAX;

            if (workbenchId == 0 || workbenchId == 22) {
                coefficient *= 2.5; 
            }
            if (bench_type == 5) {
                coefficient *= 1.5;
            }
            if (bench_type == 4) {
                coefficient *= 1.2; 
            }
            if (full_count == 1) {
                coefficient *= 2; 
            }
            if (workbenchs[workbenchId]->getProductStatus() || workbenchs[workbenchId]->getRestFrame() > 0) {
                coefficient *= 2; 
            }
            
            if (material_status) {
                actual_time = move_time;
                return move_time / coefficient;
            }
            else if (rest_time > 0 && full_count == 2 && workbenchs[workbenchId]->getProductStatus() == 0) {
                actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
                return actual_time / coefficient;
            }
        }
        // 图3特化
        else if (cur_map == 3) {
            // 456工作台材料格空余 且 只缺一个
            if (full_count == 1 && material_status) {
                actual_time = move_time;
                return move_time / 3;
            }
            // 目标材料格空余 缺一个以上
            if (material_status) {
                actual_time = move_time;
                return move_time;
            }
            // 正在生产 且 所有材料格都不空余 且 产品格没物品 
            if (rest_time > 0 && full_count == 2 && workbenchs[workbenchId]->getProductStatus() == 0 ) {
                actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
                return actual_time;
            }
            return INT_MAX;
        }
        
        else {
            double coefficient = 1.0;
            // 材料格空余 且 7没有该材料
            if (material_status) {
                for (auto bench : workbenchs_7) {
                    if (bench->getRestFrame() < 0 && !bench->getReservedMaterial(bench_type))
                    {
                        coefficient *= 8;
                    }
                }
            }
            // 材料格空余 且 只缺一个
            if (material_status && full_count == 1) {
                // 有7
                if (workbenchs_7.size() > 0) {
                    coefficient *= 4;
                }
                // 无7
                else {
                    coefficient *= 1.2;
                }
            }

            // 缺两个
            if (material_status) {
                actual_time = move_time;
                return move_time / coefficient;
            }
            // 正在生产 且 所有材料格都不空余 且 产品格没物品 
            else if (rest_time > 0 && full_count == 2 && workbenchs[workbenchId]->getProductStatus() == 0) {
                actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
                return actual_time / coefficient;
            }
            return INT_MAX;
        }
    }
    std::cerr << "frame:" << frame_id << endl;
    std::cerr << "err:calSellPriority" << endl;
    return INT_MAX;
}
double calSellPriority(const int& robotId, const double& distance, const int& workbenchId, const int& goods_type, double& actual_time, const bool& last = false) {
    bool material_status = !workbenchs[workbenchId]->getHoldMaterial(goods_type);
    int full_count = workbenchs[workbenchId]->getFullCount();
    // 目标材料格不空余而且不在生产
    if (!material_status && workbenchs[workbenchId]->getRestFrame() <= 0) {
        return INT_MAX;
    }
    // 检测预订目标冲突
    if (workbenchs[workbenchId]->getReservedMaterial(goods_type)) {
        return INT_MAX;
    }
    // 目标不可达
    if (!robots[robotId]->isReachable(workbenchId)) {
        return INT_MAX;
    }
    if (workbenchs[workbenchId]->getIsUnreachable()) {
        return INT_MAX;
    }

    int bench_type = workbenchs[workbenchId]->getType();
    double speed = MAX_FORWARD_SPEED;
    double offset = 1.2;
    double move_time = distance / speed * offset;
    double rest_time = workbenchs[workbenchId]->getRestFrame() / static_cast<double>(FPS);

    // 降低123456卖到9的优先级
    /*if (bench_type == 9 && goods_type != 7 && workbenchs_7.size() > 0) {
        return INT_MAX - 1;
    }*/
    // 降低123卖到9的优先级
    if (bench_type == 9 && (goods_type == 1 || goods_type == 2 || goods_type == 3 )) {
        return INT_MAX - 1;
    }

    // 89工作台始终为运动时间
    if (bench_type == 8 || bench_type == 9) {
        actual_time = move_time;
        return move_time;
    }

    // 最后时刻只看时间最短的
    if (last) {
        if (material_status) {
            actual_time = move_time;
            return move_time;
        }
        else if (full_count == 2 && workbenchs[workbenchId]->getProductStatus() == 0 && (bench_type == 4 || bench_type == 5 || bench_type == 6)) {
            actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
            return actual_time;
        }
        else  if (full_count == 3 && workbenchs[workbenchId]->getProductStatus() == 0 && bench_type == 7) {
            actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
            return actual_time;
        }
        return INT_MAX;
    }

    // 7工作台
    if (bench_type == 7) {
        double coefficient = 1.0;
        // 材料格空余
        if (material_status)
        {
            // 只缺一个 
            if (full_count == 2) {
                coefficient *= 2;
            }
            // 缺两个 
            if (full_count == 1) {
                coefficient *= 1;
            }
            // 缺三个
            if (full_count == 0) {
                coefficient *= 1;
            }
            actual_time = move_time;
            return move_time;
        }
        // 正在生产 且 所有材料格都不空余 且 产品格没物品 
        else if (rest_time > 0 && full_count == 3 && workbenchs[workbenchId]->getProductStatus() == 0) {
            actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
            return actual_time;
        }
        return INT_MAX;
    }

    // 456工作台
    else if ((bench_type == 4 || bench_type == 5 || bench_type == 6)) {
        double coefficient = 1.0;
        // 7没有该材料
        /*for (auto bench : workbenchs_7) {
            if (bench->getRestFrame() < 0 && !bench->getReservedMaterial(bench_type))
            {
                coefficient *= 2;
            }
        }*/
        // 材料格空余 且 只缺一个
        if (material_status && full_count == 1) {
            // 有7
            if (workbenchs_7.size() > 0) {
                coefficient *= 2;
            }
            // 无7
            else {
                coefficient *= 2;
            }
        }
        // 产品格有产品
        if (workbenchs[workbenchId]->getProductStatus() == 1) {
            coefficient *= 2;
        }

        // 材料格空余
        if (material_status) {
            actual_time = move_time;
            return move_time / coefficient;
        }
        // 正在生产 且 所有材料格都不空余 且 产品格没物品 
        else if (rest_time > 0 && full_count == 2 && workbenchs[workbenchId]->getProductStatus() == 0) {
            actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
            return actual_time / coefficient;
        }
        return INT_MAX;
        
    }
    std::cerr << "frame:" << frame_id << endl;
    std::cerr << "err:calSellPriority" << endl;
    return INT_MAX;
}

// 寻找优先级最高的卖出工作台
int findSellBench(const int& robotId) {
    // 计算能收购该物品类型的工作台
    int goods_type = robots[robotId]->getGoodsType();
    vector<Workbench*> workbenchs_n = findSellBenchs(goods_type);
    size_t size = workbenchs_n.size();

    // 计算优先级最高的工作台
    int target_bench = workbenchs_n[0]->getWorkbenchId();
    double min_time = INT_MAX;
    for (int i = 0; i < size; ++i) {
        int workbench_id = workbenchs_n[i]->getWorkbenchId();
        double distance = robots[robotId]->calDistance(*workbenchs_n[i]);
        double actual_time;
        double time = calSellPriority(robotId, distance, workbench_id, robots[robotId]->getGoodsType(), actual_time);

        if (time < min_time) {
            min_time = time;
            target_bench = workbench_id;
        }
        // 判断冲突
        //if (!checkConflict(robotId, workbench_id) && !workbenchs[workbench_id].getReservedMaterial(goods_type)) {
        //    if (time < min_time) {
        //        min_time = time;
        //        target_bench = workbench_id;
        //    }
        //}

    }
    workbenchs[target_bench]->setReservedMaterial(goods_type, true);
    if (target_bench == -1) {
        std::cerr << "error:findSellBench" << endl;
    }
    return target_bench;
}

// 计算买入优先级
double calBuyPriority_old(const int& robotId, const int& workbenchId) {
    // 不在生产 并且 没有产品
    if (workbenchs[workbenchId]->getRestFrame() < 0 && workbenchs[workbenchId]->getProductStatus() == 0) {
        return INT_MAX;
    }
    // 目标冲突检测
    if (checkConflict(robotId, workbenchId)) {
        return INT_MAX;
    }

    int bench_type = workbenchs[workbenchId]->getType();
    double distance = robots[robotId]->calDistance(*workbenchs[workbenchId]);
    double speed = MAX_FORWARD_SPEED;
    double offset = 1.1;
    double move_time = distance / speed * offset;
    double rest_time = workbenchs[workbenchId]->getRestFrame() / static_cast<double>(FPS);

    int threshold = 8000;
    if (cur_map == 3) threshold = 8500;
    // 当不位于4567工作台上时 降低购买优先级
    if ((bench_type == 4 || bench_type == 5 || bench_type == 6 || bench_type == 7) && frame_id < threshold) {
        // 位于工作台上
        if (robots[robotId]->getWorkbenchId() == workbenchId)
        {
            // 已经有产品
            if (workbenchs[workbenchId]->getProductStatus() == 1 ) {
                return move_time;
            }
            // 正在生产
            else if (rest_time > 0) {
                return calAllowWaitTime(rest_time, move_time, bench_type);
            }
            else {
                return INT_MAX;
            }
        }
        // 不位于工作台上
        else {
            if (bench_type == 7) {
                return INT_MAX;
            }
            else {
                if (workbenchs[workbenchId]->getProductStatus() == 1 ) {
                    return move_time * 4; 
                }
                else if (rest_time > 0) {
                    return calAllowWaitTime(rest_time, move_time, bench_type) * 4; 
                }
                else {
                    return INT_MAX;
                }
            }
        }
    }
    else {
        // 已经有产品
        if (workbenchs[workbenchId]->getProductStatus() == 1) {

            // 7不在生产 且 7的该材料格没有被预订 
            /* for (auto bench7 : workbenchs_7) {
                if (bench7->getRestFrame() < 0 && !bench7->getReservedMaterial(bench_type) &&
                    (bench_type == 4 || bench_type == 5 || bench_type == 6) && workbenchs[workbenchId]->getProductStatus() == 1)
                {
                    return move_time / 3;
                }
            }*/

            return move_time;
        }
        // 正在生产
        else if (rest_time > 0) {
            return calAllowWaitTime(rest_time, move_time, bench_type);
        }
        else {
            return INT_MAX;
        }
    }

    std::cerr << "err:calBuyPriority" << endl;
    return INT_MAX;
}
double calBuyPriority(const int& robotId, const int& workbenchId) {
    // 不在生产 并且 没有产品
    if (workbenchs[workbenchId]->getRestFrame() < 0 && workbenchs[workbenchId]->getProductStatus() == 0) {
        return INT_MAX;
    }
    // 目标冲突检测
    if (checkConflict(robotId, workbenchId)) {
        return INT_MAX;
    }
    // 目标不可达
    if (!robots[robotId]->isReachable(workbenchId)) {
        return INT_MAX;
    }
    if (workbenchs[workbenchId]->getIsUnreachable()) {
        return INT_MAX;
    }

    int bench_type = workbenchs[workbenchId]->getType();
    //distance = robots[robotId]->calDistance(*workbenchs[workbenchId]);
    double distance = 0;
    int cur_node = graph->coordinateToNode(robots[robotId]->getCoordinate())->id;
    int bench_node = graph->workbenchToNode(workbenchId)->id;
    if (all_path.find({ cur_node, bench_node }) != all_path.end()) {
        distance = all_path[{ cur_node, bench_node }].size() * 1.0;
    }
    else {
        distance = robots[robotId]->calDistance(*workbenchs[workbenchId]);
    }
    double speed = MAX_FORWARD_SPEED;
    double offset = 1.1;
    double move_time = distance / speed * offset;
    double rest_time = workbenchs[workbenchId]->getRestFrame() / static_cast<double>(FPS);


    // 当不位于4567工作台上时 降低购买优先级
    if ((bench_type == 4 || bench_type == 5 || bench_type == 6 || bench_type == 7) && frame_id < TOTAL_FRAME - 2000) {
        // 位于工作台上
        if (robots[robotId]->getWorkbenchId() == workbenchId)
        {
            // 已经有产品
            if (workbenchs[workbenchId]->getProductStatus() == 1) {
                return 0;
                //return move_time;
            }
            // 正在生产
            else if (rest_time > 0) {
                //return 0;
                return calAllowWaitTime(rest_time, move_time, bench_type);
            }
            else {
                return INT_MAX;
            }
        }
        // 不位于工作台上
        else {
            if (bench_type == 7) {
                return INT_MAX;
            }
            else {
                if (workbenchs[workbenchId]->getProductStatus() == 1) {
                    return move_time * 10;
                }
                else if (rest_time > 0) {
                    return calAllowWaitTime(rest_time, move_time, bench_type) * 10;
                }
                else {
                    return INT_MAX;
                }
            }
        }
    }
    else {
        // 已经有产品
        if (workbenchs[workbenchId]->getProductStatus() == 1) {

            // 7不在生产 且 7的该材料格没有被预订 
            /* for (auto bench7 : workbenchs_7) {
                if (bench7->getRestFrame() < 0 && !bench7->getReservedMaterial(bench_type) &&
                    (bench_type == 4 || bench_type == 5 || bench_type == 6) && workbenchs[workbenchId]->getProductStatus() == 1)
                {
                    return move_time / 3;
                }
            }*/

            return move_time;
        }
        // 正在生产
        else if (rest_time > 0) {
            return calAllowWaitTime(rest_time, move_time, bench_type);
        }
        else {
            return INT_MAX;
        }
    }

    std::cerr << "err:calBuyPriority" << endl;
    return INT_MAX;
}

// 寻找优先级最高的买入工作台
int findBuyBench(const int& robotId, const vector<Workbench*>& workbenchs_n) {
    int target_bench = -1;
    double min_time = INT_MAX;
    size_t num = workbenchs_n.size();
    for (int i = 0; i < num; ++i) {
        int workbench_id = workbenchs_n[i]->getWorkbenchId();
        double time = calBuyPriority(robotId, workbench_id);
        if (time < min_time) {
            min_time = time;
            target_bench = workbench_id;
        }
    }
    return target_bench;
}

// 给定目标工作台类型 计算买到它缺失的材料并带过去的最快路线
int findFastestPath(const int& robotId, const vector<Workbench*>& workbenchs_n) {
    double min_time = INT_MAX;
    int target_bench = -1;
    int sell_bench_id = -1;
    vector<Workbench*>* buy_benchs = nullptr;
    // 遍历每个n号工作台 
    for (auto sell_bench : workbenchs_n) {
        // 若只缺一个材料 提高优先级
        double priority = 1;
        /*int full_count = sell_bench->getFullCount();
        if (full_count > 0) {
            priority = 1.0 / 2;
        }*/

        // 判断缺哪个材料
        int lost_material = sell_bench->getLostMaterial();
        switch (lost_material)
        {
        case 1:
            buy_benchs = &workbenchs_1;
            break;
        case 2:
            buy_benchs = &workbenchs_2;
            break;
        case 3:
            buy_benchs = &workbenchs_3;
            break;
        case 4:
            buy_benchs = &workbenchs_4;
            break;
        case 5:
            buy_benchs = &workbenchs_5;
            break;
        case 6:
            buy_benchs = &workbenchs_6;
            break;
        default:
            std::cerr << "err:findBench" << endl;
            break;
        }
        // 遍历所有能买到该材料的工作台
        for (auto buy_bench : *buy_benchs) {
            int buy_bench_id = buy_bench->getWorkbenchId();
            double buy_time = calBuyPriority(robotId, buy_bench_id);
            double offset = 1.1;
            double sell_time = buy_bench->calDistance(*workbenchs[sell_bench->getWorkbenchId()]) / MAX_FORWARD_SPEED * offset * priority;
            //double actual_time;
            //double sell_time = calSellPriority(buy_bench->calDistance(workbenchs[sell_bench->getWorkbenchId()]), sell_bench->getWorkbenchId(), buy_bench->getType(), actual_time);
            // 计算总耗时最短的
            if (buy_time + sell_time < min_time) {
                min_time = buy_time + sell_time;
                target_bench = buy_bench_id;
                sell_bench_id = sell_bench->getWorkbenchId();
            }
        }
    }
    if (target_bench == -1) {
        std::cerr << "err:findFastestPath" << endl;
    }
    robots[robotId]->setSellBenchId(sell_bench_id);
    workbenchs[sell_bench_id]->setReservedMaterial(workbenchs[target_bench]->getType(), true);
    return target_bench;
}

// 给定目标工作台类型 找最近的一个
int findClosetBench(const int& robotId, const vector<Workbench*>& workbenchs_n) {
    int target_bench = -1;
    double min_distance = INT_MAX;
    size_t num = workbenchs_n.size();
    for (int i = 0; i < num; ++i) {
        int workbench_id = workbenchs_n[i]->getWorkbenchId();
        double distance = robots[robotId]->calDistance(*workbenchs[workbench_id]);
        if (distance < min_distance) {
            min_distance = distance;
            target_bench = workbench_id;
        }
    }
    return target_bench;
}

// 给定所需材料类型 找一个工作台去买
int findMaterial(const int& robotId, int goods_type, const int& sell_bench) {
    int target_bench = -1;
    int workbenchId;
    switch (goods_type)
    {
    case 1:
        target_bench = findBuyBench(robotId, workbenchs_1);
        if (target_bench == -1) {
            target_bench = workbenchs_1[0]->getWorkbenchId();
            std::cerr << "err:findMaterial" << endl;
        }
        break;
    case 2:
        target_bench = findBuyBench(robotId, workbenchs_2);
        if (target_bench == -1) {
            target_bench = workbenchs_2[0]->getWorkbenchId();
            std::cerr << "err:findMaterial" << endl;
        }
        break;
    case 3:
        target_bench = findBuyBench(robotId, workbenchs_3);
        if (target_bench == -1) {
            target_bench = workbenchs_3[0]->getWorkbenchId();
            std::cerr << "err:findMaterial" << endl;
        }
        break;
    case 4:
        target_bench = findBuyBench(robotId, workbenchs_4);
        // 该产品还没有被生产
        if (target_bench == -1) {
            // 找买入卖出总时间最短的一条路径 带合成所需材料回来
            target_bench = findFastestPath(robotId, workbenchs_4);
        }
        // 指定了买到后带回的工作台
        else if (sell_bench != -1) {
            robots[robotId]->setSellBenchId(sell_bench);
            workbenchs[sell_bench]->setReservedMaterial(workbenchs[target_bench]->getType(), true);
        }
        break;
    case 5:
        target_bench = findBuyBench(robotId, workbenchs_5);
        if (target_bench == -1) {
            target_bench = findFastestPath(robotId, workbenchs_5);
        }
        else if (sell_bench != -1) {
            robots[robotId]->setSellBenchId(sell_bench);
            workbenchs[sell_bench]->setReservedMaterial(workbenchs[target_bench]->getType(), true);
        }
        break;
    case 6:
        target_bench = findBuyBench(robotId, workbenchs_6);
        if (target_bench == -1) {
            target_bench = findFastestPath(robotId, workbenchs_6);
        }
        else if (sell_bench != -1) {
            robots[robotId]->setSellBenchId(sell_bench);
            workbenchs[sell_bench]->setReservedMaterial(workbenchs[target_bench]->getType(), true);
        }
        break;
    case 7:
        target_bench = findBuyBench(robotId, workbenchs_7);
        if (target_bench == -1) {
            // 判断最近的一个7号工作台缺哪个材料 去买这个材料带回来
            workbenchId = findClosetBench(robotId, workbenchs_7);
            goods_type = workbenchs[workbenchId]->getLostMaterial();
            target_bench = findMaterial(robotId, goods_type, workbenchId);
        }
        break;
    default:
        break;
    }

    return target_bench;
}

// 单位利润最高优先
int unitProfitFirst(const int& robotId, const bool& last = false) {
    double max_unit_profit = 0;
    int buy_bench_id = -1;
    int sell_bench_id = -1;
    for (auto& buy_bench : workbenchs) {
        // 计算买入时间
        double buy_time = calBuyPriority(robotId, buy_bench->getWorkbenchId());
        if (buy_time == INT_MAX) {
            continue; // 无法去该工作台买入
        }

        // 计算卖出时间
        int goods_type = buy_bench->getType();
        vector<Workbench*> sell_benchs = findSellBenchs(goods_type);
        for (auto sell_bench : sell_benchs) {
            double distance = 0;
            int buy_bench_node = graph->workbenchToNode(buy_bench_id)->id;
            int sell_bench_node = graph->workbenchToNode(sell_bench->getWorkbenchId())->id;
            if (all_path.find({ buy_bench_node,sell_bench_node }) != all_path.end()) {
                distance = all_path[{ buy_bench_node, sell_bench_node }].size()*1.0;
            }
            else {
                distance = buy_bench->calDistance(*sell_bench);
            }
            double actual_time;
            double sell_time = calSellPriority(robotId, distance, sell_bench->getWorkbenchId(), goods_type, actual_time, last);
            if (sell_time == INT_MAX) {
                continue;
            }
            double total_time = buy_time + sell_time;
            double unit_profit = buy_bench->getProfit(actual_time) / total_time;
            if (unit_profit > max_unit_profit) {
                max_unit_profit = unit_profit;
                buy_bench_id = buy_bench->getWorkbenchId();
                sell_bench_id = sell_bench->getWorkbenchId();
            }
        }
    }
    /*if (buy_bench_id == -1 || sell_bench_id == -1) {
        std::cerr << "err:unitProfitFirst" << endl;
        std::cerr << "frameid:" << frame_id << endl;
        std::cerr << "robotId:" << robotId << endl;
        std::cerr << "max_unit_profit:" << max_unit_profit << endl;
        std::cerr << "target_bench:" << buy_bench_id << endl;
        std::cerr << "sell_benchid:" << sell_bench_id << endl;
    }*/

    // 找不到能卖出去的物品
    if (buy_bench_id == -1) {
        return -1;
    }

    robots[robotId]->setSellBenchId(sell_bench_id);

    // 卖出目标不为8，9 设置预订标记
    if (workbenchs[sell_bench_id]->getType() != 8 && workbenchs[sell_bench_id]->getType() != 9) {
        workbenchs[sell_bench_id]->setReservedMaterial(workbenchs[buy_bench_id]->getType(), true);
    }
    
    return buy_bench_id;
}

// 高级物品优先
int seniorFirst(const int& robotId) {
    int target_bench = -1;
    int size = 0;
    // 优先买7    
    target_bench = findBuyBench(robotId, workbenchs_7);
    if (target_bench != -1) {
        return target_bench;
    }

    // 其次买456
    vector<Workbench*> workbenchs_456;
    workbenchs_456.insert(workbenchs_456.end(), workbenchs_4.begin(), workbenchs_4.end());
    workbenchs_456.insert(workbenchs_456.end(), workbenchs_5.begin(), workbenchs_5.end());
    workbenchs_456.insert(workbenchs_456.end(), workbenchs_6.begin(), workbenchs_6.end());
    target_bench = findBuyBench(robotId, workbenchs_456);
    if (target_bench != -1) {
        return target_bench;
    }

    // 最后买123
    vector<Workbench*> workbenchs_123;
    workbenchs_123.insert(workbenchs_123.end(), workbenchs_1.begin(), workbenchs_1.end());
    workbenchs_123.insert(workbenchs_123.end(), workbenchs_2.begin(), workbenchs_2.end());
    workbenchs_123.insert(workbenchs_123.end(), workbenchs_3.begin(), workbenchs_3.end());
    target_bench = findBuyBench(robotId, workbenchs_123);
    if (target_bench != -1) {
        return target_bench;
    }
    std::cerr << "err:seniorFirst" << endl;
    return target_bench;
}

// 最近优先
int closestFirst(const int& robotId) {
    int target_bench = -1;
    target_bench = findBuyBench(robotId, workbenchs);
    if (target_bench == -1) {
        std::cerr << "err:closestFirst" << endl;
    }
    return target_bench;
}

// 固定合456
int fixPath(const int& robotId) {
    int target_bench = -1;
    int goods;
    if (robotId == 0) {
        goods = robots[robotId]->getCount12();
        target_bench = findMaterial(robotId, goods, -1);
        for (auto bench : workbenchs_4) {
            if (!bench->getHoldMaterial(goods)) {
                robots[robotId]->setSellBenchId(bench->getWorkbenchId());
                break;
            }
        }
    }
    if (robotId == 1) {
        goods = robots[robotId]->getCount13();
        target_bench = findMaterial(robotId, goods, -1);
        for (auto bench : workbenchs_5) {
            if (!bench->getHoldMaterial(goods)) {
                robots[robotId]->setSellBenchId(bench->getWorkbenchId());
                break;
            }
        }
    }
    if (robotId == 2) {
        goods = robots[robotId]->getCount23();
        target_bench = findMaterial(robotId, goods, -1);
        for (auto bench : workbenchs_6) {
            if (!bench->getHoldMaterial(goods)) {
                robots[robotId]->setSellBenchId(bench->getWorkbenchId());
                break;
            }
        }
    }
    if (robotId == 3) {
        if (frame_id < 30) {
            return -1;
        }
        for (auto& bench : workbenchs) {
            int type = bench->getType();
            if (type == 7 && bench->getProductStatus()) {
                target_bench = bench->getWorkbenchId();
            }
            if ((type == 4 || type == 5 || type == 6) && (bench->getProductStatus() || (bench->getRestFrame() > 0 && bench->getRestFrame() < 100))) {
                target_bench = bench->getWorkbenchId();
            }
        }
        if (target_bench == -1) {
            target_bench = findMaterial(robotId, robots[robotId]->getCount(), -1); // 123循环
        }
    }
    return target_bench;
}

// 固定买123
int fixPath_2(const int& robotId) {
    int target_bench = -1;

    if (robotId == 0) {
        target_bench = findMaterial(robotId, 1, -1);
    }
    else if (robotId == 3) {
        target_bench = findMaterial(robotId, 3, -1);
    }
    else if (robotId == 2) {
        target_bench = findMaterial(robotId, 2, -1);
    }
    else if (robotId == 1) {
        for (auto& bench : workbenchs) {
            int type = bench->getType();
            if ((type == 4 || type == 5 || type == 6) && (bench->getProductStatus() || (bench->getRestFrame() > 0 && bench->getRestFrame() < 80))) {
                target_bench = bench->getWorkbenchId();
                break;
            }
            else if (type == 7 && bench->getProductStatus()) {
                target_bench = bench->getWorkbenchId();
                break;
            }
        }
        if (target_bench == -1) {
            target_bench = findMaterial(robotId, robots[robotId]->getCount(), -1); // 123循环
        }

    }
    return target_bench;
}

// 判断是否位于456工作台上 且该工作台只缺一个材料 返回该材料
int judge_1(const int& robotId) {
    int workbenchId = robots[robotId]->getWorkbenchId();
    if (workbenchId == -1) {
        return 0;
    }
    int type = workbenchs[workbenchId]->getType();
    int material = workbenchs[workbenchId]->getMaterialStatus();
    int goods_type = 0;
    if (material != 0 && (type == 4 || type == 5 || type == 6)) {
        goods_type = workbenchs[workbenchId]->getLostMaterial();
    }
    return goods_type;
}

// 比较当前路径需要的总时间与剩余时间
bool compareTime(const int& robotId, const int& buy_bench_id) {
    int sell_bench_id = robots[robotId]->getSellBenchId();
    if (buy_bench_id == -1 || sell_bench_id == -1) return false;

    double offset = 1.2;
    // 计算到买入工作台的时间
    double buy_distance = robots[robotId]->calDistance(*workbenchs[buy_bench_id]);
    double buy_time = buy_distance / MAX_FORWARD_SPEED * offset;
    // 从买入工作台到卖出工作台的时
    double sell_distance = workbenchs[buy_bench_id]->calDistance(*workbenchs[sell_bench_id]);
    double sell_time = sell_distance / MAX_FORWARD_SPEED * offset;
    int rest_frame = TOTAL_FRAME - frame_id;
    // 剩余时间不足
    if (buy_time + sell_time > rest_frame / static_cast<double>(FPS)) {
        return true;
    }
    else {
        return false;
    }
}

// 计算路径
vector<Vec2> calPath(Node* start, Node* goal) {
    vector<Node*> path;
    vector<Vec2> coor_path;
    if (all_path.find({ start->id,goal->id }) != all_path.end()) {
        path = all_path[{start->id, goal->id}];
        coor_path = AStar::getCoorPath(path);
    }
    else {
        AStar astar(start, goal);
        path = astar.searching();
        astar.smoothPath(path);
        // 记录路径
        all_path[{start->id, goal->id}] = path;
        for (auto neigh : start->neighbors) {
            all_path[{neigh->id, goal->id}] = path;
        }
        coor_path = AStar::getCoorPath(path);
        // 记录反向路径
        reverse(path.begin(), path.end());
        all_path[{ goal->id, start->id}] = path;
        for (auto neigh : goal->neighbors) {
            all_path[{neigh->id, start->id}] = path;
        }
    }
    //coor_path = AStar::getCoorPath(path);
    return coor_path;
}


// TODO：回退过程中与其他机器人的距离大于阈值 则停止回退
// 碰撞检测
void checkCollision() {
    //bool collision = false;
    for (int i = 0; i < ROBOT_SIZE; ++i) {
        robots[i]->setIsCollision(0);
    }
    for (int i = 0; i < ROBOT_SIZE; ++i) {
        auto& robotI = robots[i];
        if (robotI->getCollisionFrame() > 100) {
            Node* start = graph->coordinateToNode(robotI->getCoordinate());
            Node* goal = graph->coordinateToNode(robotI->getStartCoor());
            if (start->id == goal->id) {
                continue;
            }
            vector<Vec2> coor_path = calPath(start, goal);
            robotI->setPath(coor_path);
            for (int j = 0; j < ROBOT_SIZE; ++j) {
                robots[j]->setCollisionFrame(0);
            }
            return;
        }
        /*if (collision) 
            return;*/
        for (int j = i + 1; j < ROBOT_SIZE; ++j) {
            /*if (i == j) {
                continue;
            }
            auto& robotI = robots[i];*/
            auto& robotJ = robots[j];
            auto dis = robotI->calDistance(*robotJ);
            double r1 = robotI->getGoodsType() > 0 ? RADUIS_FULL : RADUIS_EMPTY;
            double r2 = robotJ->getGoodsType() > 0 ? RADUIS_FULL : RADUIS_EMPTY;
            double cur_angle = robotI->getDirection() > 0 ? robotI->getDirection() : robotI->getDirection() + 2 * PI;
            double other_angle = robotJ->getDirection() > 0 ? robotJ->getDirection() : robotJ->getDirection() + 2 * PI;
            double dif = abs(cur_angle - other_angle); // 角度差
            if (dis <= r1 + r2 + 0.02 && PI / 2 <= dif && dif < PI * 3 / 2) {
                robotI->addCollisionFrame();
                robotJ->addCollisionFrame();
                robotI->setIsCollision(1);
                robotJ->setIsCollision(1);
                /*collision = true;
                Robot* select;
                if ((robotI->getGoodsType() == 0 && robotJ->getGoodsType() == 0) || (robotI->getGoodsType() > 0 && robotJ->getGoodsType() > 0)) {
                    select = robotI->getPath().size() < robotJ->getPath().size() ? robotI : robotJ;
                }
                else if (robotI->getGoodsType() == 0) {
                    select = robotI;
                }
                else if (robotJ->getGoodsType() == 0) {
                    select = robotJ;
                }
                Node* start = graph->coordinateToNode(select->getCoordinate());
                Node* goal = graph->coordinateToNode(select->getStartCoor());
                vector<Vec2> coor_path = calPath(start, goal);
                select->setPath(coor_path);
                break;*/
            }
        }
    }
    for (int i = 0; i < ROBOT_SIZE; ++i) {
        if(robots[i]->getIsCollision() == 0)
            robots[i]->setCollisionFrame(0);
    }
}

void action_old() {
    for (int robotId = 0; robotId < ROBOT_SIZE; robotId++) {
        int target_bench = robots[robotId]->getTargetBenchId();
        if (target_bench == -1) {
            // 买入
            if (robots[robotId]->getGoodsType() == 0) {
                //int sell_bench, goods_type;
                //int bench_id;

                // 若当前所在工作台产品格有产品 
                /*else if (curbenchId != -1 && workbenchs[curbenchId].getProductStatus() == 1) {
                    target_bench = curbenchId;
                }*/

                // 若位于456工作台上 且该工作台缺一个材料 找这个材料带回来
                /*else if (( goods_type = judge_1(robotId) ) > 0) {
                    robots[robotId]->setSellBenchId(workbenchs[curbenchId]);
                    target_bench = findMaterial(robotId, goods_type);
                }*/

                // 图1特化
                if (cur_map == 1) {
                    target_bench = fixPath(robotId);
                }
                // 图3特化
                else if (cur_map == 3) {
                    target_bench = unitProfitFirst(robotId);
                }
                // 图4特化
                else if (cur_map == 4) {
                    if (robotId == 1) {
                        target_bench = findMaterial(1, 4, 0);
                    }
                    else {
                        target_bench = unitProfitFirst(robotId);
                    }
                    //target_bench = unitProfitFirst(robotId);
                }
                else {
                    target_bench = unitProfitFirst(robotId);
                }

                // 剩余时间不足以买4567并卖掉
                if (compareTime(robotId, target_bench)) {
                    target_bench = unitProfitFirst(robotId, true);
                    if (compareTime(robotId, target_bench)) {
                        target_bench = -1;
                    }
                }
            }
            // 卖出
            else {
                // 回到买入时指定的工作台
                if (robots[robotId]->getSellBenchId() != -1) {
                    target_bench = robots[robotId]->getSellBenchId();
                }
                // 没有指定
                else {
                    target_bench = findSellBench(robotId);
                }
            }
        }
        if (target_bench == -1) {
            continue;
        }
        robots[robotId]->setTargetBenchId(target_bench);
        robots[robotId]->move_old(*workbenchs[target_bench], cur_map);
        robots[robotId]->checkCollision_old(robots, cur_map);
    }
}
void action() {
    for (auto& robot : robots) {
        int target_bench = robot->getTargetBenchId();
        // 计算目标
        if (target_bench == -1) {
            // 买入
            if (robot->getGoodsType() == 0) {
                target_bench = unitProfitFirst(robot->getRobotId());

                // 剩余时间不足以买4567并卖掉
                /*if (compareTime(robotId, target_bench)) {
                    target_bench = unitProfitFirst(robotId, true);
                    if (compareTime(robotId, target_bench)) {
                        target_bench = -1;
                    }
                }*/
            }
            // 卖出
            else {
                target_bench = robot->getSellBenchId();
                // 回到买入时指定的工作台
                //if (robot->getSellBenchId() != -1) {
                //    target_bench = robot->getSellBenchId();
                //}
                //// 没有指定
                //else {
                //    target_bench = findSellBench(robot->getRobotId());
                //}
            }
            if (target_bench == -1) {
                continue;
            }
            robot->setTargetBenchId(target_bench);
            robot->setTargetBench(workbenchs[target_bench]);
        }
        // 计算路径
        if (robot->getPath().empty() && target_bench != -1) {
            Node* start;
            if (robot->getWorkbenchId() == -1) {
                Vec2 start_coor = robot->getCoordinate();
                start = graph->coordinateToNode(start_coor);
            }
            else {
                start = graph->workbenchToNode(robot->getWorkbenchId());
            }
            Node* goal = graph->workbenchToNode(target_bench);
            vector<Vec2> coor_path = calPath(start, goal);
            robot->setPath(coor_path);
        }
        robot->move();
        //robot->checkCollision(robots);
        
    }
    checkCollision();
}

// 读取每帧数据
bool readFrame() {
    char line[1024];
    int index = 0;
    int robotId = 0;
    int type = 0, restFrame = 0, materialStatus = 0, productStatus = 0;
    double w_coordinate[2]{ 0,0 };
    int workbenchId = 0, goodsType = 0;
    double timeCoefficient = 0, collisionCoefficient = 0, angleSpeed = 0, lineSpeed[2]{ 0,0 }, direction = 0, r_coordinate[2]{ 0,0 };
    fgets(line, sizeof line, stdin); // 吸收掉一个换行符
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        std::stringstream ss(line);
        if (index < workbench_num) {
            ss >> type >> w_coordinate[0] >> w_coordinate[1] >> restFrame >> materialStatus >> productStatus;
            workbenchs[index]->setType(type);
            workbenchs[index]->setRestFrame(restFrame);
            workbenchs[index]->setMaterialStatus(materialStatus);
            workbenchs[index]->setProductStatus(productStatus);
            workbenchs[index]->setCoordinateX(w_coordinate[0]);
            workbenchs[index]->setCoordinateY(w_coordinate[1]);
        }
        else {
            ss >> workbenchId >> goodsType >> timeCoefficient >> collisionCoefficient >> angleSpeed >>
                lineSpeed[0] >> lineSpeed[1] >> direction >> r_coordinate[0] >> r_coordinate[1];
            robotId = index - workbench_num;
            robots[robotId]->setWorkbenchId(workbenchId);
            robots[robotId]->setGoodsType(goodsType);
            robots[robotId]->setTimeCoefficient(timeCoefficient);
            robots[robotId]->setCollisionCoefficient(collisionCoefficient);
            robots[robotId]->setAngleSpeed(angleSpeed);
            robots[robotId]->setLineSpeedX(lineSpeed[0]);
            robots[robotId]->setLineSpeedY(lineSpeed[1]);
            robots[robotId]->setDirection(direction);
            robots[robotId]->setCoordinateX(r_coordinate[0]);
            robots[robotId]->setCoordinateY(r_coordinate[1]);
        }
        ++index;
    }
    return false;
}

// 读取地图数据
void readMap() {
    char line[1024];
    int workbenchId = 0;
    int robotId = 0;
    int row = 0;
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            workbench_num = workbenchs.size();
            graph->init(map_data);
            return;
        }
        for (int col = 0; col < MAP_SIZE; col++) {
            map_data[row][col] = line[col];

            double x = 0.25 + col * 0.5;
            double y = 0.25 + (MAP_SIZE - 1 - row) * 0.5;
            if (line[col] == 'A') {
                Robot *r = new Robot(robotId);
                r->setCoordinateX(x);
                r->setCoordinateY(y);
                ++robotId;
                robots.push_back(r);
            }
            else if (line[col] >= '1' && line[col] <= '9') {
                Workbench *wb = new Workbench(workbenchId, line[col] - '0');
                wb->setCoordinateX(x);
                wb->setCoordinateY(y);
                ++workbenchId;
                workbenchs.push_back(wb);
            }
        }
        ++row;
    }
}

// 工作台分类
void classify() {
    Workbench* bench = nullptr;
    for (int i = 0; i < workbench_num; ++i) {
        switch (workbenchs[i]->getType())
        {
        case 1:
            bench = workbenchs[i];
            workbenchs_1.emplace_back(bench);
            break;
        case 2:
            bench = workbenchs[i];
            workbenchs_2.emplace_back(bench);
            break;
        case 3:
            bench = workbenchs[i];
            workbenchs_3.emplace_back(bench);
            break;
        case 4:
            bench = workbenchs[i];
            workbenchs_4.emplace_back(bench);
            break;
        case 5:
            bench = workbenchs[i];
            workbenchs_5.emplace_back(bench);
            break;
        case 6:
            bench = workbenchs[i];
            workbenchs_6.emplace_back(bench);
            break;
        case 7:
            bench = workbenchs[i];
            workbenchs_7.emplace_back(bench);
            break;
        case 8:
            bench = workbenchs[i];
            workbenchs_8.emplace_back(bench);
            break;
        case 9:
            bench = workbenchs[i];
            workbenchs_9.emplace_back(bench);
            break;
        default:
            break;
        }
    }
}

// 判断当前地图
void judgeMap() {
    //// 图1
    //if (workbenchs_7.size() == 8) {
    //    cur_map = 1;
    //}
    //// 图2
    //if (workbenchs_7.size() == 2 && workbenchs_8.size() == 2) {
    //    cur_map = 2;
    //}
    //// 图3
    //if (workbenchs_7.size() == 0) {
    //    cur_map = 3;
    //}
    //// 图4
    //if (workbenchs_7.size() == 1 && workbenchs_8.size() == 1) {
    //    cur_map = 4;
    //}

    if (workbenchs_9.size() == 4) {
        cur_map = 1;
    }
    if (workbenchs_8.size() == 1 && workbenchs_9.size() == 2) {
        cur_map = 3;
    }
}

// 初始化买工作台到卖工作台的路径
void initPath() {
#ifdef _WIN32
    clock_t start_time, end_time;
    start_time = clock();
#else
    struct timeval t1, t2;
    gettimeofday(&t1, nullptr);
#endif
    for (auto buy_bench : workbenchs) {
        if (buy_bench->getType() == 8 || buy_bench->getType() == 9 || buy_bench->getIsUnreachable()) {
            continue;
        }
        Node* start = graph->workbenchToNode(buy_bench->getWorkbenchId());
        auto workbenchs_n = findSellBenchs(buy_bench->getType());
        for (auto sell_bench : workbenchs_n) {
            if (sell_bench->getIsUnreachable()) {
                continue;
            }
            Node* goal = graph->workbenchToNode(sell_bench->getWorkbenchId());
            calPath(start, goal);
        }
#ifdef _WIN32
        end_time = clock();
        double usetime_ms = static_cast<double>(end_time - start_time) / CLOCKS_PER_SEC * 1000;
#else
        gettimeofday(&t2, nullptr);
        double usetime_ms = (t2.tv_sec - t1.tv_sec) * 1000 + static_cast<double>(t2.tv_usec - t1.tv_usec) / 1000;
#endif
        if (usetime_ms > 4000) {
            break;
        }
    }
}

// 初始化不可达情况(计算机器人初始位置与所有工作台的路径)
void initUnreachable() {
    /*for (auto& robot : robots) {
        for (auto& workbench : workbenchs)
        {
            Vec2 start_coor = robot->getCoordinate();
            Node* start = graph->coordinateToNode(start_coor);
            Node* goal = graph->workbenchToNode(workbench->getWorkbenchId());
            AStar astar(start, goal);
            vector<Node*> path = astar.searching();
            if (path.size() == 0) {
                robot->addUnreachableBench(workbench->getWorkbenchId());
            }
            else {
                all_path[{start->id, goal->id}] = path;
            }
        }
    }*/

    if (cur_map == 1) {
        vector<int> unreachable = { 1,2,3,4,5,6,7,8,12,13,29,30,31,32,33,34,35,36 };
        for (auto bench_id : unreachable) {
            /*for (auto& robot : robots) {
                robot->addUnreachableBench(bench_id);
            }*/
            workbenchs[bench_id]->setIsUnreachable(true);
        }

    }
    if (cur_map == 3) {
        for (auto bench : workbenchs) {
            robots[3]->addUnreachableBench(bench->getWorkbenchId());
        }
    }
}

// 初始化
void init() {
    readMap();
    classify();
    judgeMap();
    initUnreachable();
    initPath();
    puts("OK");
    fflush(stdout);
}

int main() {
    init();
    // 读取每帧输入信息
    int money;
    while (scanf("%d %d", &frame_id, &money) != EOF) {
        scanf("%d", &workbench_num);
        readFrame();

        // 输出控制指令
        printf("%d\n", frame_id);

        action();
        //return 0;
        
        printf("OK\n");
        fflush(stdout);
    }
    return 0;
}
