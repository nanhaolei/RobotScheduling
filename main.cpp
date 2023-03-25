#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <queue>
#include <cmath>
#include <climits>
#include "Robot.cpp"
using namespace std;

vector<Workbench> workbenchs;
vector<Robot> robots;
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
int workbench_num_1 = 0;
int workbench_num_2 = 0;
int workbench_num_3 = 0;
int workbench_num_4 = 0;
int workbench_num_5 = 0;
int workbench_num_6 = 0;
int workbench_num_7 = 0;
int workbench_num_8 = 0;
int workbench_num_9 = 0;
int frameId = 0;
int map;

// 目标冲突检测
bool checkConflict(const int& robotId, const int& target_bench) {
    //bool isSelected = false;
    int count = 0;
    int type = workbenchs[target_bench].getType();
    // 89不做冲突检测
    if (type == 8 || type == 9) {
        return false;
    }
    for (auto& robot : robots) {
        // 目标相同且携带物品相同
        if (robot.getRobotId() != robotId &&
            robot.getTargetBenchId() == target_bench &&
            robot.getGoodsType() == robots[robotId].getGoodsType())
        {
            count++;
        }
    }
    // 工作台阻塞时允许有两个同时去
    if (workbenchs[target_bench].getRestFrame() == 0) {
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
    //int goods_type = robots[robotId].getGoodsType();
    vector<Workbench*> workbenchs_n;
    switch (goods_type)
    {
    case 1:
        if (workbench_num_4 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_4.begin(), workbenchs_4.end());
        if (workbench_num_5 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_5.begin(), workbenchs_5.end());
        if (workbench_num_9 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_9.begin(), workbenchs_9.end());
        break;
    case 2:
        if (workbench_num_4 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_4.begin(), workbenchs_4.end());
        if (workbench_num_6 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_6.begin(), workbenchs_6.end());
        if (workbench_num_9 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_9.begin(), workbenchs_9.end());
        break;
    case 3:
        if (workbench_num_5 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_5.begin(), workbenchs_5.end());
        if (workbench_num_6 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_6.begin(), workbenchs_6.end());
        if (workbench_num_9 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_9.begin(), workbenchs_9.end());
        break;
    case 4:
    case 5:
    case 6:
        if (workbench_num_7 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_7.begin(), workbenchs_7.end());
        if (workbench_num_9 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_9.begin(), workbenchs_9.end());
        break;
    case 7:
        if (workbench_num_8 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_8.begin(), workbenchs_8.end());
        if (workbench_num_9 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_9.begin(), workbenchs_9.end());
        break;
    default:
        std::cerr << "error:findSellBenchs" << endl;
        break;
    }
    return workbenchs_n;
}

// 计算卖出优先级
double calSellPriority(const double& distance, const int& workbenchId, const int& goodsType, double& actual_time, const bool& last = false) {
    vector<int> binary(8, 0);
    bool material_status = workbenchs[workbenchId].checkMaterialStatus(goodsType, binary);
    //bool material_status = !workbenchs[workbenchId].getHoldGoods(goodsType);
    
    // 计算几个材料格不空余
    int full_count = 0;
    for (int i = 0; i < 8; ++i) {
        if (binary[i] == 1) 
            ++full_count;
    }
    // 目标材料格不空余而且不在生产
    if (!material_status && workbenchs[workbenchId].getRestFrame() <= 0) {
        return INT_MAX;
    }
    // 检测预订目标冲突
    if (workbenchs[workbenchId].getReservedGoods(goodsType)) {
        return INT_MAX;
    }

    int bench_type = workbenchs[workbenchId].getType();
    double speed = MAX_FORWARD_SPEED;
    double offset = 1.2;
    double move_time = distance / speed * offset;
    double rest_time = workbenchs[workbenchId].getRestFrame() / static_cast<double>(FPS);

    // 降低123456卖到9的优先级
    if (bench_type == 9 && goodsType != 7 && workbench_num_7 > 0) {
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
        else if (full_count == 2 && workbenchs[workbenchId].getProductStatus() == 0 && (bench_type == 4 || bench_type == 5 || bench_type == 6)) {
            actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
            return actual_time;
        }
        else  if (full_count == 3 && workbenchs[workbenchId].getProductStatus() == 0 && bench_type == 7) {
            actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
            return actual_time;
        }
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
        else if (rest_time > 0 && full_count == 3 && workbenchs[workbenchId].getProductStatus() == 0) {
            actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
            return actual_time;
        }
    }

    // 456工作台
    else if ((bench_type == 4 || bench_type == 5 || bench_type == 6)) {
        // 图4特化
        if (map == 4) {
            double coefficient = 1.0;
            if (bench_type == 4) {
                coefficient *= 12; // 63w

            }
            if (bench_type == 5) {
                coefficient *= 2; // 63w
            }

            // 材料格空余 且 只缺一个
            if (full_count == 1) {
                coefficient *= 4; // 63w
            }

            // 材料格空余
            if (material_status) {
                actual_time = move_time;
                return move_time / coefficient;
            }
            // 正在生产 且 所有材料格都不空余 且 产品格没物品 
            else if (rest_time > 0 && full_count == 2 && workbenchs[workbenchId].getProductStatus() == 0) {
                actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
                return actual_time / coefficient;
            }
            return INT_MAX;
        }
        // 图2特化
        else if (map == 2) {
            double coefficient = 1.0;
            bool sell = material_status || (rest_time > 0 && full_count == 2 && workbenchs[workbenchId].getProductStatus() == 0);
            if (!sell) return INT_MAX;

            if (workbenchId == 0 || workbenchId == 22) {
                coefficient *= 2.5; // 85w
            }
            if (bench_type == 5) {
                coefficient *= 1.5; // 85w
            }
            if (bench_type == 4) {
                coefficient *= 1.2; // 85w
            }
            if (full_count == 1) {
                coefficient *= 2; // 85w
            }
            if (workbenchs[workbenchId].getProductStatus() || workbenchs[workbenchId].getRestFrame() > 0) {
                coefficient *= 2; // 85w
            }
            
            if (material_status) {
                actual_time = move_time;
                return move_time / coefficient;
            }
            else if (rest_time > 0 && full_count == 2 && workbenchs[workbenchId].getProductStatus() == 0) {
                actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
                return actual_time / coefficient;
            }
        }
        // 图3特化
        else if (map == 3) {
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
            if (rest_time > 0 && full_count == 2 && workbenchs[workbenchId].getProductStatus() == 0 ) {
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
                    if (bench->getRestFrame() < 0 && !bench->getReservedGoods(bench_type))
                    {
                        coefficient *= 8;
                    }
                }
            }
            // 材料格空余 且 只缺一个
            if (material_status && full_count == 1) {
                // 有7
                if (workbench_num_7 > 0) {
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
            else if (rest_time > 0 && full_count == 2 && workbenchs[workbenchId].getProductStatus() == 0) {
                actual_time = calAllowWaitTime(rest_time, move_time, bench_type);
                return actual_time / coefficient;
            }
        }
    }
    
    return INT_MAX;
}

// 寻找优先级最高的卖出工作台
int findSellBench(const int& robotId) {
    // 计算能收购该物品类型的工作台
    int goods_type = robots[robotId].getGoodsType();
    vector<Workbench*> workbenchs_n = findSellBenchs(goods_type);
    int size = workbenchs_n.size();

    // 计算优先级最高的工作台
    int target_bench = workbenchs_n[0]->getWorkbenchId();
    double min_time = INT_MAX;
    for (int i = 0; i < size; ++i) {
        int workbench_id = workbenchs_n[i]->getWorkbenchId();
        double distance = robots[robotId].calDistance(*workbenchs_n[i]);
        double actual_time;
        double time = calSellPriority(distance, workbench_id, robots[robotId].getGoodsType(), actual_time);

        if (time < min_time) {
            min_time = time;
            target_bench = workbench_id;
        }
        // 判断冲突
        //if (!checkConflict(robotId, workbench_id) && !workbenchs[workbench_id].getReservedGoods(goods_type)) {
        //    if (time < min_time) {
        //        min_time = time;
        //        target_bench = workbench_id;
        //    }
        //}

    }
    workbenchs[target_bench].setReservedGoods(goods_type, true);
    if (target_bench == -1) {
        std::cerr << "error:findSellBench" << endl;
    }
    return target_bench;
}

// 计算买入优先级
double calBuyPriority(const int& robotId, const int& workbenchId) {
    // 不在生产 并且 没有产品
    if (workbenchs[workbenchId].getRestFrame() < 0 && workbenchs[workbenchId].getProductStatus() == 0) {
        return INT_MAX;
    }
    // 目标冲突检测
    if (checkConflict(robotId, workbenchId)) {
        return INT_MAX;
    }

    int bench_type = workbenchs[workbenchId].getType();
    double distance = robots[robotId].calDistance(workbenchs[workbenchId]);
    double speed = MAX_FORWARD_SPEED;
    double offset = 1.1;
    double move_time = distance / speed * offset;
    double rest_time = workbenchs[workbenchId].getRestFrame() / static_cast<double>(FPS);

    // 当不位于4567工作台上时 降低购买优先级
    int threshold = 8000;
    if (map == 3) threshold = 8500;

    if ((bench_type == 4 || bench_type == 5 || bench_type == 6 || bench_type == 7) && frameId < threshold) {
        // 位于工作台上
        if (robots[robotId].getWorkbenchId() == workbenchId)
        {
            // 已经有产品
            if (workbenchs[workbenchId].getProductStatus() == 1 ) {
                return move_time;
            }
            // 正在生产
            else if (rest_time > 0) {
                return calAllowWaitTime(rest_time, move_time, bench_type);
            }
        }
        // 不位于工作台上
        else {
            if (bench_type == 7) {
                return INT_MAX;
            }
            else {
                if (workbenchs[workbenchId].getProductStatus() == 1 ) {
                    return move_time * 4; 
                }
                else if (rest_time > 0) {
                    return calAllowWaitTime(rest_time, move_time, bench_type) * 4; 
                }
                //return INT_MAX-1;
            }
        }
    }
    else {
        // 已经有产品
        if (workbenchs[workbenchId].getProductStatus() == 1) {

            // 7不在生产 且 7的该材料格没有被预订 
            /* for (auto bench7 : workbenchs_7) {
                if (bench7->getRestFrame() < 0 && !bench7->getReservedGoods(bench_type) &&
                    (bench_type == 4 || bench_type == 5 || bench_type == 6) && workbenchs[workbenchId].getProductStatus() == 1)
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
    }

    std::cerr << "framid:" << frameId << endl;
    std::cerr << "err:calBuyPriority" << endl;
    return INT_MAX;
}

// 寻找优先级最高的买入工作台
int findBuyBench(const int& robotId, const vector<Workbench*>& workbenchs_n) {
    int target_bench = -1;
    double min_time = INT_MAX;
    int num = workbenchs_n.size();
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
int findBuyBench(const int& robotId, const vector<Workbench>& workbenchs_n) {
    int target_bench = -1;
    double min_time = INT_MAX;
    int num = workbenchs_n.size();
    for (int i = 0; i < num; ++i) {
        int workbench_id = workbenchs_n[i].getWorkbenchId();
        double time = calBuyPriority(robotId, workbench_id);
        if (time < min_time) {
            min_time = time;
            target_bench = workbench_id;
        }
    }
    return target_bench;
}

// 给定目标工作台类型 计算买到它缺失的材料并带过去的最快路线
int findFastestBench(const int& robotId, const vector<Workbench*>& workbenchs_n) {
    double min_time = INT_MAX;
    int target_bench = -1;
    int sell_benchId = -1;
    vector<Workbench*>* buy_benchs = nullptr;
    // 遍历每个n号工作台 
    for (auto sell_bench : workbenchs_n) {
        // 若只缺一个材料 提高优先级
        double priority = 1;
        /*double priority = 1;
        vector<int> binary(8, 0);
        int ms = sell_bench->getMaterialStatus();
        int i = 0;
        while (ms > 0) {
            binary[i] = ms % 2;
            ms /= 2;
            ++i;
        }
        int full_count = 0;
        for (int i = 0; i < 8; ++i) {
            if (binary[i] == 1) ++full_count;
        }
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
            int buy_benchId = buy_bench->getWorkbenchId();
            double buy_time = calBuyPriority(robotId, buy_benchId);
            double offset = 1.1;
            double sell_time = buy_bench->calDistance(workbenchs[sell_bench->getWorkbenchId()]) / MAX_FORWARD_SPEED * offset * priority;
            //double actual_time;
            //double sell_time = calSellPriority(buy_bench->calDistance(workbenchs[sell_bench->getWorkbenchId()]), sell_bench->getWorkbenchId(), buy_bench->getType(), actual_time);
            // 计算总耗时最短的
            if (buy_time + sell_time < min_time) {
                min_time = buy_time + sell_time;
                target_bench = buy_benchId;
                sell_benchId = sell_bench->getWorkbenchId();
            }
        }
    }
    if (target_bench == -1) {
        std::cerr << "err:findBench:target_bench==-1" << endl;
    }
    robots[robotId].setSellBenchId(sell_benchId);
    workbenchs[sell_benchId].setReservedGoods(workbenchs[target_bench].getType(), true);
    return target_bench;
}

// 给定目标工作台类型 找最近的一个
int findClosetBench(const int& robotId, const vector<Workbench*>& workbenchs_n) {
    int target_bench = -1;
    double min_distance = INT_MAX;
    int num = workbenchs_n.size();
    for (int i = 0; i < num; ++i) {
        int workbench_id = workbenchs_n[i]->getWorkbenchId();
        double distance = robots[robotId].calDistance(workbenchs[workbench_id]);
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
        if (target_bench == -1)
        {
            // 找买入卖出总时间最短的一条路径 带合成所需材料回来
            target_bench = findFastestBench(robotId, workbenchs_4);
        }
        // 指定了买到后带回的工作台
        else if (sell_bench != -1) {
            robots[robotId].setSellBenchId(sell_bench);
            workbenchs[sell_bench].setReservedGoods(workbenchs[target_bench].getType(), true);
        }
        break;
    case 5:
        target_bench = findBuyBench(robotId, workbenchs_5);
        if (target_bench == -1)
        {
            target_bench = findFastestBench(robotId, workbenchs_5);
        }
        else if (sell_bench != -1) {
            robots[robotId].setSellBenchId(sell_bench);
            workbenchs[sell_bench].setReservedGoods(workbenchs[target_bench].getType(), true);
        }
        break;
    case 6:
        target_bench = findBuyBench(robotId, workbenchs_6);
        if (target_bench == -1)
        {
            target_bench = findFastestBench(robotId, workbenchs_6);
        }
        else if (sell_bench != -1) {
            robots[robotId].setSellBenchId(sell_bench);
            workbenchs[sell_bench].setReservedGoods(workbenchs[target_bench].getType(), true);
        }
        break;
    case 7:
        target_bench = findBuyBench(robotId, workbenchs_7);
        if (target_bench == -1)
        {
            // 判断最近的一个7号工作台缺哪个材料 去买这个材料
            workbenchId = findClosetBench(robotId, workbenchs_7);
            goods_type = workbenchs[workbenchId].getLostMaterial();
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
        double buy_time = calBuyPriority(robotId, buy_bench.getWorkbenchId());
        if (buy_time == INT_MAX) {
            continue; // 无法去该工作台买入
        }

        // 计算卖出时间
        int goods_type = buy_bench.getType();
        vector<Workbench*> sell_benchs = findSellBenchs(goods_type);
        for (auto sell_bench : sell_benchs) {
            double distance = buy_bench.calDistance(*sell_bench);
            double actual_time;
            double sell_time = calSellPriority(distance, sell_bench->getWorkbenchId(), goods_type, actual_time, last);
            if (sell_time == INT_MAX) {
                continue;
            }
            double total_time = buy_time + sell_time;
            double unit_profit = buy_bench.getProfit(actual_time) / total_time;
            if (unit_profit > max_unit_profit) {
                max_unit_profit = unit_profit;
                buy_bench_id = buy_bench.getWorkbenchId();
                sell_bench_id = sell_bench->getWorkbenchId();
            }
        }
    }
    /*if (buy_bench_id == -1 || sell_bench_id == -1) {
        std::cerr << "err:unitProfitFirst" << endl;
        std::cerr << "frameid:" << frameId << endl;
        std::cerr << "robotId:" << robotId << endl;
        std::cerr << "max_unit_profit:" << max_unit_profit << endl;
        std::cerr << "target_bench:" << buy_bench_id << endl;
        std::cerr << "sell_benchid:" << sell_bench_id << endl;
    }*/

    // 456全部卖不出去
    if (buy_bench_id == -1) {
        return -1;
    }

    robots[robotId].setSellBenchId(sell_bench_id);

    if (map == 3) {
        // 卖出目标不为8，9 设置预订标记
        if (workbenchs[sell_bench_id].getType() != 8 && workbenchs[sell_bench_id].getType() != 9) {
            workbenchs[sell_bench_id].setReservedGoods(workbenchs[buy_bench_id].getType(), true);
        }
    }
    else {
        workbenchs[sell_bench_id].setReservedGoods(workbenchs[buy_bench_id].getType(), true);
    }

    // 若更换目标 清除预订标记
    /*int old_sellbenchid = robots[robotId].getSellBenchId();
    if (old_sellbenchid != sell_benchid) {
        workbenchs[sell_benchid].setReservedGoods(workbenchs[robots[robotId].getTargetBenchId()].getType(), false);
    }
    robots[robotId].setSellBenchId(sell_benchid);
    workbenchs[sell_benchid].setReservedGoods(workbenchs[target_bench].getType(), true);*/
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
        goods = robots[robotId].getCount12();
        target_bench = findMaterial(robotId, goods, -1);
        for (auto bench : workbenchs_4) {
            if (bench->checkMaterialStatus(goods)) {
                robots[robotId].setSellBenchId(bench->getWorkbenchId());
                break;
            }
        }
    }
    if (robotId == 1) {
        goods = robots[robotId].getCount13();
        target_bench = findMaterial(robotId, goods, -1);
        for (auto bench : workbenchs_5) {
            if (bench->checkMaterialStatus(goods)) {
                robots[robotId].setSellBenchId(bench->getWorkbenchId());
                break;
            }
        }
    }
    if (robotId == 2) {
        goods = robots[robotId].getCount23();
        target_bench = findMaterial(robotId, goods, -1);
        for (auto bench : workbenchs_6) {
            if (bench->checkMaterialStatus(goods)) {
                robots[robotId].setSellBenchId(bench->getWorkbenchId());
                break;
            }
        }
    }
    if (robotId == 3) {
        if (frameId < 30) {
            return -1;
        }
        for (auto& bench : workbenchs) {
            int type = bench.getType();
            if (type == 7 && bench.getProductStatus()) {
                target_bench = bench.getWorkbenchId();
            }
            if ((type == 4 || type == 5 || type == 6) && (bench.getProductStatus() || (bench.getRestFrame() > 0 && bench.getRestFrame() < 100))) {
                target_bench = bench.getWorkbenchId();
            }
        }
        if (target_bench == -1) {
            target_bench = findMaterial(robotId, robots[robotId].getCount(), -1); // 123循环
        }
    }
    return target_bench;
}

// 固定买123
int fixPath2(const int& robotId) {
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
            int type = bench.getType();
            // 65w
            if ((type == 4 || type == 5 || type == 6) && (bench.getProductStatus() || (bench.getRestFrame() > 0 && bench.getRestFrame() < 80))) {
                target_bench = bench.getWorkbenchId();
                break;
            }
            else if (type == 7 && bench.getProductStatus()) {
                target_bench = bench.getWorkbenchId();
                break;
            }
        }
        if (target_bench == -1) {
            target_bench = findMaterial(robotId, robots[robotId].getCount(), -1); // 123循环
        }

    }
    return target_bench;
}

// 判断是否位于456工作台上 且该工作台只缺一个材料 返回该材料
int judge_1(const int& robotId) {
    int workbenchId = robots[robotId].getWorkbenchId();
    if (workbenchId == -1) {
        return 0;
    }
    int type = workbenchs[workbenchId].getType();
    int material = workbenchs[workbenchId].getMaterialStatus();
    int goods_type = 0;
    if (material != 0 && (type == 4 || type == 5 || type == 6)) {
        goods_type = workbenchs[workbenchId].getLostMaterial();
    }
    return goods_type;
}

// 判断是否 7号工作台没有目标是它 且 只缺一个材料 
bool judge_2(int& sell_bench, int& goods_type) {
    for (auto workbench : workbenchs_7) {
        // 判断是否只缺一个材料 若是获取该材料
        int material = workbench->getMaterialStatus();
        goods_type = 0;
        switch (material)
        {
            // 有45缺6
        case 48:
            goods_type = 6;
            break;
            // 有46缺5
        case 80:
            goods_type = 5;
            break;
            // 有56缺4
        case 96:
            goods_type = 4;
            break;
        default:
            goods_type = 0;
            break;
        }
        if (!workbench->getReservedGoods(goods_type) && goods_type != 0) {
            sell_bench = workbench->getWorkbenchId();
            return true;
        }
    }
    return false;
}

// 比较当前路径需要的总时间与剩余时间
bool compare_time(const int& robotId, const int& buy_bench_id) {
    int sell_bench_id = robots[robotId].getSellBenchId();
    if (buy_bench_id == -1 || sell_bench_id == -1) return false;

    double offset = 1.2;
    // 计算到买入工作台的时间
    double buy_distance = robots[robotId].calDistance(workbenchs[buy_bench_id]);
    double buy_time = buy_distance / MAX_FORWARD_SPEED * offset;
    // 从买入工作台到卖出工作台的时
    double sell_distance = workbenchs[buy_bench_id].calDistance(workbenchs[sell_bench_id]);
    double sell_time = sell_distance / MAX_FORWARD_SPEED * offset;
    int rest_frame = 9000 - frameId;
    // 剩余时间不足
    if (buy_time + sell_time > rest_frame / static_cast<double>(FPS)) {
        return true;
    }
    else {
        return false;
    }

    //int goods = workbenchs[buy_bench_id].getType();
    //if (goods == 7 || goods == 6 || goods == 5 || goods == 4) {
    //    // 计算卖掉的最短时间
    //    vector<Workbench*> workbenchs_n;
    //    double min_distance = INT_MAX;
    //    if (goods == 7)
    //    {
    //        if (workbench_num_8 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_8.begin(), workbenchs_8.end());
    //        if (workbench_num_9 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_9.begin(), workbenchs_9.end());
    //    }
    //    else {
    //        if (workbench_num_7 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_7.begin(), workbenchs_7.end());
    //        if (workbench_num_9 > 0) workbenchs_n.insert(workbenchs_n.end(), workbenchs_9.begin(), workbenchs_9.end());
    //    }
    //    for (auto sell_bench : workbenchs_n) {
    //        // 判断有空位
    //        /*if (sell_bench->checkMaterialStatus(goods)) {*/
    //        if (!sell_bench->getReservedGoods(goods)) {
    //            double sell_distance = workbenchs[buy_bench_id].calDistance(*sell_bench);
    //            if (sell_distance < min_distance) {
    //                min_distance = sell_distance;
    //                //sell_benchId = sell_bench->getWorkbenchId();
    //            }
    //        }
    //    }
    //    double sell_time = min_distance / MAX_FORWARD_SPEED * offset;
    //    int rest_frame = 9000 - frameId;
    //    // 需要的时间大于剩余时间
    //    if (buy_time + sell_time > rest_frame / static_cast<double>(FPS) && min_distance != INT_MAX) {
    //        return true;
    //    }
    //    else {
    //        return false;
    //    }
    //}
    //return false;
}

// 碰撞检测
// to do:有多个机器人在检测范围内 考虑最近的一个?
void checkCollision(const int& robotId) {
    // 特殊情况下在墙边 不检测
    /*if (robots[robotId].isBesideBoundary()) {
        return;
    }*/
    if (map == 3) {
        if (robots[robotId].isBesideBoundary()) {
            return;
        }
    }
    double offset_angle = PI / 1.1;
    double cur_dirction[2]{ cos(robots[robotId].getDirection()),sin(robots[robotId].getDirection()) };
    double cur_coor[2]{ robots[robotId].getCoordinateX(),robots[robotId].getCoordinateY() };

    for (auto& robot : robots) {
        double distance = robot.calDistance(robots[robotId]);
        double check_distance = 4.5;
        double spped_check_distance = 2;
        double check_angle = PI / 8;
        //double ratio = distance > 5 ? 5 : distance;
        //double check_distance = RADUIS_FULL * 2.1 * ratio;
        //double check_angle = PI / 2 / ratio;
        //double check_angle = PI / 2 + 1.0 / 4 + 1 / (ratio - 5);
        //double check_angle = PI * 3 / 5 - ratio * PI / 10;

        if (robot.getRobotId() != robotId) {
            double other_coor[2]{ robot.getCoordinateX(),robot.getCoordinateY() };
            // 计算当前机器人朝向和目标位置之间的叉乘
            double dx = other_coor[0] - cur_coor[0];
            double dy = other_coor[1] - cur_coor[1];
            double dir[2]{ dx / length(dx, dy) , dy / length(dx, dy) };
            double cross = cur_dirction[0] * dir[1] - cur_dirction[1] * dir[0]; // A×B为正 B在A的逆时针方向 否则顺时针方向 

            // 计算当前机器人朝向和目标位置之间的夹角
            double cos = cur_dirction[0] * dir[0] + cur_dirction[1] * dir[1];
            if (cos < -1) { cos = -1; }
            if (cos > 1) { cos = 1; }
            double between_angle = acos(cos);

            // 计算角度差
            double cur_angle = robots[robotId].getDirection() > 0 ? robots[robotId].getDirection() : robots[robotId].getDirection() + 2 * PI;
            double other_angle = robot.getDirection() > 0 ? robot.getDirection() : robot.getDirection() + 2 * PI;
            double dif = abs(cur_angle - other_angle); // 角度差

            // 只检测当前朝向一定范围内的扇形区域
            if (between_angle < check_angle && distance < check_distance) {

                if (PI / 2 <= dif && dif < PI * 5 / 8) {
                    // 顺时针转
                    robots[robotId].rotate(-offset_angle);
                }
                else if (PI * 11 / 8 < dif && dif <= PI * 3 / 2) {
                    // 逆时针转
                    robots[robotId].rotate(offset_angle);
                }
                else if (PI * 5 / 8 <= dif && dif <= PI * 11 / 8)
                {
                    if (cross > 0) {
                        // 顺时针转
                        robots[robotId].rotate(-offset_angle);
                    }
                    else {
                        robots[robotId].rotate(offset_angle);
                    }
                }

                /*if (cross > 0) {
                    robots[robotId].rotate(-offset_angle);
                }
                else {
                    robots[robotId].rotate(offset_angle);
                }
                if (PI / 2  <= dif && dif < PI) {
                    robots[robotId].rotate(-offset_angle);
                }
                else if (PI <= dif && dif <= PI * 3 / 2 ) {
                    robots[robotId].rotate(offset_angle);
                }*/

                /*if (cross > 0) {
                    if(PI /2 <=dif && dif <= PI)
                    {
                        robots[robotId].rotate(-offset_angle);
                    }
                    else if(dif <= PI  && dif <= PI * 3/2) {
                        robots[robotId].rotate(offset_angle);
                    }
                }
                else {
                    if (PI  <= dif && dif <= PI * 3/2)
                    {
                        robots[robotId].rotate(offset_angle);
                    }
                    else if(PI / 2 <= dif && dif <= PI) {
                        robots[robotId].rotate(-offset_angle);
                    }
                }*/

                // 距离非常接近减速
                if (distance < spped_check_distance) {
                    robots[robotId].forward(3);
                }
            }
            
            // 特殊情况下两个一直对向贴在一起
            if (distance < RADUIS_FULL * 2.1 && PI / 2 <= dif && dif < PI * 3 / 2) {
                robots[robotId].rotate(offset_angle);
            }
            
            // 图1特化 
            if (map == 1 || map == 2) {
                if (between_angle < PI / 3 && distance < RADUIS_FULL * 12 && robot.isBesideBoundary()) {
                    robots[robotId].forward(3);
                    robots[robotId].rotate(offset_angle / 2);
                }
            }
            // 图3
            if(map == 3) {
                if (between_angle < PI / 3 && distance < RADUIS_FULL * 12 && robot.getTargetBenchId() == robots[robotId].getTargetBenchId()) {
                    robots[robotId].forward(3);
                    robots[robotId].rotate(offset_angle / 2);
                }
            }
        }
    }
}

void action() {
    for (int robotId = 0; robotId < 4; robotId++) {
        int target_bench = robots[robotId].getTargetBenchId();
        if (target_bench == -1) {
            // 买入
            if (robots[robotId].getGoodsType() == 0) {
                //int sell_bench, goods_type;
                //int bench_id;

                // 若当前所在工作台产品格有产品 
                /*else if (curbenchId != -1 && workbenchs[curbenchId].getProductStatus() == 1) {
                    target_bench = curbenchId;
                }*/

                // 若7号工作台没有目标是它 且只缺一个材料 去买这个材料
                /*else if (judge_2(sell_bench, goods_type)) {
                    target_bench = findMaterial(robotId, goods_type, sell_bench);
                }*/

                // 若位于456工作台上 且该工作台缺一个材料 找这个材料带回来
                /*else if (( goods_type = judge_1(robotId) ) > 0) {
                    robots[robotId].setSellBenchId(workbenchs[curbenchId]);
                    target_bench = findMaterial(robotId, goods_type);
                }*/

                // 图1特化
                if (map == 1) {
                    target_bench = fixPath(robotId);
                }
                // 图3特化
                else if (map == 3) {
                    target_bench = unitProfitFirst(robotId);
                }
                // 图4特化
                else if (map == 4) {
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
                if (compare_time(robotId, target_bench)) {
                    target_bench = unitProfitFirst(robotId, true);
                    if (compare_time(robotId, target_bench)) {
                        target_bench = -1;
                    }
                }
            }
            // 卖出
            else {
                // 回到买入时指定的工作台
                if (robots[robotId].getSellBenchId() != -1) {
                    target_bench = robots[robotId].getSellBenchId();
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

        // 图1特化
        if (map == 1) {
            robots[robotId].move(workbenchs[target_bench], map);
        }
        // 图3特化
        else if (map == 3) {
            robots[robotId].move(workbenchs[target_bench], map);
        }
        // 图4特化
        else if (map == 4)
        {
            robots[robotId].move(workbenchs[target_bench], map);
        }
        else {
            robots[robotId].move(workbenchs[target_bench]);
        }
        checkCollision(robotId);
    }
}

bool readMap() {
    char line[1024];
    int workbenchId = 0;
    int robotId = 0;
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            workbench_num = workbenchs.size();
            return true;
        }
        for (int i = 0; i < 100; i++) {
            if (line[i] == 'A') {
                Robot r = Robot(robotId);
                ++robotId;
                robots.push_back(r);
            }
            else if (line[i] != '.') {
                Workbench wb = Workbench(workbenchId, line[i] - '0');
                ++workbenchId;
                workbenchs.push_back(wb);
            }
        }
    }
    return false;
}

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
            workbenchs[index].setType(type);
            workbenchs[index].setRestFrame(restFrame);
            workbenchs[index].setMaterialStatus(materialStatus);
            workbenchs[index].setProductStatus(productStatus);
            workbenchs[index].setCoordinateX(w_coordinate[0]);
            workbenchs[index].setCoordinateY(w_coordinate[1]);
        }
        else {
            ss >> workbenchId >> goodsType >> timeCoefficient >> collisionCoefficient >> angleSpeed >>
                lineSpeed[0] >> lineSpeed[1] >> direction >> r_coordinate[0] >> r_coordinate[1];
            robotId = index - workbench_num;
            robots[robotId].setWorkbenchId(workbenchId);
            robots[robotId].setGoodsType(goodsType);
            robots[robotId].setTimeCoefficient(timeCoefficient);
            robots[robotId].setCollisionCoefficient(collisionCoefficient);
            robots[robotId].setAngleSpeed(angleSpeed);
            robots[robotId].setLineSpeedX(lineSpeed[0]);
            robots[robotId].setLineSpeedY(lineSpeed[1]);
            robots[robotId].setDirection(direction);
            robots[robotId].setCoordinateX(r_coordinate[0]);
            robots[robotId].setCoordinateY(r_coordinate[1]);
        }
        ++index;
    }
    return false;
}

void classify() {
    Workbench* bench = nullptr;
    for (int i = 0; i < workbench_num; ++i) {
        switch (workbenchs[i].getType())
        {
        case 1:
            bench = &workbenchs[i];
            workbenchs_1.emplace_back(bench);
            break;
        case 2:
            bench = &workbenchs[i];
            workbenchs_2.emplace_back(bench);
            break;
        case 3:
            bench = &workbenchs[i];
            workbenchs_3.emplace_back(bench);
            break;
        case 4:
            bench = &workbenchs[i];
            workbenchs_4.emplace_back(bench);
            break;
        case 5:
            bench = &workbenchs[i];
            workbenchs_5.emplace_back(bench);
            break;
        case 6:
            bench = &workbenchs[i];
            workbenchs_6.emplace_back(bench);
            break;
        case 7:
            bench = &workbenchs[i];
            workbenchs_7.emplace_back(bench);
            break;
        case 8:
            bench = &workbenchs[i];
            workbenchs_8.emplace_back(bench);
            break;
        case 9:
            bench = &workbenchs[i];
            workbenchs_9.emplace_back(bench);
            break;
        default:
            break;
        }
    }
    workbench_num_1 = workbenchs_1.size();
    workbench_num_2 = workbenchs_2.size();
    workbench_num_3 = workbenchs_3.size();
    workbench_num_4 = workbenchs_4.size();
    workbench_num_5 = workbenchs_5.size();
    workbench_num_6 = workbenchs_6.size();
    workbench_num_7 = workbenchs_7.size();
    workbench_num_8 = workbenchs_8.size();
    workbench_num_9 = workbenchs_9.size();
}

int main() {
    // 初始化数据
    readMap();
    classify();
    puts("OK");
    fflush(stdout);
    int money;

    // 图1
    if (workbench_num_7 == 8) {
        map = 1;
    }
    // 图2
    if (workbench_num_7 == 2 && workbench_num_8 == 2) {
        map = 2;
    }
    // 图3
    if (workbench_num_7 == 0) {
        map = 3;
    }
    // 图4
    if (workbench_num_7 == 1 && workbench_num_8 == 1) {
        map = 4;
    }

    // 读取每帧输入信息
    while (scanf("%d %d", &frameId, &money) != EOF) {
        scanf("%d", &workbench_num);
        readFrame();

        // 输出控制指令
        printf("%d\n", frameId);

        action();
        //if (frameId > 50) action();
        printf("OK\n");
        fflush(stdout);
    }
    return 0;
}




//struct TempWorkbench {
//    int workbenchId;
//    double distance;
//    TempWorkbench(int id, double d) : workbenchId(id), distance(d) {};
//};
//struct compare {
//    bool operator() (const TempWorkbench& w1, const TempWorkbench& w2){
//        return w1.distance > w2.distance;
//    }
//};
//// 寻找目标类型工作台中 成品格有物品 且最近的四个
//vector<int> findBuyBench(const int& robotId, const vector<Workbench*>& workbenchs_n, const int& workbench_num) {
//    priority_queue<TempWorkbench, vector<TempWorkbench>, compare> closetWorkbenchs;
//    vector<int> target_benchs;
//    for (int i = 0; i < workbench_num; ++i) {
//        int workbenchId = workbenchs_n[i]->getWorkbenchId();
//        if (workbenchs_n[i]->getProductStatus() == 1) {
//            double distance = robots[robotId].calDistance(workbenchs[workbenchId]);
//            TempWorkbench workbench = TempWorkbench( workbenchId, distance );
//            if (closetWorkbenchs.size() < 4) {
//                closetWorkbenchs.push(workbench);
//            }
//            else if(distance < closetWorkbenchs.top().distance) {
//                closetWorkbenchs.pop();
//                closetWorkbenchs.push(workbench);
//            }
//        }
//    }
//    while (!closetWorkbenchs.empty()) {
//        target_benchs.emplace_back(closetWorkbenchs.top().workbenchId);
//        closetWorkbenchs.pop();
//    }
//    reverse(target_benchs.begin(), target_benchs.end());
//    return target_benchs;
//}
// 寻找不与其他机器人的目标冲突的一个工作台
//int findBenchWithoutConflict(vector<int>& target_benchs) {
//    int size = target_benchs.size();
//    int target_bench = target_benchs[0];
//    for (int index = 0; index < size; ++index) {
//        for (auto& robot : robots) {
//            if (robot.getTargetBenchId() == target_bench) {
//                target_bench = target_benchs[++index];
//                break;
//            }
//        }
//    }
//    return target_bench;
//}
