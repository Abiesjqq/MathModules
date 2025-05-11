#include <bits/stdc++.h>
using namespace std;

struct task {
    char priority;
    int x, y, num;
};

struct candidate {
    vector<pair<int, int>> assignment;
    int cost;
};

struct position_coord {
    int x, y;
};

struct UAV {
    position_coord position;
    int remain_power;
};

struct obstacle {
    position_coord position;
    int radius;
    int obstacle_time;
};

const int max_gen = 100;  // 最大进化次数
const int pop_size = 60;  // 每次方案数量
const int INF = 4e10;
const int max_time = 600;
const int speed = 50;
const int max_dist = 1000;
const int min_dist = 50;
const int epsilon = 10;
const int time_step = 0.2;

vector<candidate> population;  // 方案组成的数组
vector<obstacle> obstacles;
vector<position_coord> targets;
int target_num;
int drone_num;

int main() {
    cin >> target_num >> drone_num;
    // int obstacle_x, obstacle_y, obstacle_time, obstacle_radius;
    // cin >> obstacle_x >> obstacle_y >> obstacle_time >> obstacle_radius;

    // 初始化population为{(1,1),(1,2)...,(1,n);...}
    for (int i = 1; i <= pop_size; i++) {
        candidate candi;
        for (int j = 1; j <= target_num; j++) {
            candi.assignment.push_back({1, j});
        }
        candi.cost = INF;
        population.push_back(candi);
    }

    // 进化过程
    for (int gen = 1; gen <= max_gen; gen++) {
        int cost_arr[pop_size] = {INF};

        // 计算每个方案的适应度,更新cost_arr数组
        for (int i = 0; i < pop_size; i++) {
            cost_arr[i] = calculateCost(population[i].assignment);
        }

        // 生成下一代

        // 第一位的交换,变异,第二位的交换
    }
}

// 生成min~max间随机整数
int generateRandom(int min, int max) {
    random_device rd;
    mt19937 gen(rd());

    uniform_int_distribution<> distrib(min, max);
    return distrib(gen);
}

// 第一位交换
int firstCross(candidate candi) {
    int ran1 = generateRandom(1, drone_num);
    int ran2 = generateRandom(1, drone_num);
    int uav_num1 = candi.assignment[ran1].first;
    int uav_num2 = candi.assignment[ran2].first;
    int tmp = uav_num1;
    uav_num1 = uav_num2;
    uav_num2 = tmp;
}

// 第一位突变
int firstMutation(candidate candi) {
    int ran1 = generateRandom(1, drone_num);
    int ran2 = generateRandom(1, drone_num);
    candi.assignment[ran1].first = ran2;
}

// 第二位交换
int secondCross(candidate candi) {
    int ran1 = generateRandom(1, drone_num);
    int ran2 = generateRandom(1, drone_num);
    int uav_num1 = candi.assignment[ran1].second;
    int uav_num2 = candi.assignment[ran2].second;
    int tmp = uav_num1;
    uav_num1 = uav_num2;
    uav_num2 = tmp;
}

int calculateCost(vector<pair<int, int>> assignment) {
    int cost = 0;
    double total_time = 0.0;
    // 初始化
    vector<position_coord> uav_locations;  // 表示t时刻各个无人机的位置,全零
    for (int i = 1; i <= drone_num; i++) {
        uav_locations.push_back((position_coord){0, 0});
    }
    int uav_powers[drone_num];
    for (int i = 0; i < drone_num; i++) {
        uav_powers[i] = max_time;
    }

    bool in_origin[drone_num] = {false};
    bool all_in_origin;

    // 所有无人机回到原点之前,不断更新无人机位置
    while (!all_in_origin) {
        total_time += time_step;

        vector<position_coord> new_uav_locations;
        // 更新uav_locations
        //

        // 更新总距离
        for (int i = 0; i < drone_num; i++) {
            // cost += calculateDistance(uav_locations[i],
            // new_uav_locations[i]);
            uav_locations = new_uav_locations;
        }

        // 更新每个无人机的飞行时间
        for (int i = 0; i < drone_num; i++) {
            if (!in_origin[i]) {
                uav_powers[i] -= time_step;
            }
        }
    }

    /*
    uav_locations 无人机数量*2矩阵,
    uav_powers大小为n的数组,表示各个无人机的电量
    利用swarm(粒子群)更新location,直到所有所有无人机返回原点
    用结束问题时的时间作为cost(?)
    */
}

const int swarm_gen = 60;
const int swarm_size = 30;
const double MAX_DRIFT;  // 最大随机偏移量
const double IBC;        // individual_best的调节效应
const double SBC;        // swarm_best的调节效应

// 根据输入的坐标求解速度
vector<double> swarm(vector<position_coord>) {
    vector<vector<double>> particle_swarm;
    vector<vector<double>> individual_best;
    vector<double> swarm_best;
    // 随机初始化particle_swarm. (0,\pi)
    for (int k = 0; k < swarm_gen; k++) {
        for (int i = 0; i < swarm_size; i++) {
            for (int j = 0; j < drone_num; j++) {
                particle_swarm[i][j] += rand() / RAND_MAX / MAX_DRIFT;
                particle_swarm[i][j] +=
                    (individual_best[i][j] - particle_swarm[i][j]) * IBC;
                particle_swarm[i][j] +=
                    (swarm_best[j] - particle_swarm[i][j]) * SBC;
            }
            // new_fitness=fitness()
        }
    }
}

// 计算两点之间的距离
double calculateDistance(position_coord pos1, position_coord pos2) {
    return sqrt(pow((pos1.x - pos2.x), 2) + pow((pos1.y - pos2.y), 2));
}

// 生成是否可移动的数组
vector<bool> detectMovable(vector<UAV> uavs) {
    vector<bool> movable;
    for (int i = 0; i < drone_num; i++) {
        movable.push_back(true);
    }
    for (int i = 0; i < drone_num; i++) {
        UAV uav = uavs[i];
        if (uav.remain_power > 0)
            continue;
        else if (uav.remain_power == 0) {
            for (int j = 0; j < i; j++) {
                UAV uav_j = uavs[j];
                if (uav_j.remain_power == 0) {
                    movable[i] = false;
                    break;
                }
            }
        } else {
            movable[i] = false;
        }
    }
    return movable;
}

// 判断最小最大距离是否合法
bool is_valid(vector<UAV> uavs, vector<obstacle> obstacles) {
    // 在原点的不考虑最小距离
    // 所有无人机都要考虑最大距离
    for (int i = 0; i < drone_num; i++) {
        for (int j = 0; j < drone_num; j++) {
            position_coord pos_i = uavs[i].position;
            position_coord pos_j = uavs[j].position;
            double cur_dist =
                sqrt(pow((pos_i.x - pos_j.x), 2) + pow((pos_i.y - pos_j.y), 2));

            if (cur_dist > max_dist)
                return false;
            if ((pos_i.x == 0 && pos_i.y == 0) ||
                (pos_j.x == 0 && pos_j.y == 0))
                continue;
            if (cur_dist < min_dist)
                return false;
        }
    }
    return true;
}

// 计算fitness
int calculate_fitness(candidate plan,
                      vector<double> angles,
                      vector<UAV>& uavs) {
    bool vis[target_num];  // 标记是否到达过
    for (int i = 0; i < target_num; i++)
        vis[i] = false;

    // 判断无人机是否可移动（是否在充电）
    vector<bool> movable = detectMovable(uavs);
    for (int i = 0; i < drone_num; i++) {
        position_coord position = uavs[i].position;
        int x = position.x;
        int y = position.y;
        movable[i] = (x == 0 && y == 0) ? false : true;
    }

    // 移动可移动的无人机
    for (int i = 0; i < drone_num; i++) {
        if (!movable[i])
            continue;
        position_coord position = uavs[i].position;
        double theta = angles[i];
        position.x += cos(theta) * speed * time_step;
        position.y += sin(theta) * speed * time_step;

        // 判断是否访问了某个目标点
        for (int i = 0; i < target_num; i++) {
            position_coord target_pos = targets[i];
            double cur_dist = sqrt(pow((position.x - target_pos.x), 2) +
                                   pow((position.y - target_pos.y), 2));
            if (cur_dist < epsilon)
                vis[i] = true;
        }
    }

    // 判断是否合法
    bool valid = is_valid(uavs, obstacles);
    if (!valid) {
        return INF;
    }

    int fitness;

    // 增加每个无人机到自己下一个目标的距离之和
    for (int i = 0; i < drone_num; i++) {  // 找每个无人机的下一个目标
        for (int j = 0; j < target_num; j++) {
            int target_num = plan.assignment[j].second;
            if (plan.assignment[j].first != i || vis[target_num])
                continue;
            fitness += calculateDistance(uavs[i].position, targets[target_num]);
        }
    }

    // 增加全局的下一个目标到对应的无人机的距离
    // 存疑(?)
    int unvisted_target;
    for (unvisted_target = 0; unvisted_target < target_num; unvisted_target++) {
        if (!vis[unvisted_target])
            break;
    }
    int uav_idx;
    for (pair<int, int> v : plan.assignment) {
        if (v.second == unvisted_target) {
            uav_idx = v.first;
            break;
        }
    }

    fitness +=
        calculateDistance(targets[unvisted_target], uavs[uav_idx].position);

    return fitness;
}
