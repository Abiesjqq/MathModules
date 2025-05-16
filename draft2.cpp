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

// const int max_gen = 100;  // 最大进化次数
// const int pop_size = 60;  // 每次方案数量
const int INF = 4e10;
const int max_time = 600;
const int speed = 50;
const int max_dist = 1000;
const int min_dist = 50;
const int epsilon = 10;
const int time_step = 0.2;

vector<candidate> population, temp_population;  // 方案组成的数组
vector<obstacle> obstacles;
vector<position_coord> targets;
vector<task> tasks;
vector<UAV> uavs;
int target_num;
int drone_num;

int main() {
    cin >> target_num >> drone_num;

    candidate best_candidate = GA();

    // 可视化best_candidiate
}

// 生成min~max间随机整数
int generateRandom(int min, int max) {
    random_device rd;
    mt19937 gen(rd());

    uniform_int_distribution<> distrib(min, max);
    return distrib(gen);
}

int calculateCost(vector<pair<int, int>> assignment) {
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

        // 更新uav_locations
        vector<double> angles = swarm(uavs, tasks, obstacles);
        vector<position_coord> new_uav_locations;
        // 移动可移动的无人机
        for (int i = 0; i < drone_num; i++) {
            position_coord position = uavs[i].position;
            double theta = angles[i];
            position.x += cos(theta) * speed * time_step;
            position.y += sin(theta) * speed * time_step;

            if (calculateDistance(position, (position_coord){0, 0}) < epsilon) {
                in_origin[i] = true;
            }
        }

        all_in_origin = true;
        for (int i = 0; i < drone_num; i++) {
            if (!in_origin[i])
                all_in_origin = false;
        }

        // 更新每个无人机的飞行时间
        for (int i = 0; i < drone_num; i++) {
            if (!in_origin[i]) {
                uav_powers[i] -= time_step;
            }
        }
    }

    return round(total_time);
}

// 下面临时赋值
const int swarm_gen = 60;
const int swarm_size = 30;
const double MAX_DRIFT = 0.8;  // 最大随机偏移量
const double IBC = 1.5;        // individual_best的调节效应
const double SBC = 1.5;        // swarm_best的调节效应

// 将任务分配给无人机，基于当前角度进行简单预测
candidate assign_tasks(const vector<UAV>& uavs,
                       const vector<task>& tasks,
                       const vector<double>& angles) {
    candidate plan;
    int drone_num = uavs.size();
    int target_num = tasks.size();

    // 所有无人机的当前位置
    vector<position_coord> drone_positions;
    for (const auto& uav : uavs) {
        drone_positions.push_back(uav.position);
    }

    // 预测无人机下一步位置（仅作为任务分配依据）
    vector<position_coord> predicted_positions(drone_num);
    for (int i = 0; i < drone_num; ++i) {
        double theta = angles[i];
        predicted_positions[i].x =
            drone_positions[i].x + cos(theta) * 100;  // 步长设为100
        predicted_positions[i].y = drone_positions[i].y + sin(theta) * 100;
    }

    // 分配任务：对每个任务，找离它最近的预测位置的无人机
    for (int t = 0; t < target_num; ++t) {
        int best_drone = -1;
        double min_dist = numeric_limits<double>::max();
        position_coord target_pos = {tasks[t].x, tasks[t].y};

        for (int d = 0; d < drone_num; ++d) {
            double dist = calculateDistance(predicted_positions[d], target_pos);
            if (dist < min_dist) {
                min_dist = dist;
                best_drone = d;
            }
        }

        plan.assignment.push_back({best_drone, tasks[t].num});
    }

    // 排序任务分配（按任务编号排序）
    sort(plan.assignment.begin(), plan.assignment.end(),
         [](const pair<int, int>& a, const pair<int, int>& b) {
             return a.second < b.second;
         });

    plan.cost = 0;  // 占位符，后续由 fitness 函数计算

    return plan;
}

// swarm函数：根据当前UAV状态和任务信息生成下一时刻的最优角度配置
vector<double> swarm(const vector<UAV>& initial_uavs,
                     const vector<task>& tasks,
                     const vector<obstacle>& obstacles) {
    int drone_num = initial_uavs.size();

    // 初始化粒子群
    vector<vector<double>> particle_swarm(swarm_size,
                                          vector<double>(drone_num));
    vector<vector<double>> velocity(swarm_size, vector<double>(drone_num, 0.0));

    // 每个粒子的个体最优
    vector<vector<double>> individual_best = particle_swarm;
    vector<int> best_fitness(swarm_size, INT_MAX);

    // 全局最优
    vector<double> swarm_best(drone_num);
    int global_best_fitness = INT_MAX;

    // 初始化粒子位置 (在 [0, π] 区间内)
    srand(time(0));
    for (int i = 0; i < swarm_size; ++i) {
        for (int j = 0; j < drone_num; ++j) {
            particle_swarm[i][j] = ((double)rand() / RAND_MAX) * M_PI;
        }
    }

    // 初始 fitness 计算
    for (int i = 0; i < swarm_size; ++i) {
        candidate plan = assign_tasks(initial_uavs, tasks, particle_swarm[i]);
        int fit = calculate_fitness(plan.assignment, particle_swarm[i],
                                    const_cast<vector<UAV>&>(initial_uavs));
        if (fit < best_fitness[i]) {
            best_fitness[i] = fit;
            individual_best[i] = particle_swarm[i];
        }
        if (fit < global_best_fitness) {
            global_best_fitness = fit;
            swarm_best = particle_swarm[i];
        }
    }

    // 迭代优化
    for (int gen = 0; gen < swarm_gen; ++gen) {
        for (int i = 0; i < swarm_size; ++i) {
            for (int j = 0; j < drone_num; ++j) {
                // 更新速度
                double r1 = (double)rand() / RAND_MAX;
                double r2 = (double)rand() / RAND_MAX;
                velocity[i][j] =
                    velocity[i][j] * 0.8 +
                    IBC * r1 * (individual_best[i][j] - particle_swarm[i][j]) +
                    SBC * r2 * (swarm_best[j] - particle_swarm[i][j]);

                // 更新位置
                particle_swarm[i][j] += velocity[i][j];

                // 限制角度范围 [0, π]
                if (particle_swarm[i][j] < 0)
                    particle_swarm[i][j] = 0;
                if (particle_swarm[i][j] > M_PI)
                    particle_swarm[i][j] = M_PI;
            }

            // 重新评估 fitness
            candidate plan =
                assign_tasks(initial_uavs, tasks, particle_swarm[i]);
            int fit = calculate_fitness(plan.assignment, particle_swarm[i],
                                        const_cast<vector<UAV>&>(initial_uavs));

            if (fit < best_fitness[i]) {
                best_fitness[i] = fit;
                individual_best[i] = particle_swarm[i];
            }
            if (fit < global_best_fitness) {
                global_best_fitness = fit;
                swarm_best = particle_swarm[i];
            }
        }
    }

    return swarm_best;  // 返回最优角度配置
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
        if (uav.remain_power > 0 && uav.remain_power < max_time)
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
        position_coord pos_i = uavs[i].position;

        for (int k = 0; k < obstacles.size(); k++) {
            if (calculateDistance(pos_i, obstacles[k].position) <
                obstacles[k].radius) {
                return false;
            }
        }

        for (int j = 0; j < drone_num; j++) {
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
int calculate_fitness(vector<pair<int, int>> assignment,
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
            int target_num = assignment[j].second;
            if (assignment[j].first != i || vis[target_num])
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
    for (pair<int, int> v : assignment) {
        if (v.second == unvisted_target) {
            uav_idx = v.first;
            break;
        }
    }

    fitness +=
        calculateDistance(targets[unvisted_target], uavs[uav_idx].position);

    return fitness;
}

/*-------------------------------------------------------------------*/

long long random_state;
void mysrand(long long _seed) {
    random_state = _seed;
}
int myrand() {
    random_state *= 900000011;
    random_state += 1122222223;
    return (int)(random_state & 0x000000007fffffff);
}

candidate GA(void) {
    // 参数设定
    int max_gen = 15 * drone_num + 50;
    int pop_size = 10 * drone_num;
    int reserved_candidates = drone_num;
    int unreserved = pop_size - reserved_candidates;
    int task_mutations = 3 * drone_num;
    int path_exchanges = 6 * drone_num;
    int sequence_exchanges = 3 * drone_num;

    // int demo_charges=2;//默认每架飞机充电次数
    // int charge_mutations=drone_num;

    // 初始化population
    mysrand(time(NULL));
    population.clear();
    population.resize(pop_size);  // 确保population有空间
    for (int i = 0; i < pop_size; i++) {
        candidate candi;
        for (int j = 1; j <= target_num; j++) {
            candi.assignment.push_back({myrand(), j});
        }
        candi.cost = INF;
        population[i] = candi;  // 或使用push_back()
    }

    // 初始化全局最优解
    candidate best_candidate;
    best_candidate.cost = INF;

    // 计算初始population的cost并排序
    for (int i = 0; i < pop_size; i++) {
        population[i].cost = calculateCost(population[i].assignment);
    }
    sort(
        population.begin(), population.end(),
        [](const candidate& a, const candidate& b) { return a.cost < b.cost; });
    best_candidate = population[0];  // 初始最优解

    // *****************进化过程*****************
    for (int gen = 1; gen <= max_gen; gen++) {
        // 更新temp_population
        temp_population.clear();           // 清除旧数据
        temp_population.resize(pop_size);  // 预分配空间

        // 计算适应度
        for (int i = 0; i < pop_size; i++) {
            population[i].cost = calculateCost(population[i].assignment);
        }

        // 排序
        sort(population.begin(), population.end(),
             [](const candidate& a, const candidate& b) {
                 return a.cost < b.cost;
             });

        // 更新全局最优解
        if (population[0].cost < best_candidate.cost) {
            best_candidate = population[0];
        }

        // 填充temp_population
        // 精英保留
        for (int i = 0; i < reserved_candidates; i++) {
            temp_population[i] = population[i];
        }

        // 锦标赛选择
        for (int i = reserved_candidates; i < pop_size; i++) {
            // 随机生成三个整数并取最小值（保留在a中）
            short a, b, c;
            a = myrand() % pop_size;
            b = myrand() % pop_size;
            c = myrand() % pop_size;
            if (a > b)
                a ^= b ^= a ^= b;
            if (b > c)
                b ^= c ^= b ^= c;
            if (a > b)
                a ^= b ^= a ^= b;
            temp_population[i] = population[a];
        }

        // 突变操作
        for (int i = 0; i < task_mutations; i++) {
            short ind = reserved_candidates + myrand() % unreserved;
            temp_population[ind].assignment[myrand() % target_num].first =
                myrand() % drone_num;
        }

        // 路径交换
        for (int i = 0; i < path_exchanges; i++) {
            // 1. 随机选择两个不同的非保留个体
            short ind1 = reserved_candidates + myrand() % unreserved;
            short ind2 = reserved_candidates + myrand() % unreserved;
            while (ind2 == ind1) {
                ind2 = reserved_candidates + myrand() % unreserved;
            }

            // 2. 随机选择一个区间 [start, end]
            int length = myrand() % (target_num) + 1;  // 区间长度（至少1）
            int start = myrand() % (target_num - length + 1);
            int end = start + length - 1;

            // 3. 在区间内交换两个个体的无人机编号（pair的第一个元素）
            for (int pos = start; pos <= end; ++pos) {
                swap(temp_population[ind1].assignment[pos].first,
                     temp_population[ind2].assignment[pos].first);
            }
        }

        // 顺序交换
        for (int i = 0; i < sequence_exchanges; i++) {
            short ind = reserved_candidates + myrand() % unreserved;
            int pos =
                myrand() % (target_num - 1);  // 随机选择相邻pair的起始位置
            // 交换相邻两个pair的目标编号（pair的第二个元素）
            auto& a = temp_population[ind].assignment;
            swap(a[pos].second, a[pos + 1].second);
        }

        // 更换种群
        population.swap(temp_population);
    }
    return best_candidate;
}