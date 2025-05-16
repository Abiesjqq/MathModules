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

const int INF = 4e5;
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

int generateRandom(int min, int max);
double generateRandomDouble(double min, double max);
int calculateCost(vector<pair<int, int>> assignment);
vector<double> swarm(const vector<UAV>& initial_uavs,
                     const vector<task>& tasks,
                     const vector<pair<int, int>>& assignment,
                     const vector<obstacle>& obstacles);
double calculateDistance(position_coord pos1, position_coord pos2);
vector<bool> detectMovable(vector<UAV> uavs);
bool is_valid(vector<UAV> uavs, vector<obstacle> obstacles);
int calculate_fitness(vector<pair<int, int>> assignment,
                      vector<double> angles,
                      vector<UAV>& uavs);
candidate GA(void);

// 生成min~max间随机整数
int generateRandom(int min, int max) {
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> distrib(min, max);
    return distrib(gen);
}

double generateRandomDouble(double min, double max) {
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> distrib(min, max);
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
        vector<double> angles = swarm(uavs, tasks, assignment, obstacles);
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

// swarm函数：根据当前UAV状态和任务信息生成下一时刻的最优角度配置
vector<double> swarm(const vector<UAV>& initial_uavs,
                     const vector<task>& tasks,
                     const vector<pair<int, int>>& assignment,
                     const vector<obstacle>& obstacles) {
    int drone_num = initial_uavs.size();
    vector<vector<double>> particle_swarm(swarm_size,
                                          vector<double>(drone_num));
    vector<vector<double>> velocity(swarm_size, vector<double>(drone_num, 0.0));
    vector<vector<double>> individual_best = particle_swarm;
    vector<int> best_fitness(swarm_size, INT_MAX);
    vector<double> swarm_best(drone_num);
    int global_best_fitness = INT_MAX;

    // 初始化角度
    for (int i = 0; i < swarm_size; ++i) {
        for (int j = 0; j < drone_num; ++j) {
            particle_swarm[i][j] = generateRandomDouble(0, M_PI);
        }
    }

    // 初始 fitness 评估
    for (int i = 0; i < swarm_size; ++i) {
        vector<UAV> uav_copy = initial_uavs;
        int fit = calculate_fitness(assignment, particle_swarm[i], uav_copy,
                                    tasks, obstacles);
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
                double r1 = generateRandomDouble(0, 1);
                double r2 = generateRandomDouble(0, 1);
                velocity[i][j] =
                    velocity[i][j] * 0.8 +
                    IBC * r1 * (individual_best[i][j] - particle_swarm[i][j]) +
                    SBC * r2 * (swarm_best[j] - particle_swarm[i][j]);
                particle_swarm[i][j] += velocity[i][j];
                particle_swarm[i][j] =
                    max(0.0, min(M_PI, particle_swarm[i][j]));
            }

            vector<UAV> uav_copy = initial_uavs;
            int fit = calculate_fitness(assignment, particle_swarm[i], uav_copy,
                                        tasks, obstacles);
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

    return swarm_best;
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
int calculate_fitness(const vector<pair<int, int>>& assignment,
                      const vector<double>& angles,
                      const vector<UAV>& initial_uavs,
                      const vector<task>& tasks,
                      const vector<obstacle>& obstacles) {
    // 深拷贝 UAV 状态以进行模拟
    vector<UAV> uavs = initial_uavs;
    int n_drones = uavs.size();
    int n_targets = tasks.size();

    // 标记每个目标是否已被访问
    vector<bool> visited(n_targets, false);

    double total_time = 0.0;
    bool all_visited = false;

    // 存储每架无人机的任务队列
    vector<vector<int>> task_queues(n_drones);
    for (const auto& assign : assignment) {
        int drone_id = assign.first;
        int task_id = assign.second - 1;  // 假设任务编号从1开始
        if (task_id >= 0 && task_id < n_targets)
            task_queues[drone_id].push_back(task_id);
    }

    // 循环模拟直到所有目标都被访问或超时
    while (!all_visited && total_time < max_time * 2) {
        total_time += time_step;

        // 更新无人机位置
        for (int i = 0; i < n_drones; ++i) {
            UAV& uav = uavs[i];
            if (uav.remain_power <= 0)
                continue;

            // 如果还有任务，继续向目标移动
            if (!task_queues[i].empty()) {
                int target_id = task_queues[i][0];
                position_coord target_pos = targets[target_id];

                // 判断是否已到达目标点
                double dist = calculateDistance(uav.position, target_pos);
                if (dist < epsilon) {
                    visited[target_id] = true;
                    task_queues[i].erase(task_queues[i].begin());
                    continue;
                }

                // 否则朝目标方向飞行（这里也可以用 angle）
                double dx = target_pos.x - uav.position.x;
                double dy = target_pos.y - uav.position.y;
                double theta = atan2(dy, dx);

                uav.position.x += cos(theta) * speed * time_step;
                uav.position.y += sin(theta) * speed * time_step;

                uav.remain_power -= time_step;
            }
        }

        // 检查是否所有目标都已完成
        all_visited = true;
        for (bool v : visited) {
            if (!v) {
                all_visited = false;
                break;
            }
        }

        // 检查是否发生碰撞
        for (int i = 0; i < n_drones; ++i) {
            for (int j = i + 1; j < n_drones; ++j) {
                double dist =
                    calculateDistance(uavs[i].position, uavs[j].position);
                if (dist < min_dist)
                    return INF;  // 碰撞惩罚
            }
            for (const auto& obs : obstacles) {
                double dist = calculateDistance(uavs[i].position, obs.position);
                if (dist < obs.radius)
                    return INF;  // 障碍物惩罚
            }
        }
    }

    // Fitness = 总时间 + 未完成任务数量 × 大数
    int unvisited_count = count(visited.begin(), visited.end(), false);
    int fitness = round(total_time) + unvisited_count * 10000;

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
    // 在 GA() 中初始化 population 的地方
    for (int i = 0; i < pop_size; i++) {
        candidate candi;
        // 添加普通任务
        for (int j = 1; j <= target_num; j++) {
            candi.assignment.push_back({myrand() % drone_num, j});
        }
        // 添加充电任务，数量为 drone_num（每机至少一次）
        for (int k = 0; k < drone_num; k++) {
            int charge_task_id =
                target_num + 1 + k;  // 假设充电任务编号从 target_num+1 开始
            candi.assignment.push_back(
                {k, charge_task_id});  // 第 k 架无人机第 k 次充电
        }
        candi.cost = INF;
        population[i] = candi;
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

int main() {
    cin >> target_num >> drone_num;

    // 读取目标点 targets
    targets.resize(target_num);
    for (int i = 0; i < target_num; ++i) {
        int x, y;
        int dummy_power;  // 目标没有电量，但输入可能有占位符
        cin >> targets[i].x >> targets[i].y >> dummy_power;
    }

    // 读取无人机初始状态
    uavs.resize(drone_num);
    for (int i = 0; i < drone_num; ++i) {
        int x, y;
        int power;
        cin >> x >> y >> power;
        uavs[i].position.x = x;
        uavs[i].position.y = y;
        uavs[i].remain_power = power;
    }

    // 读取障碍物信息（如果需要）
    obstacles.clear();
    int obstacle_num;
    cin >> obstacle_num;
    for (int i = 0; i < obstacle_num; ++i) {
        obstacle obs;
        cin >> obs.position.x >> obs.position.y >> obs.radius >>
        obs.obstacle_time; obstacles.push_back(obs);
    }

    candidate best_candidate = GA();

    // 可视化 best_candidate 或输出结果
    return 0;
}