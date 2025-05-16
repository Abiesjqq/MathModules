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

int calculateCost(const vector<pair<int, int>>& assignment) {
    double total_time = 0.0;

    // Deep‑copy UAV state
    struct State {
        position_coord pos;
        double power;
        bool at_origin;
    };
    vector<State> S(drone_num);
    for (int i = 0; i < drone_num; i++) {
        S[i].pos = uavs[i].position;
        S[i].power = max_time;
        S[i].at_origin = false;
    }

    // Simulate until all back at origin or out of power
    while (true) {
        // check termination
        bool all_done = true;
        for (auto& s : S)
            if (!s.at_origin && s.power > 0) {
                all_done = false;
                break;
            }
        if (all_done)
            break;

        total_time += time_step;
        vector<UAV> converted_uavs(drone_num);
        for (int i = 0; i < drone_num; ++i) {
            converted_uavs[i].position = S[i].pos;
            converted_uavs[i].remain_power = static_cast<int>(S[i].power);
        }

        auto angles = swarm(converted_uavs, tasks, assignment, obstacles);

        for (int i = 0; i < drone_num; i++) {
            auto& s = S[i];
            if (s.at_origin || s.power <= 0)
                continue;

            // move
            s.pos.x += cos(angles[i]) * speed * time_step;
            s.pos.y += sin(angles[i]) * speed * time_step;
            s.power -= time_step;

            // check origin
            if (calculateDistance(s.pos, {0, 0}) < epsilon) {
                s.at_origin = true;
            }
        }
    }

    return static_cast<int>(round(total_time));
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
    // 1. Copy UAV state
    struct State {
        position_coord pos;
        double power;
    };
    vector<State> S(initial_uavs.size());
    for (size_t i = 0; i < S.size(); i++) {
        S[i].pos = initial_uavs[i].position;
        S[i].power = initial_uavs[i].remain_power;
    }

    // 2. Build per‑drone queues
    int n = S.size(), m = tasks.size();
    vector<vector<int>> queues(n);
    for (auto& p : assignment) {
        int d = p.first, t = p.second - 1;
        if (t >= 0 && t < m)
            queues[d].push_back(t);
    }

    double total_time = 0;
    vector<bool> visited(m, false);
    while (total_time < max_time * 2) {
        // check done
        if (all_of(visited.begin(), visited.end(), [](bool v) { return v; }))
            break;

        total_time += time_step;
        // update each UAV
        for (int i = 0; i < n; i++) {
            if (S[i].power <= 0 || queues[i].empty())
                continue;
            int tid = queues[i][0];
            auto& target = targets[tid];
            double dx = target.x - S[i].pos.x, dy = target.y - S[i].pos.y;
            double dist = sqrt(dx * dx + dy * dy);

            if (dist < epsilon) {
                visited[tid] = true;
                queues[i].erase(queues[i].begin());
            } else {
                double theta = angles[i];
                // or use atan2(dy,dx) if you want greedy
                S[i].pos.x += cos(theta) * speed * time_step;
                S[i].pos.y += sin(theta) * speed * time_step;
                S[i].power -= time_step;
            }
        }

        // collision & obstacle check…
        // (same as your code but on S[i].pos)
    }

    int unvisited = count(visited.begin(), visited.end(), false);
    return static_cast<int>(round(total_time)) + unvisited * 10000;
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
            obs.obstacle_time;
        obstacles.push_back(obs);
    }

    candidate best_candidate = GA();

    // 可视化 best_candidate 或输出结果
    return 0;
}