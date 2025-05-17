#include <bits/stdc++.h>
using namespace std;

// 结构体定义
struct position_coord {
    int x, y;
};

struct task {
    char priority;
    int x, y, num;
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

struct candidate {
    vector<pair<int, int>> assignment;
    int cost;
};

// 常量定义
namespace Constants {
const int INF = 4e5;
const int max_time = 600;
const int speed = 50;
const int max_dist = 1000;
const int min_dist = 50;
const int epsilon = 10;
const double time_step = 0.2;
const int swarm_gen = 60;
const int swarm_size = 30;
const double MAX_DRIFT = 0.8;
const double IBC = 1.5;
const double SBC = 1.5;
}  // namespace Constants

// 随机数生成器
class RandomGenerator {
   private:
    mt19937 gen;
    uniform_real_distribution<double> real_dist;
    uniform_int_distribution<int> int_dist;

   public:
    RandomGenerator()
        : gen(random_device{}()), real_dist(0.0, 1.0), int_dist(0, INT_MAX) {}

    // 获取[0,1)之间的随机浮点数
    double random() { return real_dist(gen); }

    // 获取[min,max]之间的随机整数
    int randomInt(int min, int max) {
        return uniform_int_distribution<int>(min, max)(gen);
    }

    // 获取[min,max)之间的随机浮点数
    double randomDouble(double min, double max) {
        return uniform_real_distribution<double>(min, max)(gen);
    }
};

// 工具函数
namespace Utils {
double calculateDistance(position_coord pos1, position_coord pos2) {
    return sqrt(pow((pos1.x - pos2.x), 2) + pow((pos1.y - pos2.y), 2));
}
}  // namespace Utils

// 无人机系统类
class UAVSystem {
   private:
    int drone_num;
    int target_num;
    vector<UAV> uavs;
    vector<task> tasks;
    vector<obstacle> obstacles;
    vector<position_coord> targets;
    RandomGenerator rng;

   public:
    UAVSystem(int drones, int targets)
        : drone_num(drones), target_num(targets) {}

    void setUAVs(const vector<UAV>& u) { uavs = u; }
    void setTasks(const vector<task>& t) { tasks = t; }
    void setObstacles(const vector<obstacle>& o) { obstacles = o; }
    void setTargets(const vector<position_coord>& t) { targets = t; }

    vector<bool> detectMovable() {
        vector<bool> movable(drone_num, true);
        for (int i = 0; i < drone_num; i++) {
            UAV uav = uavs[i];
            if (uav.remain_power > 0 && uav.remain_power < Constants::max_time)
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

    bool is_valid() {
        for (int i = 0; i < drone_num; i++) {
            position_coord pos_i = uavs[i].position;
            for (const auto& obs : obstacles) {
                if (Utils::calculateDistance(pos_i, obs.position) <
                    obs.radius) {
                    return false;
                }
            }
            for (int j = 0; j < drone_num; j++) {
                position_coord pos_j = uavs[j].position;
                double cur_dist = Utils::calculateDistance(pos_i, pos_j);
                if (cur_dist > Constants::max_dist)
                    return false;
                if ((pos_i.x == 0 && pos_i.y == 0) ||
                    (pos_j.x == 0 && pos_j.y == 0))
                    continue;
                if (cur_dist < Constants::min_dist)
                    return false;
            }
        }
        return true;
    }

    int calculate_fitness(const vector<pair<int, int>>& assignment,
                          const vector<UAV>& initial_uavs,
                          const vector<task>& tasks,
                          const vector<obstacle>& obstacles) {
        // 深拷贝 UAV 状态
        vector<UAV> uavs = initial_uavs;
        int n_drones = uavs.size();
        int n_targets = tasks.size();

        // 标记目标访问状态和分配关系
        vector<bool> visited(n_targets, false);
        vector<bool> target_assigned(n_targets, false);

        // 存储每架无人机的任务队列
        vector<vector<int>> task_queues(n_drones);
        for (const auto& assign : assignment) {
            int drone_id = assign.first;
            int task_id = assign.second - 1;  // 任务编号从1开始
            if (task_id >= 0 && task_id < n_targets) {
                task_queues[drone_id].push_back(task_id);
                target_assigned[task_id] = true;
            }
        }

        double fitness = 0.0;

        // 计算已分配目标的距离成本
        for (int i = 0; i < n_drones; ++i) {
            if (!task_queues[i].empty()) {
                int next_target_id = task_queues[i][0];
                position_coord target_pos = {tasks[next_target_id].x,
                                             tasks[next_target_id].y};
                double dist =
                    Utils::calculateDistance(uavs[i].position, target_pos);
                fitness += dist;
            }
        }

        // 计算未分配目标的惩罚成本（到最近无人机的距离）
        for (int t = 0; t < n_targets; ++t) {
            if (!target_assigned[t]) {
                double min_dist = numeric_limits<double>::max();
                position_coord target_pos = {tasks[t].x, tasks[t].y};

                for (const auto& uav : uavs) {
                    double dist =
                        Utils::calculateDistance(uav.position, target_pos);
                    if (dist < min_dist) {
                        min_dist = dist;
                    }
                }
                fitness += min_dist * 2.0;  // 未分配目标的加权惩罚
            }
        }

        // 碰撞检测（保留原有逻辑）
        for (int i = 0; i < n_drones; ++i) {
            if (uavs[i].remain_power <= 0)
                continue;

            // 无人机间碰撞
            for (int j = i + 1; j < n_drones; ++j) {
                if (uavs[j].remain_power <= 0)
                    continue;
                double dist = Utils::calculateDistance(uavs[i].position,
                                                       uavs[j].position);
                if (dist < Constants::min_dist)
                    return Constants::INF;
            }

            // 障碍物碰撞
            for (const auto& obs : obstacles) {
                double dist =
                    Utils::calculateDistance(uavs[i].position, obs.position);
                if (dist < obs.radius)
                    return Constants::INF;
            }

            // 边界检查
            if (uavs[i].position.x < 0 || uavs[i].position.y < 0 ||
                uavs[i].position.x > 10000 || uavs[i].position.y > 10000) {
                return Constants::INF;
            }
        }

        return static_cast<int>(fitness);
    }

    vector<double> swarm(const vector<pair<int, int>>& assignment) {
        int drone_num = uavs.size();
        vector<vector<double>> particle_swarm(Constants::swarm_size,
                                              vector<double>(drone_num));
        vector<vector<double>> velocity(Constants::swarm_size,
                                        vector<double>(drone_num, 0.0));
        vector<vector<double>> individual_best = particle_swarm;
        vector<int> best_fitness(Constants::swarm_size, INT_MAX);
        vector<double> swarm_best(drone_num);
        int global_best_fitness = INT_MAX;

        // 初始化粒子群
        for (int i = 0; i < Constants::swarm_size; ++i) {
            for (int j = 0; j < drone_num; ++j) {
                particle_swarm[i][j] = rng.randomDouble(0, 2 * M_PI);
            }
        }

        // 初始评估
        for (int i = 0; i < Constants::swarm_size; ++i) {
            vector<UAV> uav_copy = uavs;
            int fit = calculate_fitness(assignment, uav_copy, tasks, obstacles);
            if (fit < best_fitness[i]) {
                best_fitness[i] = fit;
                individual_best[i] = particle_swarm[i];
            }
            if (fit < global_best_fitness) {
                global_best_fitness = fit;
                swarm_best = particle_swarm[i];
            }
        }

        // PSO主循环
        for (int gen = 0; gen < Constants::swarm_gen; ++gen) {
            // 计算自适应惯性权重
            double w = 0.9 - (0.5 * gen / Constants::swarm_gen);  // 线性衰减

            for (int i = 0; i < Constants::swarm_size; ++i) {
                for (int j = 0; j < drone_num; ++j) {
                    double r1 = rng.random();
                    double r2 = rng.random();

                    // 使用自适应参数更新速度
                    velocity[i][j] =
                        w * velocity[i][j] +
                        Constants::IBC * r1 *
                            (individual_best[i][j] - particle_swarm[i][j]) +
                        Constants::SBC * r2 *
                            (swarm_best[j] - particle_swarm[i][j]);

                    // 更新粒子位置
                    particle_swarm[i][j] += velocity[i][j];
                    particle_swarm[i][j] =
                        max(0.0, min(2 * M_PI, particle_swarm[i][j]));
                }

                // 评估新位置
                vector<UAV> uav_copy = uavs;
                int fit =
                    calculate_fitness(assignment, uav_copy, tasks, obstacles);
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

    // 在PSO结果后，将角度序列转换为坐标点
    vector<position_coord> generatePath(const vector<double>& angles,
                                        const UAV& uav) {
        vector<position_coord> path = {uav.position};
        position_coord current = uav.position;
        for (double theta : angles) {
            current.x += cos(theta) * Constants::speed * Constants::time_step;
            current.y += sin(theta) * Constants::speed * Constants::time_step;
            path.push_back(current);
        }

        return path;
    }

    // 新增外部函数：计算路径总距离
    double calculatePathDistance(const vector<position_coord>& path) {
        double distance = 0.0;
        for (int i = 1; i < path.size(); ++i) {
            distance += Utils::calculateDistance(path[i - 1], path[i]);
        }
        return distance;
    }

    int calculateCost(const vector<pair<int, int>>& assignment) {
        double total_time = 0.0;
        vector<position_coord> uav_locations(drone_num, {0, 0});
        vector<int> uav_powers(drone_num, Constants::max_time);
        vector<bool> in_origin(drone_num, false);
        bool all_in_origin;

        while (!all_in_origin) {
            total_time += Constants::time_step;
            vector<double> angles = swarm(assignment);

            for (int i = 0; i < drone_num; i++) {
                position_coord position = uavs[i].position;
                double theta = angles[i];
                position.x +=
                    cos(theta) * Constants::speed * Constants::time_step;
                position.y +=
                    sin(theta) * Constants::speed * Constants::time_step;
                if (Utils::calculateDistance(position, {0, 0}) <
                    Constants::epsilon) {
                    in_origin[i] = true;
                }
            }

            all_in_origin = true;
            for (int i = 0; i < drone_num; i++) {
                if (!in_origin[i])
                    all_in_origin = false;
            }

            for (int i = 0; i < drone_num; i++) {
                if (!in_origin[i]) {
                    uav_powers[i] -= Constants::time_step;
                }
            }
        }
        return round(total_time);
    }
    candidate GA() {
        int max_gen = 15 * drone_num + 50;
        int pop_size = 10 * drone_num;
        int reserved_candidates = drone_num;
        int unreserved = pop_size - reserved_candidates;
        int task_mutations = 3 * drone_num;
        int path_exchanges = 6 * drone_num;
        int sequence_exchanges = 3 * drone_num;
        const int tournament_size = 3;  // 锦标赛选择的大小

        vector<candidate> population(pop_size);
        for (int i = 0; i < pop_size; i++) {
            candidate candi;
            for (int j = 1; j <= target_num; j++) {
                candi.assignment.push_back(
                    {rng.randomInt(0, drone_num - 1), j});
            }
            for (int k = 0; k < drone_num; k++) {
                int charge_task_id = target_num + 1 + k;
                candi.assignment.push_back({k, charge_task_id});
            }
            candi.cost = Constants::INF;
            population[i] = candi;
        }

        candidate best_candidate;
        best_candidate.cost = Constants::INF;

        // 评估初始种群
        for (int i = 0; i < pop_size; i++) {
            population[i].cost = calculateCost(population[i].assignment);
        }
        sort(population.begin(), population.end(),
             [](const candidate& a, const candidate& b) {
                 return a.cost < b.cost;
             });
        best_candidate = population[0];

        for (int gen = 1; gen <= max_gen; gen++) {
            vector<candidate> temp_population(pop_size);

            // 评估当前种群
            for (int i = 0; i < pop_size; i++) {
                population[i].cost = calculateCost(population[i].assignment);
            }
            sort(population.begin(), population.end(),
                 [](const candidate& a, const candidate& b) {
                     return a.cost < b.cost;
                 });

            // 更新全局最优
            if (population[0].cost < best_candidate.cost) {
                best_candidate = population[0];
            }

            // 精英保留策略：直接保留前reserved_candidates个最优个体
            for (int i = 0; i < reserved_candidates; i++) {
                temp_population[i] = population[i];
            }

            // 使用锦标赛选择填充剩余种群
            for (int i = reserved_candidates; i < pop_size; i++) {
                // 进行锦标赛选择
                candidate best_in_tournament =
                    population[rng.randomInt(0, pop_size - 1)];
                for (int j = 1; j < tournament_size; j++) {
                    int random_index = rng.randomInt(0, pop_size - 1);
                    if (population[random_index].cost <
                        best_in_tournament.cost) {
                        best_in_tournament = population[random_index];
                    }
                }
                temp_population[i] = best_in_tournament;
            }

            // 保持原有的变异操作不变
            for (int i = 0; i < task_mutations; i++) {
                int ind =
                    reserved_candidates + rng.randomInt(0, unreserved - 1);
                temp_population[ind]
                    .assignment[rng.randomInt(0, target_num - 1)]
                    .first = rng.randomInt(0, drone_num - 1);
            }

            for (int i = 0; i < path_exchanges; i++) {
                int ind1 =
                    reserved_candidates + rng.randomInt(0, unreserved - 1);
                int ind2 =
                    reserved_candidates + rng.randomInt(0, unreserved - 1);
                while (ind2 == ind1) {
                    ind2 =
                        reserved_candidates + rng.randomInt(0, unreserved - 1);
                }
                int length = rng.randomInt(1, target_num);
                int start = rng.randomInt(0, target_num - length);
                int end = start + length - 1;
                for (int pos = start; pos <= end; ++pos) {
                    swap(temp_population[ind1].assignment[pos].first,
                         temp_population[ind2].assignment[pos].first);
                }
            }

            for (int i = 0; i < sequence_exchanges; i++) {
                int ind =
                    reserved_candidates + rng.randomInt(0, unreserved - 1);
                int pos = rng.randomInt(0, target_num - 2);
                auto& a = temp_population[ind].assignment;
                swap(a[pos].second, a[pos + 1].second);
            }

            // 确保最优个体不被破坏
            if (best_candidate.cost < temp_population[0].cost) {
                temp_population[0] = best_candidate;
            }

            population.swap(temp_population);
        }
        return best_candidate;
    }
};
int main() {
    int target_num, drone_num;
    cin >> target_num >> drone_num;

    UAVSystem uavSystem(drone_num, target_num);

    vector<position_coord> targets(target_num);
    for (int i = 0; i < target_num; ++i) {
        int x, y, dummy_power;
        cin >> targets[i].x >> targets[i].y >> dummy_power;
    }
    uavSystem.setTargets(targets);

    vector<UAV> uavs(drone_num);
    for (int i = 0; i < drone_num; ++i) {
        int x, y, power;
        cin >> x >> y >> power;
        uavs[i].position = {x, y};
        uavs[i].remain_power = power;
    }
    uavSystem.setUAVs(uavs);

    vector<obstacle> obstacles;
    int obstacle_num;
    cin >> obstacle_num;
    for (int i = 0; i < obstacle_num; ++i) {
        obstacle obs;
        cin >> obs.position.x >> obs.position.y >> obs.radius >>
            obs.obstacle_time;
        obstacles.push_back(obs);
    }
    uavSystem.setObstacles(obstacles);

    candidate best_candidate = uavSystem.GA();

    // 输出最优解的任务分配方案
    cout << "===== Optimal Task Assignment =====" << endl;
    cout << "Total cost: " << best_candidate.cost << endl;
    cout << "----------------------------------" << endl;

    // 按无人机分组输出任务分配
    vector<vector<int>> drone_tasks(drone_num);
    for (const auto& assign : best_candidate.assignment) {
        int drone_id = assign.first;
        int task_id = assign.second;
        if (task_id <= target_num) {  // 只显示实际目标点任务，不显示充电任务
            drone_tasks[drone_id].push_back(task_id);
        }
    }

    for (int i = 0; i < drone_num; ++i) {
        cout << "Drone " << i << " tasks: ";
        if (drone_tasks[i].empty()) {
            cout << "No tasks assigned";
        } else {
            for (int task_id : drone_tasks[i]) {
                cout << task_id << " ";
            }
        }
        cout << endl;
    }

    // 输出目标点覆盖情况
    vector<bool> target_covered(target_num, false);
    for (const auto& assign : best_candidate.assignment) {
        int task_id = assign.second;
        if (task_id <= target_num) {
            target_covered[task_id - 1] = true;
        }
    }

    int covered_count =
        count(target_covered.begin(), target_covered.end(), true);
    cout << "----------------------------------" << endl;
    cout << "Target coverage: " << covered_count << "/" << target_num << endl;
    if (covered_count < target_num) {
        cout << "Uncovered targets: ";
        for (int i = 0; i < target_num; ++i) {
            if (!target_covered[i]) {
                cout << (i + 1) << " ";
            }
        }
        cout << endl;
    }

    // 输出无人机利用率
    cout << "----------------------------------" << endl;
    vector<int> task_counts(drone_num, 0);
    for (const auto& assign : best_candidate.assignment) {
        if (assign.second <= target_num) {
            task_counts[assign.first]++;
        }
    }

    cout << "Drone task distribution:" << endl;
    for (int i = 0; i < drone_num; ++i) {
        cout << "Drone " << i << ": " << task_counts[i] << " tasks" << endl;
    }

    return 0;
}