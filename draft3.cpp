#include <bits/stdc++.h>
using namespace std;

// 结构体定义
struct position_coord {  // 位置坐标结构体
    int x, y;            // x和y坐标
};

// 任务类型枚举
enum TaskType {
    EMERGENCY_DELIVERY = 1,  // 紧急投送
    NORMAL_DELIVERY = 2,     // 普通投送
    RECON = 3                // 侦察任务
};

// 任务结构体
struct task {
    TaskType priority;   // 任务优先级
    int x, y;            // 目标位置坐标
    int num;             // 任务编号
    union {              // 联合体，根据任务类型存储不同需求
        int load;        // 对于投送任务，存储载荷需求
        int hover_time;  // 对于侦察任务，存储悬停时间需求
    } requirement;
};

// 无人机结构体
struct UAV {
    position_coord position;  // 当前位置
    int remain_power;         // 剩余电量（秒）
    int max_power;            // 最大电量
    int charge_time;          // 充电时间
    int load_capacity;        // 载荷能力（投送任务）
    int hover_capacity;       // 悬停能力（侦察任务）
};

// 障碍物结构体
struct obstacle {
    position_coord position;  // 障碍物位置
    int radius;               // 障碍物半径
    int obstacle_time;        // 障碍物持续时间
};

// 候选解结构体
struct candidate {
    vector<pair<int, int>> assignment;  // 任务分配方案（无人机ID，任务ID）
    int cost;                           // 该方案的总成本
};

// 常量定义命名空间
namespace Constants {
const int INF = 4e5;           // 表示无穷大的值
const int max_time = 600;      // 最大任务时间（秒）
const int speed = 50;          // 无人机速度（米/秒）
const int max_dist = 1000;     // 无人机间最大距离（米）
const int min_dist = 50;       // 无人机间最小安全距离（米）
const int epsilon = 10;        // 接近阈值（米）
const double time_step = 0.2;  // 仿真时间步长（秒）
const int swarm_gen = 60;      // PSO迭代次数
const int swarm_size = 30;     // PSO粒子群大小
const double MAX_DRIFT = 0.8;  // PSO参数
const double IBC = 1.5;        // PSO认知系数
const double SBC = 1.5;        // PSO社会系数

// 优先级权重（优先级越高，权重越大）
const double PRIORITY_WEIGHT_EMERGENCY = 5.0;  // 紧急投送权重
const double PRIORITY_WEIGHT_NORMAL = 2.0;     // 普通投送权重
const double PRIORITY_WEIGHT_RECON = 1.0;      // 侦察任务权重
}  // namespace Constants

// 随机数生成器类
class RandomGenerator {
   private:
    mt19937 gen;                                  // 随机数引擎
    uniform_real_distribution<double> real_dist;  // 实数分布
    uniform_int_distribution<int> int_dist;       // 整数分布

   public:
    RandomGenerator()
        : gen(random_device{}()), real_dist(0.0, 1.0), int_dist(0, INT_MAX) {}

    // 生成0-1之间的随机实数
    double random() { return real_dist(gen); }

    // 生成[min,max]范围内的随机整数
    int randomInt(int min, int max) {
        return uniform_int_distribution<int>(min, max)(gen);
    }

    // 生成[min,max]范围内的随机实数
    double randomDouble(double min, double max) {
        return uniform_real_distribution<double>(min, max)(gen);
    }
};

// 工具函数命名空间
namespace Utils {
// 计算两点间距离
double calculateDistance(position_coord pos1, position_coord pos2) {
    return sqrt(pow((pos1.x - pos2.x), 2) + pow((pos1.y - pos2.y), 2));
}

// 根据任务类型获取权重
double getPriorityWeight(TaskType priority) {
    switch (priority) {
        case EMERGENCY_DELIVERY:
            return Constants::PRIORITY_WEIGHT_EMERGENCY;
        case NORMAL_DELIVERY:
            return Constants::PRIORITY_WEIGHT_NORMAL;
        case RECON:
            return Constants::PRIORITY_WEIGHT_RECON;
        default:
            return 1.0;
    }
}
}  // namespace Utils

// 无人机系统类
class UAVSystem {
   private:
    int drone_num;                   // 无人机数量
    int target_num;                  // 目标点数量
    vector<UAV> uavs;                // 无人机列表
    vector<task> tasks;              // 任务列表
    vector<obstacle> obstacles;      // 障碍物列表
    vector<position_coord> targets;  // 目标点列表
    RandomGenerator rng;             // 随机数生成器实例

    // 模拟无人机移动并返回路径长度
    double simulateMovement(UAV& uav,
                            const vector<double>& angles,
                            const position_coord& target) {
        double total_distance = 0.0;
        position_coord current = uav.position;

        // 按照角度序列逐步移动
        for (double theta : angles) {
            position_coord next;
            next.x = current.x +
                     cos(theta) * Constants::speed * Constants::time_step;
            next.y = current.y +
                     sin(theta) * Constants::speed * Constants::time_step;

            // 如果接近目标点，则返回总距离
            if (Utils::calculateDistance(next, target) <= Constants::epsilon) {
                total_distance += Utils::calculateDistance(current, target);
                return total_distance;
            }

            total_distance += Utils::calculateDistance(current, next);
            current = next;
        }

        // 加上最后一段距离
        total_distance += Utils::calculateDistance(current, target);
        return total_distance;
    }

    // 检查无人机是否能执行特定任务
    bool canPerformTask(const UAV& uav, const task& t) {
        if (t.priority == RECON) {
            return uav.hover_capacity >= t.requirement.hover_time;
        } else {  // 投送任务
            return uav.load_capacity >= t.requirement.load;
        }
    }

   public:
    // 构造函数
    UAVSystem(int drones, int targets)
        : drone_num(drones), target_num(targets) {}

    // 设置无人机列表
    void setUAVs(const vector<UAV>& u) { uavs = u; }
    // 设置任务列表
    void setTasks(const vector<task>& t) { tasks = t; }
    // 设置障碍物列表
    void setObstacles(const vector<obstacle>& o) { obstacles = o; }
    // 设置目标点列表
    void setTargets(const vector<position_coord>& t) { targets = t; }

    // 计算适应度（带优先级权重）
    int calculate_fitness(const vector<pair<int, int>>& assignment,
                          const vector<UAV>& initial_uavs,
                          const vector<task>& tasks,
                          const vector<obstacle>& obstacles) {
        vector<UAV> uavs = initial_uavs;
        int n_drones = uavs.size();
        int n_targets = tasks.size();

        // 跟踪任务分配和完成情况
        vector<bool> target_assigned(n_targets, false);
        vector<vector<int>> task_queues(n_drones);

        // 为每架无人机构建任务队列，确保优先级顺序
        for (const auto& assign : assignment) {
            int drone_id = assign.first;
            int task_id = assign.second - 1;  // 任务ID从1开始

            if (task_id >= 0 && task_id < n_targets) {
                // 按优先级顺序插入任务（紧急优先，侦察最后）
                auto it = task_queues[drone_id].begin();
                while (it != task_queues[drone_id].end() &&
                       tasks[*it].priority <= tasks[task_id].priority) {
                    ++it;
                }
                task_queues[drone_id].insert(it, task_id);
                target_assigned[task_id] = true;
            }
        }

        double total_cost = 0.0;

        // 计算每架无人机的路径成本
        for (int drone_id = 0; drone_id < n_drones; ++drone_id) {
            if (task_queues[drone_id].empty())
                continue;

            UAV uav = uavs[drone_id];
            position_coord current_pos = uav.position;

            for (int task_id : task_queues[drone_id]) {
                const task& current_task = tasks[task_id];
                position_coord target_pos = {current_task.x, current_task.y};

                // 检查无人机是否能执行此任务
                if (!canPerformTask(uav, current_task)) {
                    // 分配不可能任务的大惩罚
                    total_cost += Constants::INF * Utils::getPriorityWeight(
                                                       current_task.priority);
                    continue;
                }

                // 使用PSO优化路径
                vector<double> best_angles =
                    swarm_single_drone(uav, target_pos);
                int time_used = 0;
                double path_length = simulateMovementWithCharging(
                    uav, best_angles, target_pos, time_used);

                if (path_length >= Constants::INF) {
                    // 路径失败的加权惩罚
                    total_cost += Constants::INF * Utils::getPriorityWeight(
                                                       current_task.priority);
                    continue;
                }

                // 应用任务完成（悬停时间或载荷）
                if (current_task.priority == RECON) {
                    uav.remain_power -= current_task.requirement.hover_time;
                } else {
                    uav.load_capacity -= current_task.requirement.load;
                }

                // 添加加权路径成本
                total_cost += path_length *
                              Utils::getPriorityWeight(current_task.priority);
                uav.position = target_pos;
            }

            uavs[drone_id] = uav;
        }

        // 未分配目标的惩罚（按优先级加权）
        for (int t = 0; t < n_targets; ++t) {
            if (!target_assigned[t]) {
                double min_dist = numeric_limits<double>::max();
                position_coord target_pos = {tasks[t].x, tasks[t].y};

                for (const auto& uav : uavs) {
                    double dist =
                        Utils::calculateDistance(uav.position, target_pos);
                    if (dist < min_dist)
                        min_dist = dist;
                }

                // 按任务优先级加权惩罚
                total_cost += min_dist * 2.0 *
                              Utils::getPriorityWeight(tasks[t].priority);
            }
        }

        // 碰撞检测
        for (int i = 0; i < n_drones; ++i) {
            if (uavs[i].remain_power <= 0)
                continue;

            // 无人机间碰撞检测
            for (int j = i + 1; j < n_drones; ++j) {
                if (uavs[j].remain_power <= 0)
                    continue;
                double dist = Utils::calculateDistance(uavs[i].position,
                                                       uavs[j].position);
                if (dist < Constants::min_dist)
                    return Constants::INF;
            }

            // 障碍物碰撞检测
            for (const auto& obs : obstacles) {
                double dist =
                    Utils::calculateDistance(uavs[i].position, obs.position);
                if (dist < obs.radius)
                    return Constants::INF;
            }
        }

        return static_cast<int>(total_cost);
    }

    // 处理充电期间的移动
    double simulateMovementWithCharging(UAV& uav,
                                        const vector<double>& angles,
                                        const position_coord& target,
                                        int& time_used) {
        double total_distance = 0.0;
        position_coord current = uav.position;

        for (double theta : angles) {
            position_coord next;
            next.x = current.x +
                     cos(theta) * Constants::speed * Constants::time_step;
            next.y = current.y +
                     sin(theta) * Constants::speed * Constants::time_step;

            if (Utils::calculateDistance(next, target) <= Constants::epsilon) {
                double final_dist = Utils::calculateDistance(current, target);
                double final_time = final_dist / Constants::speed;

                if (uav.remain_power >= final_time) {
                    total_distance += final_dist;
                    time_used += final_time;
                    uav.remain_power -= final_time;
                    return total_distance;
                } else {
                    return handleCharging(uav, current, target, total_distance,
                                          time_used);
                }
            }

            double dist_segment = Utils::calculateDistance(current, next);
            double time_segment = dist_segment / Constants::speed;

            if (uav.remain_power >= time_segment) {
                total_distance += dist_segment;
                time_used += time_segment;
                uav.remain_power -= time_segment;
                current = next;
            } else {
                return handleCharging(uav, current, target, total_distance,
                                      time_used);
            }
        }

        double remaining_dist = Utils::calculateDistance(current, target);
        double remaining_time = remaining_dist / Constants::speed;

        if (uav.remain_power >= remaining_time) {
            total_distance += remaining_dist;
            time_used += remaining_time;
            uav.remain_power -= remaining_time;
        } else {
            total_distance =
                handleCharging(uav, current, target, total_distance, time_used);
        }

        return total_distance;
    }

    // 处理充电逻辑
    double handleCharging(UAV& uav,
                          const position_coord& current,
                          const position_coord& target,
                          double current_distance,
                          int& time_used) {
        position_coord origin = {0, 0};
        double return_dist = Utils::calculateDistance(current, origin);
        double return_time = return_dist / Constants::speed;

        if (uav.remain_power < return_time)
            return Constants::INF;

        current_distance += return_dist;
        time_used += return_time;
        uav.remain_power -= return_time;

        time_used += uav.charge_time;
        uav.remain_power = uav.max_power;

        double to_target_dist = Utils::calculateDistance(origin, target);
        double to_target_time = to_target_dist / Constants::speed;

        if (uav.remain_power < to_target_time)
            return Constants::INF;

        current_distance += to_target_dist;
        time_used += to_target_time;
        uav.remain_power -= to_target_time;

        return current_distance;
    }

    // PSO路径优化
    vector<double> swarm_single_drone(const UAV& uav,
                                      const position_coord& target) {
        // 初始化粒子群
        vector<vector<double>> particle_swarm(Constants::swarm_size,
                                              vector<double>(1));
        vector<vector<double>> velocity(Constants::swarm_size,
                                        vector<double>(1, 0.0));
        vector<vector<double>> individual_best = particle_swarm;
        vector<double> best_fitness(Constants::swarm_size,
                                    numeric_limits<double>::max());
        vector<double> swarm_best(1);
        double global_best_fitness = numeric_limits<double>::max();

        // 初始化粒子位置
        for (int i = 0; i < Constants::swarm_size; ++i) {
            particle_swarm[i][0] = rng.randomDouble(0, 2 * M_PI);
        }

        // 评估初始粒子群
        for (int i = 0; i < Constants::swarm_size; ++i) {
            UAV uav_copy = uav;
            int time_used = 0;
            double path_length = simulateMovementWithCharging(
                uav_copy, particle_swarm[i], target, time_used);
            if (path_length < best_fitness[i]) {
                best_fitness[i] = path_length;
                individual_best[i] = particle_swarm[i];
            }
            if (path_length < global_best_fitness) {
                global_best_fitness = path_length;
                swarm_best = particle_swarm[i];
            }
        }

        // PSO迭代
        for (int gen = 0; gen < Constants::swarm_gen; ++gen) {
            double w = 0.9 - (0.5 * gen / Constants::swarm_gen);  // 惯性权重
            for (int i = 0; i < Constants::swarm_size; ++i) {
                double r1 = rng.random();
                double r2 = rng.random();
                // 更新速度
                velocity[i][0] =
                    w * velocity[i][0] +
                    Constants::IBC * r1 *
                        (individual_best[i][0] - particle_swarm[i][0]) +
                    Constants::SBC * r2 *
                        (swarm_best[0] - particle_swarm[i][0]);
                // 更新位置
                particle_swarm[i][0] += velocity[i][0];
                particle_swarm[i][0] = fmod(particle_swarm[i][0], 2 * M_PI);
                if (particle_swarm[i][0] < 0)
                    particle_swarm[i][0] += 2 * M_PI;

                // 评估新位置
                UAV uav_copy = uav;
                int time_used = 0;
                double path_length = simulateMovementWithCharging(
                    uav_copy, particle_swarm[i], target, time_used);
                if (path_length < best_fitness[i]) {
                    best_fitness[i] = path_length;
                    individual_best[i] = particle_swarm[i];
                }
                if (path_length < global_best_fitness) {
                    global_best_fitness = path_length;
                    swarm_best = particle_swarm[i];
                }
            }
        }
        return swarm_best;
    }

    // 计算分配方案的成本
    int calculateCost(const vector<pair<int, int>>& assignment) {
        return calculate_fitness(assignment, uavs, tasks, obstacles);
    }

    // 打印进度条
    void printProgress(int current_gen, int total_gen, int current_cost) {
        const int bar_width = 50;  // 进度条长度
        float progress = static_cast<float>(current_gen) / total_gen;

        cout << "\rGeneration: " << current_gen << " / " << total_gen
             << " | Best Cost: " << current_cost << " | [";

        int pos = static_cast<int>(bar_width * progress);
        for (int i = 0; i < bar_width; ++i) {
            if (i < pos)
                cout << "|||";
            else if (i == pos)
                cout << ">";
            else
                cout << " ";
        }
        cout << "] " << int(progress * 100.0) << "%     ";
        cout.flush();
    }

    // 遗传算法（带优先级支持）
    candidate GA() {
        int max_gen = 15 * drone_num + 50;                // 最大迭代次数
        int pop_size = 10 * drone_num;                    // 种群大小
        int reserved_candidates = drone_num;              // 保留的精英个体数
        int unreserved = pop_size - reserved_candidates;  // 非精英个体数
        int task_mutations = 3 * drone_num;               // 任务变异次数
        int path_exchanges = 6 * drone_num;               // 路径交换次数
        int sequence_exchanges = 3 * drone_num;           // 序列交换次数
        const int tournament_size = 3;                    // 锦标赛选择的大小

        // 初始化种群
        vector<candidate> population(pop_size);
        for (int i = 0; i < pop_size; i++) {
            candidate candi;
            // 随机分配任务给无人机
            for (int j = 1; j <= target_num; j++) {
                candi.assignment.push_back(
                    {rng.randomInt(0, drone_num - 1), j});
            }
            // 添加充电任务
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
        // 按成本排序
        sort(population.begin(), population.end(),
             [](const candidate& a, const candidate& b) {
                 return a.cost < b.cost;
             });
        best_candidate = population[0];

        // 遗传算法主循环
        int gen;
        for (gen = 1; gen <= max_gen; gen++) {
            vector<candidate> temp_population(pop_size);

            // 评估当前种群
            for (int i = 0; i < pop_size; i++) {
                population[i].cost = calculateCost(population[i].assignment);
            }
            sort(population.begin(), population.end(),
                 [](const candidate& a, const candidate& b) {
                     return a.cost < b.cost;
                 });

            // 更新最佳候选
            if (population[0].cost < best_candidate.cost) {
                best_candidate = population[0];
            }

            // 精英保留
            for (int i = 0; i < reserved_candidates; i++) {
                temp_population[i] = population[i];
            }

            // 锦标赛选择
            for (int i = reserved_candidates; i < pop_size; i++) {
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

            // 变异操作
            // 任务变异：随机改变任务的无人机分配
            for (int i = 0; i < task_mutations; i++) {
                int ind =
                    reserved_candidates + rng.randomInt(0, unreserved - 1);
                temp_population[ind]
                    .assignment[rng.randomInt(0, target_num - 1)]
                    .first = rng.randomInt(0, drone_num - 1);
            }

            // 路径交换：交换两个个体的部分任务分配
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

            // 序列交换：交换个体内部两个相邻任务的顺序
            for (int i = 0; i < sequence_exchanges; i++) {
                int ind =
                    reserved_candidates + rng.randomInt(0, unreserved - 1);
                int pos = rng.randomInt(0, target_num - 2);
                auto& a = temp_population[ind].assignment;
                swap(a[pos].second, a[pos + 1].second);
            }

            // 确保最佳候选不被丢弃
            if (best_candidate.cost < temp_population[0].cost) {
                temp_population[0] = best_candidate;
            }

            population.swap(temp_population);
        }

        return best_candidate;
    }
};

int main() {
    // 从终端输入目标数量和无人机数量
    int target_num, drone_num;
    cout << "Enter number of targets: ";
    cin >> target_num;
    cout << "Enter number of drones: ";
    cin >> drone_num;

    // 创建无人机系统实例
    UAVSystem uavSystem(drone_num, target_num);

    // 输入目标位置
    vector<position_coord> targets(target_num);
    cout << "\nEnter target positions (x y for each target):" << endl;
    for (int i = 0; i < target_num; ++i) {
        cout << "Target " << i + 1 << ": ";
        cin >> targets[i].x >> targets[i].y;
    }
    uavSystem.setTargets(targets);

    // 输入任务信息
    vector<task> tasks(target_num);
    cout << "\nEnter task details for each target (priority x y requirement):"
         << endl;
    cout << "Priority: 1=Emergency, 2=Normal, 3=Recon" << endl;
    for (int i = 0; i < target_num; ++i) {
        int priority, x, y, requirement;
        cout << "Target " << i + 1 << " task: ";
        cin >> priority >> x >> y >> requirement;

        tasks[i].priority = static_cast<TaskType>(priority);
        tasks[i].x = x;
        tasks[i].y = y;
        tasks[i].num = i + 1;

        // 根据任务类型设置需求
        if (priority == RECON) {
            tasks[i].requirement.hover_time = requirement;
        } else {
            tasks[i].requirement.load = requirement;
        }
    }
    uavSystem.setTasks(tasks);

    // 输入无人机信息
    vector<UAV> uavs(drone_num);
    cout << "\nEnter drone details (x y remain_power max_power charge_time "
            "load_capacity hover_capacity):"
         << endl;
    for (int i = 0; i < drone_num; ++i) {
        int x, y, power, max_power, charge_time, load_cap, hover_cap;
        cout << "Drone " << i + 1 << ": ";
        cin >> x >> y >> power >> max_power >> charge_time >> load_cap >>
            hover_cap;

        uavs[i].position = {x, y};
        uavs[i].remain_power = power;
        uavs[i].max_power = max_power;
        uavs[i].charge_time = charge_time;
        uavs[i].load_capacity = load_cap;
        uavs[i].hover_capacity = hover_cap;
    }
    uavSystem.setUAVs(uavs);

    // 输入障碍物信息
    vector<obstacle> obstacles;
    int obstacle_num;
    cout << "\nEnter number of obstacles: ";
    cin >> obstacle_num;
    if (obstacle_num > 0) {
        cout << "Enter obstacle details (x y radius obstacle_time):" << endl;
        for (int i = 0; i < obstacle_num; ++i) {
            obstacle obs;
            cout << "Obstacle " << i + 1 << ": ";
            cin >> obs.position.x >> obs.position.y >> obs.radius >>
                obs.obstacle_time;
            obstacles.push_back(obs);
        }
    }
    uavSystem.setObstacles(obstacles);

    // 运行遗传算法并获取最佳解决方案
    candidate best_candidate = uavSystem.GA();

    // 输出结果
    cout << "\n===== Optimal Task Assignment =====" << endl;
    cout << "Total weighted cost: " << best_candidate.cost << endl;
    cout << "----------------------------------" << endl;

    // 组织任务分配结果
    vector<vector<int>> drone_tasks(drone_num);
    vector<bool> target_covered(target_num, false);
    vector<int> priority_counts(4, 0);  // 索引1-3对应优先级

    // 解析最佳分配方案
    for (const auto& assign : best_candidate.assignment) {
        int drone_id = assign.first;
        int task_id = assign.second;
        if (task_id <= target_num) {
            drone_tasks[drone_id].push_back(task_id);
            target_covered[task_id - 1] = true;
            priority_counts[tasks[task_id - 1].priority]++;
        }
    }

    // 打印无人机任务分配情况
    for (int i = 0; i < drone_num; ++i) {
        cout << "Drone " << i + 1 << " tasks: ";
        if (drone_tasks[i].empty()) {
            cout << "No tasks assigned";
        } else {
            // 按优先级排序(紧急优先)
            sort(drone_tasks[i].begin(), drone_tasks[i].end(),
                 [&tasks](int a, int b) {
                     return tasks[a - 1].priority < tasks[b - 1].priority;
                 });

            // 打印任务ID和优先级
            for (int task_id : drone_tasks[i]) {
                cout << task_id << "(P" << tasks[task_id - 1].priority << ") ";
            }
        }
        cout << endl;
    }

    // 打印按优先级分类的目标覆盖情况
    cout << "----------------------------------" << endl;
    cout << "Target coverage by priority:" << endl;
    cout << "Emergency (P1): " << priority_counts[EMERGENCY_DELIVERY] << endl;
    cout << "Normal (P2): " << priority_counts[NORMAL_DELIVERY] << endl;
    cout << "Recon (P3): " << priority_counts[RECON] << endl;

    // 检查并打印未覆盖的目标
    int uncovered = count(target_covered.begin(), target_covered.end(), false);
    if (uncovered > 0) {
        cout << "Uncovered targets: ";
        for (int i = 0; i < target_num; ++i) {
            if (!target_covered[i]) {
                cout << (i + 1) << "(P" << tasks[i].priority << ") ";
            }
        }
        cout << endl;
    }

    return 0;
}

/*
输入数据

(1)
Enter number of targets: 5
Enter number of drones: 3

Enter target positions (x y for each target):
Target 1: 1200 800
Target 2: 300 450
Target 3: 950 200
Target 4: 600 1200
Target 5: 1500 500

Enter task details for each target (priority x y requirement):
Priority: 1=Emergency, 2=Normal, 3=Recon
Target 1 task: 1 1200 800 0  // 所有任务设为紧急投放（无具体需求）
Target 2 task: 1 300 450 0
Target 3 task: 1 950 200 0
Target 4 task: 1 600 1200 0
Target 5 task: 1 1500 500 0

Enter drone details (x y remain_power max_power charge_time load_capacity
hover_capacity):
Drone 1: 0 0 9999 9999 0 0 0
Drone 2:0 0 9999 9999 0 0 0
Drone 3: 0 0 9999 9999 0 0 0

Enter number of obstacles: 0


5 3
1200 800 0
300 450 0
950 200 0
600 1200 0
1500 500 0
1 1200 800 0
1 300 450 0
1 950 200 0
1 600 1200 0
1 1500 500 0
0 0 9999 9999 0 0 0
0 0 9999 9999 0 0 0
0 0 9999 9999 0 0 0
0


[(1)结果]
===== Optimal Task Assignment =====
Total weighted cost: 16417
----------------------------------
Drone 1 tasks: 2(P1)
Drone 2 tasks: 3(P1) 5(P1) 1(P1) 4(P1)
Drone 3 tasks: No tasks assigned
----------------------------------
Target coverage by priority:
Emergency (P1): 5
Normal (P2): 0
Recon (P3): 0

(2)
Enter number of targets: 5
Enter number of drones: 3

Enter target positions (x y for each target):
Target 1: 1200 800
Target 2: 300 450
Target 3: 950 200
Target 4: 600 1200
Target 5: 1500 500

Enter task details for each target (priority x y requirement):
Priority: 1=Emergency, 2=Normal, 3=Recon
Target 1 task: 1 1200 800 0  // 所有任务设为紧急投放（无具体需求）
Target 2 task: 1 300 450 0
Target 3 task: 1 950 200 0
Target 4 task: 1 600 1200 0
Target 5 task: 1 1500 500 0

Enter drone details (x y remain_power max_power charge_time load_capacity
hover_capacity):
Drone 1: 0 0 9999 9999 0 0 0
Drone 2:0 0 9999 9999 0 0 0
Drone 3: 0 0 9999 9999 0 0 0

Enter number of obstacles: 1
Obstacle 1: 900 250 100 100



(3)
Enter number of targets: 10
Enter number of drones: 3

Enter target positions (x y for each target):
Target 1: 1200 800
Target 2: 300 450
Target 3: 950 200
Target 4: 600 1200
Target 5: 1500 500
Target 6: 400 1000
Target 7: 800 300
Target 8: 1300 1000
Target 9: 200 600
Target 10: 1000 1100

Enter task details for each target (priority x y requirement):
Priority: 1=Emergency, 2=Normal, 3=Recon
Target 1 task: 1 1200 800 10
Target 2 task: 2 300 450 6 
Target 3 task: 1 950 200 8
Target 4 task: 3 600 1200 50
Target 5 task: 2 1500 500 12
Target 6 task: 3 400 1000 40
Target 7 task: 1 800 300 7
Target 8 task: 3 1300 1000 60
Target 9 task: 2 200 600 5
Target 10 task: 1 1000 1100 9

Enter drone details (x y remain_power max_power charge_time load_capacity
hover_capacity): 
Drone 1: 0 0 500 500 60 15 30 
Drone 2: 0 0 600 600 60 10 60
Drone 3: 0 0 450 450 60 20 20

Enter number of obstacles: 0

*/