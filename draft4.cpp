// 还有IO，swarm, fitness三部分
#include <bits/stdc++.h>
using namespace std;

// 结构体定义
struct position_coord {
    double x, y;
};

enum TaskType { EMERGENCY_DELIVERY = 1, NORMAL_DELIVERY = 2, RECON = 3 };

struct task {
    TaskType priority;
    double x, y;
    int num;
    union {
        int load;
        int hover_time;
    } requirement;
};
struct UAV {
    position_coord position;  // 当前位置
    double working_time;      // 工作时间（初始化为0，开始充电时重置为-60）
    int max_working_time;     // 最大工作时间
    int load_capacity;        // 载荷能力
    int hover_capacity;       // 悬停能力
    double hovering_time;     // 侦察任务倒计时
};
struct Circle {
    double x, y, r;
};
vector<Circle> obstacles;
struct candidate {
    vector<pair<int, int>> assignment;
    /*
    在assignment中，每个元素表示一个任务，其中的前target_num个任务是需要去某个目标点完成的投递或侦查任务，
    后面的若干的任务是需要去原点完成的充电任务。
    每个任务的第一个分量表示这个任务由第几个无人机完成，第二个分量表示这个任务在全局所有任务中的排序
    （无人机编号和任务序号都是从1开始的）
    */
    double cost;
};

// 全局变量
int drone_num, target_num;
vector<UAV> uavs;
vector<task> tasks;
vector<position_coord> targets;
RandomGenerator rng;
const int INF = 4e5;           // 表示无穷大的值
const int speed = 50;          // 无人机速度（米/秒）
const int max_dist = 1000;     // 无人机间最大距离（米）
const int min_dist = 50;       // 无人机间最小安全距离（米）
const int epsilon = 10;        // 接近阈值（米）
const double time_step = 0.2;  // 仿真时间步长（秒）
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

int main() {
    input();
    candidate best_candidate = GA();
    output(best_candidate);
    return 0;
}

double getPriorityWeight(TaskType priority) {
    switch (priority) {
        case EMERGENCY_DELIVERY:
            return 1;
        case NORMAL_DELIVERY:
            return 2;
        case RECON:
            return 3;
        default:
            return 1.0;
    }
}
}  // namespace Utils

// 输入输出函数

candidate GA() {
    int max_gen = 15 * drone_num + 50;                // 最大迭代次数
    int pop_size = 10 * drone_num;                    // 种群大小
    int reserved_candidates = drone_num;              // 保留的精英个体数
    int unreserved = pop_size - reserved_candidates;  // 非精英个体数
    int task_mutations = 3 * drone_num;               // 任务变异次数
    int path_exchanges = 6 * drone_num;               // 路径交换次数
    int sequence_exchanges = 3 * drone_num;           // 序列交换次数
    int charge_mutations = 2 * drone_num;             // 充电任务变异次数
    int default_charges = 1;                          // 每架飞机的初始充电次数

    // 初始化种群
    vector<candidate> population(pop_size);
    for (int i = 0; i < pop_size; i++) {
        candidate candi;
        // 目标任务分配
        for (int j = 1; j <= target_num; j++) {
            candi.assignment.push_back({rng.randomInt(0, drone_num - 1), j});
        }
        // 添加初始充电任务
        for (int k = 0; k < default_charges * drone_num; k++) {
            candi.assignment.push_back({rng.randomInt(0, drone_num - 1), 0});
        }
        // 重新分配任务ID
        updateTaskIds(candi);
        candi.cost = INF;
        population[i] = candi;
    }

    candidate best_candidate;
    best_candidate.cost = INF;

    // 评估初始种群
    for (int i = 0; i < pop_size; i++) {
        population[i].cost = calculateCost(population[i]);
    }
    sort(
        population.begin(), population.end(),
        [](const candidate& a, const candidate& b) { return a.cost < b.cost; });
    best_candidate = population[0];

    // 遗传算法主循环
    for (int gen = 1; gen <= max_gen; gen++) {
        vector<candidate> temp_population(pop_size);

        // 精英保留
        for (int i = 0; i < reserved_candidates; i++) {
            temp_population[i] = population[i];
        }

        // 锦标赛选择
        for (int i = reserved_candidates; i < pop_size; i++) {
            vector<int> indices(3);
            for (int j = 0; j < 3; j++) {
                indices[j] = rng.randomInt(0, pop_size - 1);
            }
            int best_idx = indices[0];
            for (int j = 1; j < 3; j++) {
                if (population[indices[j]].cost < population[best_idx].cost) {
                    best_idx = indices[j];
                }
            }
            temp_population[i] = population[best_idx];
        }

        // 变异操作
        // 任务变异：随机改变任务的无人机分配
        for (int i = 0; i < task_mutations; i++) {
            int ind = reserved_candidates + rng.randomInt(0, unreserved - 1);
            int task_idx =
                rng.randomInt(0, temp_population[ind].assignment.size() - 1);
            temp_population[ind].assignment[task_idx].first =
                rng.randomInt(0, drone_num - 1);
        }

        // 充电任务变异
        for (int i = 0; i < charge_mutations; i++) {
            int ind = reserved_candidates + rng.randomInt(0, unreserved - 1);
            candidate& c = temp_population[ind];
            bool add = (rng.random() < 0.5);
            if (add) {
                c.assignment.push_back({rng.randomInt(0, drone_num - 1), 0});
            } else {
                vector<int> charge_indices;
                for (int j = 0; j < c.assignment.size(); ++j) {
                    if (c.assignment[j].second > target_num) {
                        charge_indices.push_back(j);
                    }
                }
                if (!charge_indices.empty()) {
                    int idx = charge_indices[rng.randomInt(
                        0, charge_indices.size() - 1)];
                    c.assignment.erase(c.assignment.begin() + idx);
                }
            }
            updateTaskIds(c);
        }

        // 路径交换：选择操作A或B
        for (int i = 0; i < path_exchanges; i++) {
            int ind1 = reserved_candidates + rng.randomInt(0, unreserved - 1);
            int ind2 = reserved_candidates + rng.randomInt(0, unreserved - 1);
            while (ind2 == ind1) {
                ind2 = reserved_candidates + rng.randomInt(0, unreserved - 1);
            }
            candidate& c1 = temp_population[ind1];
            candidate& c2 = temp_population[ind2];

            // 按task_id排序
            auto sort_by_id = [](const pair<int, int>& a,
                                 const pair<int, int>& b) {
                return a.second < b.second;
            };
            vector<pair<int, int>> sorted_c1 = c1.assignment;
            vector<pair<int, int>> sorted_c2 = c2.assignment;
            sort(sorted_c1.begin(), sorted_c1.end(), sort_by_id);
            sort(sorted_c2.begin(), sorted_c2.end(), sort_by_id);

            // 随机选择操作
            bool do_A = (rng.random() < 0.5);
            int n = rng.randomInt(1, min(sorted_c1.size(), sorted_c2.size()));

            vector<pair<int, int>> new_c1 = sorted_c1;
            vector<pair<int, int>> new_c2 = sorted_c2;

            if (do_A) {
                // 交换前n个无人机ID
                for (int j = 0; j < n; j++) {
                    swap(new_c1[j].first, new_c2[j].first);
                }
            } else {
                // 交换后n个无人机ID
                int start1 = sorted_c1.size() - n;
                int start2 = sorted_c2.size() - n;
                if (start1 >= 0 && start2 >= 0) {
                    for (int j = 0; j < n; j++) {
                        swap(new_c1[start1 + j].first,
                             new_c2[start2 + j].first);
                    }
                }
            }

            // 重新分配task_id
            updateTaskIds(c1);
            updateTaskIds(c2);
        }

        // 序列交换：交换相邻任务的顺序
        for (int i = 0; i < sequence_exchanges; i++) {
            int ind = reserved_candidates + rng.randomInt(0, unreserved - 1);
            int pos =
                rng.randomInt(0, temp_population[ind].assignment.size() - 2);
            swap(temp_population[ind].assignment[pos].second,
                 temp_population[ind].assignment[pos + 1].second);
        }

        // 评估新种群
        for (int i = 0; i < pop_size; i++) {
            temp_population[i].cost =
                calculateCost(temp_population[i]);
        }
        sort(temp_population.begin(), temp_population.end(),
             [](const candidate& a, const candidate& b) {
                 return a.cost < b.cost;
             });

        // 更新最佳候选
        if (temp_population[0].cost < best_candidate.cost) {
            best_candidate = temp_population[0];
        }

        population.swap(temp_population);
    }

    return best_candidate;
}

// GA的辅助函数：重新分配任务ID
void updateTaskIds(candidate& cand) {
    vector<pair<int, int>> sorted_tasks;
    // 分离目标任务和充电任务
    vector<pair<int, int>> targets;
    vector<pair<int, int>> charges;
    for (auto& task : cand.assignment) {
        if (task.second <= target_num) {
            targets.push_back(task);
        } else {
            charges.push_back(task);
        }
    }
    // 合并并排序
    sorted_tasks.insert(sorted_tasks.end(), targets.begin(), targets.end());
    sorted_tasks.insert(sorted_tasks.end(), charges.begin(), charges.end());

    // 重新分配task_id
    int tid = 1;
    for (auto& task : sorted_tasks) {
        task.second = tid++;
    }
    // 保持原顺序（按task_id排序）
    sort(sorted_tasks.begin(), sorted_tasks.end(),
         [](const pair<int, int>& a, const pair<int, int>& b) {
             return a.second < b.second;
         });
    cand.assignment = sorted_tasks;
}

// calculateCost的辅助函数：判断可行性
bool feasibility(const candidate& candidate1) {
    std::vector<std::vector<int>> drone_tasks(drone_num +
                                              1);  // 1-based无人机编号
    for (const auto& p : candidate1.assignment) {
        int drone_id = p.first;
        int task_id = p.second;
        drone_tasks[drone_id].push_back(task_id);
    }

    for (int drone_id = 1; drone_id <= drone_num; ++drone_id) {
        const auto& tasks_list = drone_tasks[drone_id];
        if (tasks_list.empty())
            continue;  // 无任务则跳过

        int start = 0;  // 当前航行段的起始任务索引

        while (start < tasks_list.size()) {
            int end = start;  // 当前航行段的结束任务索引

            // 找到充电任务或任务队列末尾
            while (end < tasks_list.size() && tasks_list[end] <= target_num) {
                end++;
            }

            int total_load = 0;                    // 当前航行段的总载荷
            double total_distance = 0;             // 当前航行段的总航程
            position_coord prev_pos = {0.0, 0.0};  // 起点为原点

            // 计算当前航行段的载荷和航程
            for (int i = start; i < end; ++i) {
                int task_idx = tasks_list[i] - 1;  // 转换为0-based任务索引
                const task& t = tasks[task_idx];

                // 计算载荷（仅投递任务需要载荷）
                if (t.priority == EMERGENCY_DELIVERY ||
                    t.priority == NORMAL_DELIVERY) {
                    total_load += t.requirement.load;
                }

                // 计算航程（使用distance函数）
                position_coord current_pos = {t.x, t.y};
                total_distance += distance(prev_pos, current_pos);
                prev_pos = current_pos;  // 更新前一个位置为当前位置
            }

            // 检查约束条件
            if (total_load > uavs[drone_id - 1].load_capacity) {
                return false;  // 载荷超限
            }
            if (total_distance > 30000.0) {
                return false;  // 航程超限
            }

            start = end;  // 移动到下一个航行段的起始位置
        }
    }
    return true;  // 所有条件满足
}

double calculateCost(candidate candidate1) {
    /**
     * @brief 计算候选方案的总成本
     *
     * @param candidate1 要评估的候选方案
     * @return 双精度浮点数，表示总成本（INF表示无效方案）
     *
     * 总成本公式：
     * cost = 5 * T1（所有紧急投递完成时间） + 2 * T2（所有普通投递完成时间） +
     * T3（所有侦查完成时间）
     *
     * 模拟流程：
     * 1. 初始化无人机状态
     * 2. 每0.2秒调用swarm函数获取移动方向
     * 3. 更新无人机位置、工作时间、悬停时间
     * 4. 检查任务完成条件并记录时间
     */

    // 第一步：可行性检查
    if (!feasibility(candidate1))
        return INF;

    // 第二步：复制原始无人机状态（避免修改全局变量）
    std::vector<UAV> current_uavs(uavs.begin(), uavs.end());

    // 第三步：解析任务分配（按无人机编号存储任务列表）
    std::vector<std::vector<int>> drone_tasks(drone_num +
                                              1);  // 1-based无人机编号
    for (const auto& p : candidate1.assignment) {
        int drone_id = p.first;
        int task_id = p.second;
        drone_tasks[drone_id].push_back(task_id);
    }

    // 第四步：初始化无人机状态结构
    struct UAV_state {
        UAV uav;                        // 当前无人机状态
        std::vector<int> task_queue;    // 该无人机的任务列表
        int current_task_index = 0;     // 当前任务在队列中的索引
        position_coord current_target;  // 当前目标点坐标
    };
    std::vector<UAV_state> states(drone_num);

    // 初始化每个无人机的状态
    for (int i = 0; i < drone_num; ++i) {
        states[i].uav = current_uavs[i];            // 复制初始状态
        states[i].task_queue = drone_tasks[i + 1];  // drone_id是i+1（1-based）

        // 设置初始目标点
        if (!states[i].task_queue.empty()) {
            int first_task_id = states[i].task_queue[0];
            if (first_task_id <= target_num) {
                // 目标点任务（坐标来自任务列表）
                const task& first_task = tasks[first_task_id - 1];
                states[i].current_target = {(double)first_task.x, first_task.y};
            } else {
                // 充电任务（目标点为原点）
                states[i].current_target = {0.0, 0.0};
            }
        } else {
            // 无任务则默认目标点为原点
            states[i].current_target = {0.0, 0.0};
        }
    }

    // 第五步：初始化计数器和时间变量
    int total_tasks = tasks.size();
    int accomplished = 0;
    double emergency_time = 0.0, normal_time = 0.0, recon_time = 0.0;
    double current_time = 0.0;

    // 第六步：主模拟循环（直到所有任务完成）
    while (accomplished < total_tasks) {
        // 检查是否有无人机电量耗尽
        bool any_fail = false;
        for (const auto& s : states) {
            if (s.uav.working_time > s.uav.max_working_time) {
                any_fail = true;
                break;
            }
        }
        if (any_fail)
            return INF;

        // 获取无人机移动方向（由swarm函数计算）
        std::vector<double> directions = swarm();  // 假设返回弧度制方向数组

        // 更新每个无人机的状态
        for (int i = 0; i < drone_num; ++i) {
            UAV_state& s = states[i];
            UAV& uav = s.uav;

            // 更新悬停时间（无论是否移动，每0.2秒增加）
            uav.hovering_time += time_step;

            // 更新工作时间（条件判断见下文）
            if (uav.position.x == 0.0 && uav.position.y == 0.0 &&
                uav.working_time == 0.0) {
                // 在原点且工作时间为0时不增加
            } else {
                uav.working_time += time_step;
            }

            // 判断是否可移动（不可移动的条件见用户要求）
            bool is_movable = true;
            if (uav.working_time < 0) {  // 充电状态
                is_movable = false;
            } else if (uav.hovering_time < 0) {  // 悬停状态
                is_movable = false;
            } else if (uav.working_time == 0) {  // 需检查是否有更早的无人机
                // 如果存在编号更小的无人机也处于 working_time == 0，则不可移动
                for (int j = 0; j < i; ++j) {
                    if (states[j].uav.working_time == 0) {
                        is_movable = false;
                        break;
                    }
                }
            }

            if (is_movable) {  // 可移动的无人机更新位置
                // 根据方向移动（速度为50米/秒，时间步长0.2秒）
                double dir = directions[i];
                double dx = speed * time_step * std::cos(dir);
                double dy = speed * time_step * std::sin(dir);
                uav.position.x += dx;
                uav.position.y += dy;

                // 检查是否到达目标点
                position_coord target = s.current_target;
                double distance_to_target =
                    distance(uav.position, target);  // 调用封装的距离函数

                if (distance_to_target <= epsilon) {  // 到达目标点
                    if (target.x == 0.0 && target.y == 0.0) {
                        // 充电任务完成，开始充电
                        uav.working_time = -60.0;
                    } else {
                        // 处理目标点任务
                        int task_id = s.task_queue[s.current_task_index];
                        const task& t = tasks[task_id - 1];  // 转换为0-based

                        switch (t.priority) {
                            case RECON:  // 侦查任务
                                uav.hovering_time = -t.requirement.hover_time;
                                break;
                            case EMERGENCY_DELIVERY:
                            case NORMAL_DELIVERY:  // 投递任务
                                // 记录任务完成时间
                                if (t.priority == EMERGENCY_DELIVERY) {
                                    emergency_time =
                                        std::max(emergency_time, current_time);
                                } else {
                                    normal_time =
                                        std::max(normal_time, current_time);
                                }
                                // 更新已完成任务数
                                accomplished++;
                                // 切换到下一个任务
                                s.current_task_index++;
                                if (s.current_task_index <
                                    s.task_queue.size()) {
                                    int next_task_id =
                                        s.task_queue[s.current_task_index];
                                    if (next_task_id <= target_num) {
                                        // 下一个任务是目标点任务
                                        const task& next_t =
                                            tasks[next_task_id - 1];
                                        s.current_target = {next_t.x, next_t.y};
                                    } else {
                                        // 下一个任务是充电任务
                                        s.current_target = {0.0, 0.0};
                                    }
                                }
                                break;
                        }
                    }
                }
            }
        }

        // 更新当前时间（时间步长0.2秒）
        current_time += time_step;
    }

    // 第七步：计算侦查任务的最晚完成时间
    for (const auto& s : states) {
        for (int task_id : s.task_queue) {
            const task& t = tasks[task_id - 1];
            if (t.priority == RECON) {
                recon_time = std::max(recon_time, current_time);
            }
        }
    }

    // 返回总成本
    return 5 * emergency_time + 2 * normal_time + recon_time;
}

// calculateCost的辅助函数：计算等效距离
double distance(const position_coord& a, const position_coord& b) {
    // 当前实现为欧氏距离（勾股定理）
    return std::hypot(a.x - b.x, a.y - b.y);
    // 未来扩展：可添加障碍物绕行路径计算，例如：
    // return path_planning_with_obstacles(a, b, obstacles);
}
