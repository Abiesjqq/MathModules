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
    double cost;
};

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

// 全局变量
extern int drone_num, target_num;
extern vector<UAV> uavs;
extern vector<task> tasks;
extern vector<position_coord> targets;
extern RandomGenerator rng;
extern const int INF;
extern const int speed;
extern const int max_dist;
extern const int min_dist;
extern const int epsilon;
extern const double time_step;

// 函数声明
void input();
void output(const candidate& best_candidate);
double getPriorityWeight(TaskType priority);
candidate GA();
void updateTaskIds(candidate& cand);
bool feasibility(const candidate& candidate1);
double calculateCost(candidate candidate1);
bool doesSegmentIntersectCircle(const position_coord& A,
                                const position_coord& B,
                                const Circle& Z);
vector<position_coord> getTangentPoints(const position_coord& P,
                                        const Circle& Z);
double arcLength(const position_coord& C,
                 const position_coord& D,
                 const Circle& Z);
double distance(const position_coord& a, const position_coord& b);
vector<double> swarm(const vector<UAV>& initial_uavs,
                     const candidate& candidate1,
                     int accomplished);
double calculateFitness(const vector<UAV>& uavs,
                        const candidate candidate1,
                        int accomplished);
bool isStateLegal(const vector<UAV>& uavs,
                  int accomplished,
                  const candidate& candidate1);

// 全局变量定义
int drone_num, target_num;
vector<UAV> uavs;
vector<task> tasks;
vector<position_coord> targets;
RandomGenerator rng;
const int INF = 4e7;
const int speed = 50;
const int max_dist = 1000;
const int min_dist = 50;
const int epsilon = 10;
const double time_step = 0.2;

// 其余代码保持不变...

void input() {
    // 读取无人机数量
    cin >> drone_num;
    uavs.resize(drone_num);

    // 读取每架无人机的参数
    for (int i = 0; i < drone_num; ++i) {
        cin >> uavs[i].load_capacity >> uavs[i].max_working_time >>
            uavs[i].hover_capacity;
        uavs[i].position = {0.0, 0.0};  // 初始位置为原点
        uavs[i].working_time = 0.0;
        uavs[i].hovering_time = 0.0;
    }

    // 读取目标点数量
    cin >> target_num;
    tasks.resize(target_num);

    // 读取每个任务的参数
    for (int i = 0; i < target_num; ++i) {
        string type_str;
        cin >> type_str >> tasks[i].x >> tasks[i].y;

        // 设置任务类型
        if (type_str == "EMERGENCY_DELIVERY") {
            tasks[i].priority = EMERGENCY_DELIVERY;
            cin >> tasks[i].requirement.load;
        } else if (type_str == "NORMAL_DELIVERY") {
            tasks[i].priority = NORMAL_DELIVERY;
            cin >> tasks[i].requirement.load;
        } else if (type_str == "RECON") {
            tasks[i].priority = RECON;
            cin >> tasks[i].requirement.hover_time;
        }
        tasks[i].num = i + 1;  // 任务编号从1开始
    }

    // 读取障碍物数量
    int obstacle_num;
    cin >> obstacle_num;
    obstacles.resize(obstacle_num);

    // 读取每个障碍物的参数
    for (int i = 0; i < obstacle_num; ++i) {
        cin >> obstacles[i].x >> obstacles[i].y >> obstacles[i].r;
    }

    printf("Input completed. Number of drones: %d, Number of targets: %d\n",
           drone_num, target_num);

    // 初始化随机数生成器
    rng = RandomGenerator();
}

void output(const candidate& best_candidate) {
    // 打开输出文件
    ofstream outfile("drone_paths.csv");
    if (!outfile.is_open()) {
        cerr << "Error opening output file!" << endl;
        return;
    }

    // 1. 复制原始无人机状态（避免修改全局变量）
    vector<UAV> current_uavs = uavs;

    // 2. 解析任务分配（按无人机编号存储任务列表）
    vector<vector<int>> drone_tasks(drone_num + 1);  // 1-based无人机编号
    for (const auto& p : best_candidate.assignment) {
        int drone_id = p.first + 1;  // 转换为1-based
        int task_id = p.second;
        drone_tasks[drone_id].push_back(task_id);
    }

    // 3. 初始化无人机状态结构
    struct UAV_state {
        UAV uav;                        // 当前无人机状态
        vector<int> task_queue;         // 该无人机的任务列表
        int current_task_index = 0;     // 当前任务在队列中的索引
        position_coord current_target;  // 当前目标点坐标
        vector<position_coord> path;    // 记录完整飞行路径
    };
    vector<UAV_state> states(drone_num);

    // 初始化每个无人机的状态
    for (int i = 0; i < drone_num; ++i) {
        states[i].uav = current_uavs[i];
        states[i].task_queue = drone_tasks[i + 1];  // drone_id是i+1（1-based）
        states[i].path.push_back({0.0, 0.0});       // 初始位置

        // 设置初始目标点
        if (!states[i].task_queue.empty()) {
            int first_task_id = states[i].task_queue[0];
            if (first_task_id <= target_num) {
                // 目标点任务
                const task& first_task = tasks[first_task_id - 1];
                states[i].current_target = {first_task.x, first_task.y};
            } else {
                // 充电任务
                states[i].current_target = {0.0, 0.0};
            }
        } else {
            states[i].current_target = {0.0, 0.0};
        }
    }

    // 4. 模拟飞行过程
    double current_time = 0.0;
    int total_tasks = tasks.size();
    int accomplished = 0;

    // 输出文件头（CSV格式，MATLAB可读）
    outfile << "Time(s)";
    for (int i = 0; i < drone_num; ++i) {
        outfile << ",UAV" << i + 1 << "_x,UAV" << i + 1 << "_y";
    }
    outfile << endl;

    // 每0.2秒记录一次位置
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
            break;

        // 获取无人机移动方向
        vector<double> directions =
            swarm(current_uavs, best_candidate, accomplished);  // 计算移动方向

        // 更新每个无人机的状态
        for (int i = 0; i < drone_num; ++i) {
            UAV_state& s = states[i];
            UAV& uav = s.uav;

            // 更新悬停时间
            uav.hovering_time += time_step;

            // 更新工作时间
            if (!(uav.position.x == 0.0 && uav.position.y == 0.0 &&
                  uav.working_time == 0.0)) {
                uav.working_time += time_step;
            }

            // 判断是否可移动
            bool is_movable = true;
            if (uav.working_time < 0) {
                is_movable = false;
            } else if (uav.hovering_time < 0) {
                is_movable = false;
            } else if (uav.working_time == 0) {
                for (int j = 0; j < i; ++j) {
                    if (states[j].uav.working_time == 0) {
                        is_movable = false;
                        break;
                    }
                }
            }

            if (is_movable) {
                // 移动无人机
                double dir = directions[i];
                double dx = speed * time_step * cos(dir);
                double dy = speed * time_step * sin(dir);
                uav.position.x += dx;
                uav.position.y += dy;

                // 记录路径点
                s.path.push_back(uav.position);

                // 检查是否到达目标点
                position_coord target = s.current_target;
                double distance_to_target =
                    sqrt(pow(uav.position.x - target.x, 2) +
                         pow(uav.position.y - target.y, 2));

                if (distance_to_target <= epsilon)

                    /*
                    {
                        if (target.x == 0.0 && target.y == 0.0) {
                            // 充电任务
                            uav.working_time = -60.0;
                        } else {
                            // 处理目标点任务
                            int task_id = s.task_queue[s.current_task_index];
                            const task& t = tasks[task_id - 1];

                            if (t.priority == RECON) {
                                uav.hovering_time = -t.requirement.hover_time;
                            } else {
                                accomplished++;
                                s.current_task_index++;
                                if (s.current_task_index < s.task_queue.size())
                    { int next_task_id = s.task_queue[s.current_task_index]; if
                    (next_task_id <= target_num) { const task& next_t =
                                            tasks[next_task_id - 1];
                                        s.current_target = {next_t.x, next_t.y};
                                    } else {
                                        s.current_target = {0.0, 0.0};
                                    }
                                }
                            }
                        }
                    }

                    */
                    //
                    if (target.x == 0.0 && target.y == 0.0) {
                        // 充电任务
                        uav.working_time = -60.0;
                    } else {
                        // 处理目标点任务
                        int task_id = s.task_queue[s.current_task_index];
                        const task& t = tasks[task_id - 1];

                        if (t.priority == RECON) {
                            uav.hovering_time = -t.requirement.hover_time;
                        } else {
                            accomplished++;
                            s.current_task_index++;
                            if (s.current_task_index < s.task_queue.size()) {
                                int next_task_id =
                                    s.task_queue[s.current_task_index];
                                if (next_task_id <= target_num) {
                                    const task& next_t =
                                        tasks[next_task_id - 1];
                                    s.current_target = {next_t.x, next_t.y};
                                } else {
                                    s.current_target = {0.0, 0.0};
                                }
                            }
                        }
                    }

            } else {
                // 不可移动时保持当前位置
                s.path.push_back(uav.position);
            }
        }

        // 输出当前时刻所有无人机的位置到文件
        outfile << fixed << setprecision(1) << current_time;
        for (const auto& s : states) {
            outfile << "," << s.uav.position.x << "," << s.uav.position.y;
        }
        outfile << endl;

        current_time += time_step;
    }

    // 关闭文件
    outfile.close();

    // 可选：输出每架无人机的完整路径到单独文件（MATLAB可读）
    for (int i = 0; i < drone_num; ++i) {
        ofstream drone_file("UAV" + to_string(i + 1) + "_path.csv");
        drone_file << "x,y" << endl;  // 表头
        for (const auto& pos : states[i].path) {
            drone_file << pos.x << "," << pos.y << endl;
        }
        drone_file.close();
    }

    cout << "Drone paths have been saved to CSV files for MATLAB visualization."
         << endl;
}

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

    // ----------------------------------------------------
    printf("Population initialized with %d candidates.\n", pop_size);

    candidate best_candidate;
    best_candidate.cost = INF;

    // 评估初始种群
    for (int i = 0; i < pop_size; i++) {
        population[i].cost = calculateCost(population[i]);
    }

    // ---------------------------------
    printf("Initial population evaluated.\n");

    sort(
        population.begin(), population.end(),
        [](const candidate& a, const candidate& b) { return a.cost < b.cost; });
    best_candidate = population[0];

    //----------------------------------------
    printf("Initial population evaluated. Best cost = %.2f\n",
           best_candidate.cost);

    // 遗传算法主循环
    for (int gen = 1; gen <= max_gen; gen++) {
        printf("Generation %d: Best cost = %.2f\n", gen, best_candidate.cost);
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
            temp_population[i].cost = calculateCost(temp_population[i]);
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
            if (end == start) {
                // 当前航段为空，直接跳过。
                start = end + 1;
                continue;
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
                printf("Task %d: Load = %d\n", t.num, total_load);

                // 计算航程（使用distance函数）
                position_coord current_pos = {t.x, t.y};
                total_distance += distance(prev_pos, current_pos);
                printf("Task %d: Distance = %.2f\n", t.num, total_distance);
                // 检查与障碍物的碰撞
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
    printf("Feasibility check passed for candidate.\n");
    return true;  // 所有条件满足
}

double calculateCost(candidate candidate1) {
    // ----------------------
    printf("Calculating cost for candidate...\n");

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
    if (!feasibility(candidate1)) {
        printf("Feasibility check failed.\n");
        return INF;
    }

    // -------------
    printf("Feasibility check passed.\n");

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

    // -------------
    printf("UAV states initialized.\n");

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

    // -------------------------
    printf("All UAV states initialized.\n");

    // 第五步：初始化计数器和时间变量
    int total_tasks = tasks.size();
    int accomplished = 0;
    double emergency_time = 0.0, normal_time = 0.0, recon_time = 0.0;
    double current_time = 0.0;

    // ----------------------------
    printf("Simulation started.\n");

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
        std::vector<double> directions =
            swarm(current_uavs, candidate1, accomplished);

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

        // --------------------
        printf("Time: %.2f    ", current_time);
        printf("Accomplished tasks: %d\n", accomplished);

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

// distance的辅助函数：判断线段与障碍的相交性
bool doesSegmentIntersectCircle(const position_coord& A,
                                const position_coord& B,
                                const Circle& Z) {
    double xc = Z.x;
    double yc = Z.y;
    double r = Z.r;
    double Ax = A.x, Ay = A.y;
    double Bx = B.x, By = B.y;
    double dx = Bx - Ax;
    double dy = By - Ay;
    double ax = Ax - xc;
    double ay = Ay - yc;
    double a = dx * dx + dy * dy;
    double b = 2 * (dx * ax + dy * ay);
    double c = ax * ax + ay * ay - r * r;
    double discriminant = b * b - 4 * a * c;

    if (discriminant < 0)
        return false;
    if (a == 0) {
        return (ax * ax + ay * ay < r * r);
    }

    double sqrtD = sqrt(discriminant);
    double t1 = (-b - sqrtD) / (2 * a);
    double t2 = (-b + sqrtD) / (2 * a);

    bool intersect = false;

    if (discriminant > 0) {
        if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)) {
            intersect = true;
        }
    } else if (discriminant == 0) {
        intersect = false;
    }

    return intersect;
}

// distance的辅助函数：取切点
std::vector<position_coord> getTangentPoints(const position_coord& P,
                                             const Circle& Z) {
    double xc = Z.x;
    double yc = Z.y;
    double r = Z.r;
    double px = P.x;
    double py = P.y;
    double dx = xc - px;
    double dy = yc - py;
    double d = sqrt(dx * dx + dy * dy);

    if (d < r) {
        return {};
    }

    double phi = atan2(dy, dx);
    double cos_delta = -r / d;
    double delta = acos(cos_delta);

    double theta1 = phi + delta;
    double theta2 = phi - delta;

    position_coord c1{xc + r * cos(theta1), yc + r * sin(theta1)};
    position_coord c2{xc + r * cos(theta2), yc + r * sin(theta2)};

    return {c1, c2};
}

// distance的辅助函数：求弧长
double arcLength(const position_coord& C,
                 const position_coord& D,
                 const Circle& Z) {
    double xc = Z.x;
    double yc = Z.y;
    double dx1 = C.x - xc;
    double dy1 = C.y - yc;
    double dx2 = D.x - xc;
    double dy2 = D.y - yc;

    double dot = dx1 * dx2 + dy1 * dy2;
    double r = Z.r;

    double cos_theta = dot / (r * r);
    cos_theta = std::max(std::min(cos_theta, 1.0), -1.0);
    double theta = acos(cos_theta);

    double arc = r * theta;
    double min_arc = std::min(arc, r * (2 * M_PI - theta));
    return min_arc;
}

// 计算等效距离
double distance(const position_coord& a, const position_coord& b) {
    for (const auto& obstacle : obstacles) {
        if (doesSegmentIntersectCircle(a, b, obstacle)) {
            const Circle& Z = obstacle;

            auto A_tangents = getTangentPoints(a, Z);
            auto B_tangents = getTangentPoints(b, Z);

            if (A_tangents.empty() || B_tangents.empty()) {
                return std::numeric_limits<double>::infinity();
            }

            double min_total = INFINITY;
            position_coord c_best, d_best;

            for (const auto& c : A_tangents) {
                for (const auto& d : B_tangents) {
                    double current_arc = arcLength(c, d, Z);
                    double total =
                        distance(a, c) + current_arc + distance(d, b);
                    if (total < min_total) {
                        min_total = total;
                        c_best = c;
                        d_best = d;
                    }
                }
            }

            return min_total;
        }
    }

    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

vector<double> swarm(const vector<UAV>& initial_uavs,
                     const candidate& candidate1,
                     int accomplished) {
    vector<bool> movable(drone_num, true);  // 标记无人机是否可移动
    vector<int> movable_indices;            // 可移动无人机的索引

    // 确定可移动的无人机
    for (int i = 0; i < drone_num; ++i) {
        const UAV& uav = initial_uavs[i];
        // 不可移动的条件判断
        if (uav.working_time < 0 || uav.hovering_time < 0) {
            movable[i] = false;
        } else if (uav.working_time == 0) {
            // 检查是否存在编号更小的无人机工作时间为0
            for (int j = 0; j < i; ++j) {
                if (initial_uavs[j].working_time == 0) {
                    movable[i] = false;
                    break;
                }
            }
        }
        if (movable[i]) {
            movable_indices.push_back(i);
        }
    }

    int movable_drones_count = movable_indices.size();
    if (movable_drones_count == 0) {
        // 无无人机可移动，返回全0角度
        return vector<double>(drone_num, 0.0);
    }

    // 粒子群算法参数（动态计算）
    const int num_particles = 4 * drone_num + 10;  // 粒子数量
    const int max_iter = 10 * drone_num + 30;      // 最大迭代次数
    const double w = 0.7;                          // 惯性权重
    const double c1 = 1.5;                         // 认知系数
    const double c2 = 1.5;                         // 社会系数

    // 粒子结构体定义
    struct Particle {
        vector<double> position;  // 当前角度向量
        vector<double> velocity;
        vector<double> pbest_pos;  // 个人最优位置
        double pbest_fitness;

        Particle(int dim) {
            position.resize(dim);
            velocity.resize(dim);
            pbest_pos.resize(dim);
            // 初始化角度为随机值，速度初始化为0
            for (int i = 0; i < dim; ++i) {
                position[i] = rng.random() * 2 * M_PI;
                velocity[i] = 0.0;
                pbest_pos[i] = position[i];
            }
            pbest_fitness = INF;
        }
    };

    vector<Particle> particles;
    for (int i = 0; i < num_particles; ++i) {
        particles.emplace_back(
            movable_drones_count);  // 使用可移动无人机数量作为维度
    }

    // 全局最优位置和适应度
    vector<double> gbest_pos(movable_drones_count);
    double gbest_fitness = INF;

    // 粒子群迭代优化
    for (int iter = 0; iter < max_iter; ++iter) {
        for (auto& p : particles) {
            vector<UAV> new_uavs = initial_uavs;  // 复制初始无人机状态
            // 根据当前角度计算新位置
            for (int k = 0; k < movable_drones_count; ++k) {
                int drone_idx = movable_indices[k];
                double angle = p.position[k];
                // 计算位移（使用全局速度和时间步长）
                double dx = speed * cos(angle) * time_step;
                double dy = speed * sin(angle) * time_step;
                new_uavs[drone_idx].position.x += dx;
                new_uavs[drone_idx].position.y += dy;
            }

            // 调用适应度函数计算当前粒子的适应度
            double fitness =
                calculateFitness(new_uavs, candidate1, accomplished);

            // 更新个人最优和全局最优
            if (fitness < p.pbest_fitness) {
                p.pbest_fitness = fitness;
                p.pbest_pos = p.position;
                if (fitness < gbest_fitness) {
                    gbest_fitness = fitness;
                    gbest_pos = p.position;
                }
            }
        }

        // 更新粒子速度和位置
        for (auto& p : particles) {
            for (int i = 0; i < movable_drones_count; ++i) {
                // 生成随机数
                double r1 = rng.random();
                double r2 = rng.random();

                // 速度更新公式（PSO标准公式）
                double vel_c1 = c1 * r1 * (p.pbest_pos[i] - p.position[i]);
                double vel_c2 = c2 * r2 * (gbest_pos[i] - p.position[i]);
                double new_velocity = w * p.velocity[i] + vel_c1 + vel_c2;
                p.velocity[i] = new_velocity;

                // 更新位置并限制角度范围在[0, 2π)
                p.position[i] += p.velocity[i];
                while (p.position[i] < 0)
                    p.position[i] += 2 * M_PI;
                while (p.position[i] >= 2 * M_PI)
                    p.position[i] -= 2 * M_PI;
            }
        }
    }

    // 生成最终角度结果
    vector<double> result(drone_num, 0.0);
    for (int k = 0; k < movable_drones_count; ++k) {
        int idx = movable_indices[k];
        result[idx] = gbest_pos[k];
    }

    return result;
}

double calculateFitness(const vector<UAV>& uavs,
                        const candidate candidate1,
                        int accomplished) {
    /**
     * @brief 计算无人机编队状态的适应度值
     *
     * 函数首先检查无人机状态的合法性，若不合法返回INF。
     * 合法情况下计算两部分距离之和：
     * 1. 每个无人机到其下一个任务点的等效距离之和
     * 2. 全局下一个目标点到负责无人机的等效距离
     *
     * @param uavs 无人机状态数组
     * @param candidate1 当前任务分配方案
     * @param accomplished 已完成的目标数量
     * @return double 适应度值
     */

    // 判断当前状态是否合法，不合法直接返回无穷大
    if (!isStateLegal(uavs, accomplished, candidate1)) {
        return INF;
    }

    double total = 0.0;

    // 遍历所有无人机（编号从1到drone_num）
    for (int uav_id = 1; uav_id <= drone_num; ++uav_id) {
        int next_task_order = -1;  // 存储下一个任务的排序号

        // 查找该无人机的第一个未完成任务（排序号大于已完成数目的最小值）
        for (const auto& assignment : candidate1.assignment) {
            if (assignment.first == uav_id &&
                assignment.second > accomplished) {
                if (next_task_order == -1 ||
                    assignment.second < next_task_order) {
                    next_task_order = assignment.second;
                }
            }
        }

        // 如果没有找到下一个任务，跳过计算
        if (next_task_order == -1)
            continue;

        // 获取任务坐标（排序号从1开始，数组索引需减1）
        const task& next_task = tasks[next_task_order - 1];
        position_coord task_pos = {next_task.x, next_task.y};

        // 获取该无人机当前位置
        const UAV& uav = uavs[uav_id - 1];  // 数组0-based索引

        // 累加等效距离（调用distance函数）
        total += distance(uav.position, task_pos);
    }

    // 处理全局下一个目标点的特殊计算
    if (accomplished < target_num) {
        // 全局下一个目标点的排序号
        int global_next_order = accomplished + 1;

        // 查找负责该任务的无人机编号
        int target_uav_id = -1;
        for (const auto& assignment : candidate1.assignment) {
            if (assignment.second == global_next_order) {
                target_uav_id = assignment.first;
                break;
            }
        }

        // 如果找到有效无人机（理论上必能找到）
        if (target_uav_id != -1) {
            // 获取该无人机和目标点坐标
            const UAV& target_uav = uavs[target_uav_id - 1];
            const task& global_next_task = tasks[global_next_order - 1];
            position_coord global_task_pos = {global_next_task.x,
                                              global_next_task.y};

            // 累加等效距离
            total += distance(target_uav.position, global_task_pos);
        }
    }

    return total;
}

bool isStateLegal(const vector<UAV>& uavs,
                  int accomplished,
                  const candidate& candidate1) {
    /**
     * @brief 判断无人机状态是否合法
     *
     * 检查以下5种不合法情况：
     * 1. 已起飞无人机间距离小于50m
     * 2. 无人机间距离大于1000m
     * 3. 工作时间超限
     * 4. 与障碍物碰撞
     * 5. 不按顺序访问目标点
     *
     * @param uavs 无人机状态数组
     * @param accomplished 已完成的目标数量
     * @param candidate1 当前任务分配方案
     * @return true 状态合法
     * @return false 状态不合法
     */

    // 条件1：检查已起飞无人机间的最小安全距离
    for (size_t i = 0; i < uavs.size(); ++i) {
        for (size_t j = i + 1; j < uavs.size(); ++j) {
            const UAV& a = uavs[i];
            const UAV& b = uavs[j];
            if (a.working_time > 0 && b.working_time > 0) {
                double dx = a.position.x - b.position.x;
                double dy = a.position.y - b.position.y;
                double dist = sqrt(dx * dx + dy * dy);
                if (dist < min_dist)
                    return false;
            }
        }
    }

    // 条件2：检查无人机间的最大通信距离
    for (size_t i = 0; i < uavs.size(); ++i) {
        for (size_t j = i + 1; j < uavs.size(); ++j) {
            const UAV& a = uavs[i];
            const UAV& b = uavs[j];
            double dx = a.position.x - b.position.x;
            double dy = a.position.y - b.position.y;
            double dist = sqrt(dx * dx + dy * dy);
            if (dist > max_dist)
                return false;
        }
    }

    // 条件3：检查工作时间是否超限
    for (const UAV& uav : uavs) {
        if (uav.working_time > uav.max_working_time)
            return false;
    }

    // 条件4：检查与障碍物的碰撞
    for (const UAV& uav : uavs) {
        for (const Circle& obs : obstacles) {
            double dx = uav.position.x - obs.x;
            double dy = uav.position.y - obs.y;
            double dist = sqrt(dx * dx + dy * dy);
            if (dist <= obs.r)
                return false;
        }
    }

    // 条件5：检查不按顺序访问目标点
    if (accomplished < target_num) {
        // 获取全局下一个目标点的排序号
        int global_next_order = accomplished + 1;

        // 收集所有后续目标点坐标
        vector<position_coord> later_positions;
        for (int order = global_next_order + 1; order <= target_num; ++order) {
            const task& t = tasks[order - 1];
            later_positions.push_back({t.x, t.y});
        }

        // 查找负责全局下一个目标的无人机
        int assigned_uav_id = -1;
        for (const auto& p : candidate1.assignment) {
            if (p.second == global_next_order) {
                assigned_uav_id = p.first;
                break;
            }
        }

        // 检查其他无人机是否过早访问后续目标点
        for (size_t uav_idx = 0; uav_idx < uavs.size(); ++uav_idx) {
            int current_uav_id = uav_idx + 1;
            if (current_uav_id == assigned_uav_id)
                continue;  // 跳过负责无人机

            const UAV& uav = uavs[uav_idx];
            for (const position_coord& pos : later_positions) {
                double dx = uav.position.x - pos.x;
                double dy = uav.position.y - pos.y;
                double dist = sqrt(dx * dx + dy * dy);
                if (dist < epsilon)
                    return false;
            }
        }
    }

    return true;
}