clear; clc;

%% 输入数据
targets = [1200, 800;   % T1
           300,  450;   % T2
           950,  200;   % T3
           600, 1200;   % T4
           1500, 500];  % T5
m = 3;                  % 无人机数量
start_pos = [0, 0];     % 起飞点
speed = 50;
% rng(1);  % 固定随机种子以保证可重复性

obstacle_time = 100; % 暂定障碍出现时间为100
obstacle_radious = 100;
obstacle_position = [900, 250];

add_target_time = 100;
add_target_position = [800, 600];

% 参数设置
pop_size = 80; % 每次迭代的不同的任务分配方案数
max_gen = 100; % 最多迭代次数
mutation_rate = 0.1; % 变异概率
cross_rate = 0.5; % 交换概率

% 初始化种群
population = randi([1, m], pop_size, size(targets,1));
    % pop_size*targets矩阵，每一行代表一个任务分配方案 

%% 进化过程
for gen = 1:max_gen
    fitness = zeros(pop_size, 1);
    for i = 1:pop_size
        assignment = population(i,:);
        
        % 严格全覆盖检查
        uncovered = setdiff(1:size(targets,1), unique(assignment));
        if ~isempty(uncovered)
            fitness(i) = Inf + 1e6*length(uncovered);
        end
        
        % 路径优化计算
        [total_dist, paths] = optimize_paths(assignment, targets, start_pos);

        % 通信约束检查
        if ~check_communication(paths, speed)
            total_dist = Inf;
        end
        
        fitness(i) = total_dist;
    end
    
    % 精英保留
    % [~, best_idx] = min(fitness);
    % new_pop = population(best_idx,:);
    [~, sort_idx] = sort(fitness);
    elite_num = 5;
    new_pop = population(sort_idx(1:elite_num), :);
    
    % 补全剩余个体（注意起始下标变化）
    for i = elite_num+1:pop_size
        candidates = randperm(pop_size, 3);
        [~, idx] = min(fitness(candidates));
        new_pop(i,:) = population(candidates(idx),:);
    end
    
    % 选择操作（锦标赛选择）
    for i = 2:pop_size
        candidates = randperm(pop_size, round(gen/20 + 2));
        [~, idx] = min(fitness(candidates));
        new_pop(i,:) = population(candidates(idx),:);
    end

    % 交叉操作
    for i = 1:2:pop_size-1
        if rand() < cross_rate
            cp = randi([1, size(targets,1)-1]);
            temp = new_pop(i, cp+1:end);
            new_pop(i, cp+1:end) = new_pop(i+1, cp+1:end);
            new_pop(i+1, cp+1:end) = temp;
        end
    end

    % 变异操作
    % 修改变异操作（保证变异后仍全覆盖）
    for i = 2:pop_size
        if rand() < mutation_rate
            % 随机选择一个目标点，重新分配
            pos = randi(size(targets,1));
            old_val = new_pop(i,pos);
            new_val = randi(m);
            
            % 确保新值不会导致该点无人覆盖
            while sum(new_pop(i,:)==old_val) <= 1 && new_val == old_val
                new_val = randi(m);
            end
            new_pop(i,pos) = new_val;
        end
    end

     population = new_pop;
end

%% 提取最优解
[~, best_idx] = min(fitness);
best_assignment = population(best_idx,:);
new_pop(1,:) = population(best_idx,:);  % 保留最优个体

%% 输出结果
fprintf('\n最优分配方案：\n');
paths_all = cell(m,1);
for uav = 1:m
    idx = find(best_assignment == uav);
    if isempty(idx)
        fprintf('无人机 %d: 无任务\n', uav);
        paths_all{uav} = [];
        continue;
    end

    pts = [targets(idx,:)];
    % disp('pts');
    % disp(pts);

    dist_matrix = pdist2(pts, pts);
    [~, path_order] = tsp_ga(dist_matrix);

    % disp(path_order);

    path_coords = pts(path_order, :);
    path_coords = [start_pos; path_coords; start_pos]; % 最后加上返回起点

    disp(path_coords);

    paths_all{uav} = path_coords;

    % 打印路径
    fprintf('无人机 %d 路径：Start', uav);
    for i = 1:length(path_order)
        point_idx = path_order(i);   % 当前访问的是 pts 中的第几个点
        % disp('point_idx');
        % disp(point_idx);

        [row, ~] = find(targets == pts(point_idx, :)); % 在原始 targets 中找匹配行
        target_idx = row(1);       % 得到原始目标点编号
        fprintf(' -> T%d', target_idx);
    end
    % 最后加上返回起点
    fprintf(' -> Start');
    fprintf('，总距离：%.2f m\n', tsp_cost(path_order, dist_matrix));
end

%% 绘制路径图
figure;
hold on;
plot(start_pos(1), start_pos(2), 'ks', 'MarkerFaceColor', 'k'); % 起点
text(start_pos(1)+20, start_pos(2)+20, 'Start', 'FontSize', 10);

colors = lines(m);
for uav = 1:m
    path = paths_all{uav};
    if ~isempty(path)
        % 主路径绘制
        plot(path(:,1), path(:,2), '-o', 'Color', colors(uav,:), 'LineWidth', 1.5);
        
        % 返航路径绘制（从最后一个点回到起点）
        last_point = path(end, :);
        plot([last_point(1), start_pos(1)], [last_point(2), start_pos(2)], ...
             '-.', 'Color', colors(uav,:), 'LineWidth', 1);
        
        % 标签
        text(last_point(1)+10, last_point(2)+10, sprintf('UAV%d', uav), 'FontSize', 10);
    end
end

% 目标点绘制
for t = 1:size(targets,1)
    plot(targets(t,1), targets(t,2), 'ro', 'MarkerFaceColor', 'r');
    text(targets(t,1)+20, targets(t,2)+20, sprintf('T%d', t), 'FontSize', 10);
end

title('无人机路径规划结果（含返航路径）');
xlabel('X 坐标 (m)');
ylabel('Y 坐标 (m)');
grid on;
axis equal;
hold off;

%% 辅助函数：计算TSP路径成本，用于生成每个无人机的路径
% 给定一个访问目标点的顺序 order(1*n) 和所有点之间的两两距离矩阵 dist_matrix(n*n)
function cost = tsp_cost(order, dist_matrix)
    n = length(order);
    cost = 0;
    for i = 1:n-1
        cost = cost + dist_matrix(order(i), order(i+1));
    end
    cost = cost + dist_matrix(order(end), order(1)); % 返回起点
end
    
%% 辅助函数：简单TSP求解（用于最终路径）  
function [min_cost, best_path] = tsp_ga(dist_matrix)
    n = size(dist_matrix,1);
    
    % Step 1: 最近邻法构造初始路径
    visited = false(1, n);
    path = zeros(1, n);
    current = 1;
    path(1) = current;
    visited(current) = true;

    for i = 2:n
        dists = dist_matrix(current, :);
        dists(visited) = inf;  % 排除已访问节点
        [~, next] = min(dists);
        path(i) = next;
        visited(next) = true;
        current = next;
    end

    % Step 2: 一次性 2-opt 改进
    best_path = path;
    min_cost = tsp_cost(best_path, dist_matrix);

    for i = 2:n-2
        for j = i+1:n-1
            new_path = best_path;
            new_path(i:j) = best_path(j:-1:i);
            new_cost = tsp_cost(new_path, dist_matrix);
            if new_cost < min_cost
                best_path = new_path;
                min_cost = new_cost;
            end
        end
    end
end

%% 辅助函数：通信，最小间距，最长时间检查
function valid = check_communication(paths, speed)
    valid = true;
    num_uavs = length(paths);
    time_step = 2;  % 检查间隔（秒）
    max_time = 600; % 最大任务时间
    
    for t = 0:time_step:max_time % 遍历时间点
        pos = zeros(num_uavs, 2);
        for uav = 1:num_uavs
            if isempty(paths{uav})
                continue;
            end
            % 计算当前飞行距离
            dist = speed * t; % 已经飞行的距离
            path = paths{uav};
            cum_dist = cumsum([0; sqrt(sum(diff(path).^2, 2))]); % 起点到各个点的距离
            idx = find(cum_dist >= dist, 1); % 找到下一个点
            if isempty(idx)
                pos(uav,:) = path(end,:);
            elseif idx == 1 % 插值得到当前位置
                ratio = dist / cum_dist(2);
                pos(uav,:) = path(1,:) + ratio * (path(2,:) - path(1,:));
            else
                if t == max_time
                    valid = false;
                end
                ratio = (dist - cum_dist(idx-1)) / (cum_dist(idx) - cum_dist(idx-1));
                pos(uav,:) = path(idx-1,:) + ratio * (path(idx,:) - path(idx-1,:));
            end
        end
        
        % 检查间距
        D = pdist2(pos, pos);
        D(eye(num_uavs) == 1) = Inf;  % 忽略自身
        if any(D(:) < 50) || any(D(:) > 1000)
            valid = false;
            return;
        end
    end
end

%% 根据任务分配方案 assignment，计算每个无人机访问目标点的最优路径和总距离
function [total_dist, paths] = optimize_paths(assignment, targets, start_pos)
    m = max(assignment); % 无人机数量
    total_dist = 0;
    paths = cell(m, 1);

    for uav = 1:m
        idx = find(assignment == uav);
        if isempty(idx)
            paths{uav} = [];
            continue;
        end

        pts = [start_pos; targets(idx,:)]; % 加上起点
        dist_matrix = pdist2(pts, pts);

        % 使用 tsp_ga 函数求解 TSP 路径
        [~, path_order] = tsp_ga(dist_matrix);
        path_coords = pts(path_order, :);
        path_coords = [path_coords; start_pos]; % 最后回到起点
        paths{uav} = path_coords;

        % 计算路径长度
        cost = tsp_cost(path_order, dist_matrix);
        total_dist = total_dist + cost;
    end
end

